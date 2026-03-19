#include <Arduino.h>
#include <FastAccelStepper.h>
#include <PID_v1.h>
#include "driver/pcnt.h"

// --- Hardware Pin Definitions ---
const int PIN_STEP = 12;
const int PIN_DIR  = 13;
const int PIN_ENA  = 15;
const int PIN_ENC_A = 10;
const int PIN_ENC_B = 11;
const int PIN_LIMIT_NEG_NO = 16;  // NO contact: HIGH=open, LOW=triggered
const int PIN_LIMIT_NEG_NC = 18;  // NC contact: LOW=closed, HIGH=triggered
const int PIN_LIMIT_POS_NO = 17;  // NO contact: HIGH=open, LOW=triggered
const int PIN_LIMIT_POS_NC = 8;   // NC contact: LOW=closed, HIGH=triggered

// --- Mechanical Constants ---
const double SCREW_LEAD_MM = 5.0;
const int STEPS_PER_REV = 5000;
const double ENCODER_RES_MM = 0.005;
const double STEPS_PER_MM = (double)STEPS_PER_REV / SCREW_LEAD_MM;
const long BACKOFF_STEPS = (long)(10.0 * STEPS_PER_MM);

// --- Limit Switch Debouncing ---
const int DEBOUNCE_COUNT = 2;  // consecutive complementary reads required
int negHitCount = 0;
int posHitCount = 0;

// --- Configurable Motion Parameters ---
int homingSpeed = 2000;       // steps/s
int normalSpeed = 4000;       // steps/s
int normalAccel = 10000;      // steps/s^2
double amplitude = 0.5;       // mm
double frequency = 0.1;       // Hz
double jerkMax = 100.0;       // mm/s^3 (for S-curve)
double dutyCycle = 0.9;       // fraction of period at constant velocity
unsigned long startTime;
unsigned long enableTime;     // timestamp when motor was enabled (for telemetry)

// --- Waveform ---
enum WaveType { WAVE_SINE, WAVE_TRAP, WAVE_SCURVE };
WaveType waveType = WAVE_SINE;

// Precomputed waveform segments for trap/scurve
#define MAX_SEGS 7
struct WaveSeg {
    double dur;
    double x0, v0, a0, j;
};
WaveSeg halfSegs[MAX_SEGS];
int numHalfSegs = 0;
double halfDur = 0;

void precomputeWaveform() {
    double s = normalSpeed / STEPS_PER_MM;   // mm/s (constant velocity on flats)
    double F = dutyCycle;

    if (waveType == WAVE_TRAP) {
        // C1 Triangle: parabolic corners, trapezoidal velocity
        double T0 = 8.0 * amplitude / (s * (1.0 + F));
        double dc = (1.0 - F) * T0 / 4.0;  // cap half-width
        double a_cap = s / dc;               // acceleration in caps
        double T_flat = T0 / 2.0 - 2.0 * dc; // flat segment duration

        numHalfSegs = 3;
        // Seg 0: valley exit (parabolic cap, accel up)
        halfSegs[0] = {dc, 0, 0, a_cap, 0};
        double x1 = 0.5 * a_cap * dc * dc;  // = s*dc/2
        // Seg 1: rising flat (constant velocity)
        halfSegs[1] = {T_flat, x1, s, 0, 0};
        double x2 = x1 + s * T_flat;
        // Seg 2: peak entry (parabolic cap, decel)
        halfSegs[2] = {dc, x2, s, -a_cap, 0};
        halfDur = T0 / 2.0;

    } else if (waveType == WAVE_SCURVE) {
        // C2 Rounded Triangle: jerk-limited corners, continuous acceleration
        double T0 = 12.0 * amplitude / (s * (2.0 + F));
        double Tj = (1.0 - F) * T0 / 4.0;  // jerk phase duration
        double J = 2.0 * s / (Tj * Tj);     // jerk magnitude
        double a_peak = J * Tj;              // peak acceleration at corner center
        double T_flat = T0 / 2.0 - 2.0 * Tj;

        numHalfSegs = 3;
        // Seg 0: valley exit (jerk ramp down, accel a_peak -> 0, vel 0 -> s)
        halfSegs[0] = {Tj, 0, 0, a_peak, -J};
        double x1 = 0.5 * a_peak * Tj * Tj + (1.0 / 6.0) * (-J) * Tj * Tj * Tj;
        // Seg 1: rising flat
        halfSegs[1] = {T_flat, x1, s, 0, 0};
        double x2 = x1 + s * T_flat;
        // Seg 2: peak entry (jerk ramp, accel 0 -> -a_peak, vel s -> 0)
        halfSegs[2] = {Tj, x2, s, 0, -J};
        halfDur = T0 / 2.0;
    }
    // WAVE_SINE doesn't need precomputation
}

double computePosition(double t_s) {
    if (waveType == WAVE_SINE) {
        return amplitude * sin(2 * PI * frequency * t_s);
    }

    // Trapezoidal or S-curve
    if (halfDur < 1e-9) return 0;
    double T_period = 2 * halfDur;
    double t_mod = fmod(t_s, T_period);
    if (t_mod < 0) t_mod += T_period;

    bool forward = (t_mod < halfDur);
    double tau = forward ? t_mod : (t_mod - halfDur);

    // Find segment
    double elapsed = 0;
    for (int i = 0; i < numHalfSegs; i++) {
        double segEnd = elapsed + halfSegs[i].dur;
        if (tau < segEnd || i == numHalfSegs - 1) {
            double lt = tau - elapsed;
            double p = halfSegs[i].x0 + halfSegs[i].v0 * lt
                     + 0.5 * halfSegs[i].a0 * lt * lt
                     + (1.0 / 6.0) * halfSegs[i].j * lt * lt * lt;
            return forward ? (-amplitude + p) : (amplitude - p);
        }
        elapsed = segEnd;
    }
    return 0;
}

// --- PID ---
double pidInput = 0.0, pidSetpoint = 0.0, pidOutput = 0.0;
double Kp = 1.0, Ki = 0.0, Kd = 0.0;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);
bool pidEnabled = false;

// --- Direct PCNT encoder on unit 3 (FastAccelStepper uses unit 0) ---
#define ENC_PCNT_UNIT PCNT_UNIT_3
#define ENC_H_LIM  30000
#define ENC_L_LIM -30000

volatile long encoderOverflow = 0;

static void IRAM_ATTR pcnt_overflow_handler(void *arg) {
    uint32_t status;
    pcnt_get_event_status(ENC_PCNT_UNIT, &status);
    if (status & PCNT_EVT_H_LIM) encoderOverflow += ENC_H_LIM;
    if (status & PCNT_EVT_L_LIM) encoderOverflow += ENC_L_LIM;
}

long getEncoderCount() {
    int16_t count;
    pcnt_get_counter_value(ENC_PCNT_UNIT, &count);
    return encoderOverflow + count;
}

void clearEncoderCount() {
    pcnt_counter_pause(ENC_PCNT_UNIT);
    pcnt_counter_clear(ENC_PCNT_UNIT);
    encoderOverflow = 0;
    pcnt_counter_resume(ENC_PCNT_UNIT);
}

void initEncoder() {
    pcnt_config_t ch0 = {};
    ch0.pulse_gpio_num = PIN_ENC_A;
    ch0.ctrl_gpio_num  = PIN_ENC_B;
    ch0.channel        = PCNT_CHANNEL_0;
    ch0.unit           = ENC_PCNT_UNIT;
    ch0.pos_mode       = PCNT_COUNT_DEC;
    ch0.neg_mode       = PCNT_COUNT_INC;
    ch0.lctrl_mode     = PCNT_MODE_REVERSE;
    ch0.hctrl_mode     = PCNT_MODE_KEEP;
    ch0.counter_h_lim  = ENC_H_LIM;
    ch0.counter_l_lim  = ENC_L_LIM;
    pcnt_unit_config(&ch0);

    pcnt_config_t ch1 = {};
    ch1.pulse_gpio_num = PIN_ENC_B;
    ch1.ctrl_gpio_num  = PIN_ENC_A;
    ch1.channel        = PCNT_CHANNEL_1;
    ch1.unit           = ENC_PCNT_UNIT;
    ch1.pos_mode       = PCNT_COUNT_INC;
    ch1.neg_mode       = PCNT_COUNT_DEC;
    ch1.lctrl_mode     = PCNT_MODE_REVERSE;
    ch1.hctrl_mode     = PCNT_MODE_KEEP;
    ch1.counter_h_lim  = ENC_H_LIM;
    ch1.counter_l_lim  = ENC_L_LIM;
    pcnt_unit_config(&ch1);

    pcnt_set_filter_value(ENC_PCNT_UNIT, 100);
    pcnt_filter_enable(ENC_PCNT_UNIT);

    pcnt_event_enable(ENC_PCNT_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(ENC_PCNT_UNIT, PCNT_EVT_L_LIM);
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(ENC_PCNT_UNIT, pcnt_overflow_handler, NULL);

    pcnt_counter_pause(ENC_PCNT_UNIT);
    pcnt_counter_clear(ENC_PCNT_UNIT);
    pcnt_counter_resume(ENC_PCNT_UNIT);
}

// --- Objects ---
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// --- State Machine ---
enum State { IDLE, RUNNING, MOVING, LIMIT_BACKOFF, HOMING, HOMING_BACKOFF };
State state = IDLE;
bool enabled = false;
int8_t homingDir = 0;
String cmdBuffer = "";

void startBackoff(int8_t hitDir) {
    // Wait for forceStop to fully complete before reversing
    while (stepper->isRunning()) {
        delayMicroseconds(10);
    }
    long curPos = stepper->getCurrentPosition();
    long backoffTarget = curPos + (-hitDir * BACKOFF_STEPS);
    stepper->setAcceleration(50000);  // high accel for fast reversal
    stepper->setSpeedInHz(homingSpeed);
    stepper->moveTo(backoffTarget);
}

void sendTelemetry(double targetMM, double pidCorr) {
    double actualMM = getEncoderCount() * ENCODER_RES_MM;
    Serial.print(millis() - enableTime);
    Serial.print(',');
    Serial.print(targetMM, 4);
    Serial.print(',');
    Serial.print(actualMM, 4);
    Serial.print(',');
    Serial.print(stepper->getCurrentPosition());
    Serial.print(',');
    Serial.println(pidCorr, 4);
}

void handleCommand(const String &cmd) {
    if (cmd == "START") {
        if (state == IDLE) {
            if (!enabled) {
                stepper->enableOutputs();
                enabled = true;
                enableTime = millis();
            }
            startTime = millis();  // waveform phase reference only
            stepper->setSpeedInHz(normalSpeed);
            stepper->setAcceleration(normalAccel);
            precomputeWaveform();
            if (pidEnabled) {
                myPID.SetMode(AUTOMATIC);
                pidOutput = 0.0;
            }
            state = RUNNING;
            Serial.println("OK:STARTED");
        }
    } else if (cmd == "STOP") {
        stepper->forceStop();
        if (state == RUNNING) {
            myPID.SetMode(MANUAL);
        }
        state = IDLE;
        Serial.println("OK:STOPPED");
    } else if (cmd == "ENABLE") {
        if (!enabled) {
            stepper->enableOutputs();
            enabled = true;
            enableTime = millis();
            Serial.println("OK:ENABLED");
        }
    } else if (cmd == "DISABLE") {
        stepper->forceStop();
        stepper->disableOutputs();
        enabled = false;
        myPID.SetMode(MANUAL);
        state = IDLE;
        Serial.println("OK:DISABLED");
    } else if (cmd == "HOME-") {
        if (!enabled) {
            stepper->enableOutputs();
            enabled = true;
        }
        if (state == IDLE) {
            state = HOMING;
            homingDir = -1;
            stepper->setSpeedInHz(homingSpeed);
            stepper->runBackward();
            Serial.println("OK:HOMING_NEG");
        }
    } else if (cmd == "HOME+") {
        if (!enabled) {
            stepper->enableOutputs();
            enabled = true;
        }
        if (state == IDLE) {
            state = HOMING;
            homingDir = 1;
            stepper->setSpeedInHz(homingSpeed);
            stepper->runForward();
            Serial.println("OK:HOMING_POS");
        }
    } else if (cmd == "STATUS") {
        switch (state) {
            case IDLE:           Serial.println("STATUS:IDLE"); break;
            case RUNNING:        Serial.println("STATUS:RUNNING"); break;
            case MOVING:         Serial.println("STATUS:MOVING"); break;
            case LIMIT_BACKOFF:  Serial.println("STATUS:LIMIT_BACKOFF"); break;
            case HOMING:         Serial.println("STATUS:HOMING"); break;
            case HOMING_BACKOFF: Serial.println("STATUS:HOMING_BACKOFF"); break;
        }
    } else if (cmd == "POS") {
        long steps = stepper->getCurrentPosition();
        double stepMM = steps / STEPS_PER_MM;
        long enc = getEncoderCount();
        double encMM = enc * ENCODER_RES_MM;
        Serial.print("POS:");
        Serial.print(stepMM, 4);
        Serial.print(",");
        Serial.println(encMM, 4);
    } else if (cmd.startsWith("MOVE:")) {
        if (!enabled) {
            stepper->enableOutputs();
            enabled = true;
        }
        if (state == IDLE) {
            double mm = cmd.substring(5).toDouble();
            long targetSteps = (long)(mm * STEPS_PER_MM);
            long curSteps = stepper->getCurrentPosition();
            long newTarget = curSteps + targetSteps;
            state = MOVING;
            stepper->setSpeedInHz(normalSpeed);
            stepper->setAcceleration(normalAccel);
            stepper->moveTo(newTarget);
            Serial.print("OK:MOVING:");
            Serial.println(mm, 4);
        }
    } else if (cmd.startsWith("VEL:")) {
        double mm_s = cmd.substring(4).toDouble();
        if (mm_s > 0) {
            normalSpeed = (int)(mm_s * STEPS_PER_MM);
            stepper->setSpeedInHz(normalSpeed);
            Serial.print("OK:VEL:");
            Serial.println(mm_s, 4);
        }
    } else if (cmd.startsWith("ACCEL:")) {
        double mm_s2 = cmd.substring(6).toDouble();
        if (mm_s2 > 0) {
            normalAccel = (int)(mm_s2 * STEPS_PER_MM);
            stepper->setAcceleration(normalAccel);
            Serial.print("OK:ACCEL:");
            Serial.println(mm_s2, 4);
        }
    } else if (cmd.startsWith("AMP:")) {
        double val = cmd.substring(4).toDouble();
        if (val > 0) {
            amplitude = val;
            Serial.print("OK:AMP:");
            Serial.println(amplitude, 4);
        }
    } else if (cmd.startsWith("FREQ:")) {
        double val = cmd.substring(5).toDouble();
        if (val > 0) {
            frequency = val;
            Serial.print("OK:FREQ:");
            Serial.println(frequency, 4);
        }
    } else if (cmd.startsWith("JERK:")) {
        double val = cmd.substring(5).toDouble();
        if (val > 0) {
            jerkMax = val;
            Serial.print("OK:JERK:");
            Serial.println(jerkMax, 4);
        }
    } else if (cmd.startsWith("DUTY:")) {
        double val = cmd.substring(5).toDouble();
        if (val > 0 && val < 1.0) {
            dutyCycle = val;
            Serial.print("OK:DUTY:");
            Serial.println(dutyCycle, 4);
        }
    } else if (cmd == "WAVEFORM:SINE") {
        waveType = WAVE_SINE;
        Serial.println("OK:WAVEFORM:SINE");
    } else if (cmd == "WAVEFORM:TRAP") {
        waveType = WAVE_TRAP;
        precomputeWaveform();
        Serial.println("OK:WAVEFORM:TRAP");
    } else if (cmd == "WAVEFORM:SCURVE") {
        waveType = WAVE_SCURVE;
        precomputeWaveform();
        Serial.println("OK:WAVEFORM:SCURVE");
    } else if (cmd.startsWith("PID:")) {
        String params = cmd.substring(4);
        int c1 = params.indexOf(',');
        int c2 = params.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > c1) {
            Kp = params.substring(0, c1).toDouble();
            Ki = params.substring(c1 + 1, c2).toDouble();
            Kd = params.substring(c2 + 1).toDouble();
            myPID.SetTunings(Kp, Ki, Kd);
            Serial.print("OK:PID:");
            Serial.print(Kp, 4); Serial.print(",");
            Serial.print(Ki, 4); Serial.print(",");
            Serial.println(Kd, 4);
        }
    } else if (cmd == "PID_ON") {
        pidEnabled = true;
        myPID.SetMode(AUTOMATIC);
        pidOutput = 0.0;
        Serial.println("OK:PID_ON");
    } else if (cmd == "PID_OFF") {
        pidEnabled = false;
        myPID.SetMode(MANUAL);
        pidOutput = 0.0;
        Serial.println("OK:PID_OFF");
    } else if (cmd == "ZERO") {
        stepper->setCurrentPosition(0);
        clearEncoderCount();
        Serial.println("OK:ZEROED");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    engine.init();
    stepper = engine.stepperConnectToPin(PIN_STEP);

    if (stepper) {
        stepper->setDirectionPin(PIN_DIR, true, 0);
        stepper->setEnablePin(PIN_ENA);
        stepper->setAcceleration(normalAccel);
        stepper->setSpeedInHz(normalSpeed);
    } else {
        Serial.println("ERROR:STEPPER_INIT_FAILED");
        while(1);
    }

    stepper->setCurrentPosition(0);
    initEncoder();

    pinMode(PIN_LIMIT_NEG_NO, INPUT_PULLUP);
    pinMode(PIN_LIMIT_NEG_NC, INPUT_PULLUP);
    pinMode(PIN_LIMIT_POS_NO, INPUT_PULLUP);
    pinMode(PIN_LIMIT_POS_NC, INPUT_PULLUP);

    myPID.SetOutputLimits(-2.0, 2.0);
    myPID.SetSampleTime(1);
    myPID.SetMode(MANUAL);

    Serial.println("READY");
}

void loop() {
    // --- Serial command parsing ---
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmdBuffer.trim();
            if (cmdBuffer.length() > 0) {
                handleCommand(cmdBuffer);
                cmdBuffer = "";
            }
        } else {
            cmdBuffer += c;
        }
    }

    // --- Limit switch check: require NO=LOW and NC=HIGH (both contacts agree) ---
    if (state == MOVING || state == RUNNING || state == HOMING) {
        bool negRaw = (digitalRead(PIN_LIMIT_NEG_NO) == LOW) && (digitalRead(PIN_LIMIT_NEG_NC) == HIGH);
        bool posRaw = (digitalRead(PIN_LIMIT_POS_NO) == LOW) && (digitalRead(PIN_LIMIT_POS_NC) == HIGH);

        negHitCount = negRaw ? (negHitCount + 1) : 0;
        posHitCount = posRaw ? (posHitCount + 1) : 0;

        bool negHit = (negHitCount >= DEBOUNCE_COUNT);
        bool posHit = (posHitCount >= DEBOUNCE_COUNT);

        if (negHit || posHit) {
            negHitCount = 0;
            posHitCount = 0;
            int8_t hitDir = negHit ? -1 : 1;
            stepper->forceStop();

            if (state == HOMING) {
                bool correctLimit = (homingDir == hitDir);
                if (correctLimit) {
                    Serial.println(negHit ? "LIMIT:NEG" : "LIMIT:POS");
                    state = HOMING_BACKOFF;
                } else {
                    Serial.println(negHit ? "ALARM:LIMIT_NEG" : "ALARM:LIMIT_POS");
                    state = LIMIT_BACKOFF;
                }
            } else {
                Serial.println(negHit ? "ALARM:LIMIT_NEG" : "ALARM:LIMIT_POS");
                state = LIMIT_BACKOFF;
            }
            startBackoff(hitDir);
        }
    } else {
        negHitCount = 0;
        posHitCount = 0;
    }

    // --- Telemetry: send position data in all states when enabled ---
    if (enabled) {
        double stepMM = stepper->getCurrentPosition() / STEPS_PER_MM;

        switch (state) {
        case RUNNING: {
            double t = (millis() - startTime) / 1000.0;
            double targetMM = computePosition(t);
            double actualMM = getEncoderCount() * ENCODER_RES_MM;

            double commandMM = targetMM;
            if (pidEnabled) {
                pidSetpoint = targetMM;
                pidInput = actualMM;
                myPID.Compute();
                commandMM = targetMM + pidOutput;
            }

            long targetSteps = (long)(commandMM * STEPS_PER_MM);
            stepper->moveTo(targetSteps);

            sendTelemetry(targetMM, pidEnabled ? pidOutput : 0.0);
            delay(1);
            break;
        }

        case MOVING:
            if (!stepper->isRunning()) {
                state = IDLE;
                Serial.println("OK:MOVE_DONE");
            }
            sendTelemetry(stepMM, 0.0);
            delay(1);
            break;

        case LIMIT_BACKOFF:
            if (!stepper->isRunning()) {
                stepper->setSpeedInHz(normalSpeed);
                stepper->setAcceleration(normalAccel);
                state = IDLE;
                Serial.println("OK:BACKOFF_DONE");
            }
            sendTelemetry(stepMM, 0.0);
            delay(1);
            break;

        case HOMING:
            sendTelemetry(stepMM, 0.0);
            delay(1);
            break;

        case HOMING_BACKOFF:
            if (!stepper->isRunning()) {
                stepper->setCurrentPosition(0);
                clearEncoderCount();
                stepper->setSpeedInHz(normalSpeed);
                stepper->setAcceleration(normalAccel);
                state = IDLE;
                Serial.println(homingDir == -1 ? "OK:HOMED_NEG" : "OK:HOMED_POS");
            }
            sendTelemetry(stepMM, 0.0);
            delay(1);
            break;

        case IDLE:
            sendTelemetry(stepMM, 0.0);
            delay(10);  // slower rate when idle (100 Hz vs 1000 Hz)
            break;

        }  // end switch
    } else {
        // Not enabled — just idle
        delay(10);
    }
}
