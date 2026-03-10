"""
Serial controller for ESP32 stepper system.
Sends commands and reads telemetry over USB serial.

Commands:
  START      - Begin oscillation, responds "OK:STARTED"
  STOP       - Stop motion,       responds "OK:STOPPED"
  ENABLE     - Energize motor,    responds "OK:ENABLED"
  DISABLE    - De-energize motor, responds "OK:DISABLED"
  HOME-      - Home to neg limit, responds "OK:HOMING_NEG" then "OK:HOMED_NEG"
  HOME+      - Home to pos limit, responds "OK:HOMING_POS" then "OK:HOMED_POS"
  MOVE:<mm>  - Relative move,     responds "OK:MOVING:<mm>" then "OK:MOVE_DONE"
  VEL:<mm/s> - Set velocity,      responds "OK:VEL:<val>"
  ACCEL:<mm/s²> - Set acceleration, responds "OK:ACCEL:<val>"
  AMP:<mm>   - Set amplitude,     responds "OK:AMP:<val>"
  FREQ:<Hz>  - Set frequency,     responds "OK:FREQ:<val>"
  DUTY:<frac> - Set duty cycle,    responds "OK:DUTY:<val>"
  PID:<Kp>,<Ki>,<Kd> - Set PID gains, responds "OK:PID:<Kp>,<Ki>,<Kd>"
  PID_ON     - Enable PID,        responds "OK:PID_ON"
  PID_OFF    - Disable PID,       responds "OK:PID_OFF"
  STATUS     - Query state,       responds "STATUS:<state>"
  POS        - Query position,    responds "POS:<step_mm>,<enc_mm>"
"""
import serial
import serial.tools.list_ports
import threading
import time


def find_esp32_port():
    """Auto-detect ESP32-S3 USB serial port."""
    for p in serial.tools.list_ports.comports():
        if "USB" in p.description or "ESP" in p.description or "JTAG" in p.description:
            return p.device
    ports = serial.tools.list_ports.comports()
    if ports:
        return ports[0].device
    return None


class StepperController:
    """Manages serial connection to the ESP32 stepper controller."""

    def __init__(self, port=None, baud=115200):
        self.port = port or find_esp32_port()
        self.baud = baud
        self.ser = None
        self._reader_thread = None
        self._running = False
        self._line_callback = None
        self._lock = threading.Lock()

    def connect(self):
        """Open serial port and start reader thread."""
        if self.ser and self.ser.is_open:
            return
        if not self.port:
            raise RuntimeError("No serial port found. Specify one explicitly.")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2)  # Wait for ESP32 boot
        self.ser.reset_input_buffer()
        self._running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def disconnect(self):
        """Stop reader thread and close serial port."""
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
            self._reader_thread = None
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None

    def set_line_callback(self, callback):
        """Set a callback function(line: str) for each received line."""
        self._line_callback = callback

    def send_command(self, cmd):
        """Send a command string to the ESP32."""
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.write((cmd.strip() + '\n').encode())

    def start(self):
        self.send_command("START")

    def stop(self):
        self.send_command("STOP")

    def enable(self):
        self.send_command("ENABLE")

    def disable(self):
        self.send_command("DISABLE")

    def home_neg(self):
        self.send_command("HOME-")

    def home_pos(self):
        self.send_command("HOME+")

    def move(self, mm):
        self.send_command(f"MOVE:{mm}")

    def set_velocity(self, mm_s):
        self.send_command(f"VEL:{mm_s}")

    def set_acceleration(self, mm_s2):
        self.send_command(f"ACCEL:{mm_s2}")

    def set_amplitude(self, mm):
        self.send_command(f"AMP:{mm}")

    def set_frequency(self, hz):
        self.send_command(f"FREQ:{hz}")

    def set_jerk(self, mm_s3):
        self.send_command(f"JERK:{mm_s3}")

    def set_duty_cycle(self, fraction):
        self.send_command(f"DUTY:{fraction}")

    def set_waveform(self, wave_type):
        """Set waveform type: 'SINE', 'TRAP', or 'SCURVE'."""
        self.send_command(f"WAVEFORM:{wave_type}")

    def set_pid(self, kp, ki, kd):
        self.send_command(f"PID:{kp},{ki},{kd}")

    def pid_on(self):
        self.send_command("PID_ON")

    def pid_off(self):
        self.send_command("PID_OFF")

    def status(self):
        self.send_command("STATUS")

    def pos(self):
        self.send_command("POS")

    def _read_loop(self):
        """Background thread: read lines from serial and dispatch."""
        while self._running:
            try:
                raw = self.ser.readline()
                if raw:
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if line and self._line_callback:
                        self._line_callback(line)
            except (serial.SerialException, OSError):
                break


if __name__ == "__main__":
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else None
    ctrl = StepperController(port)

    def on_line(line):
        print(f"  <- {line}")

    ctrl.set_line_callback(on_line)
    ctrl.connect()
    print(f"Connected to {ctrl.port}. Commands: START, STOP, HOME-, HOME+, MOVE:<mm>, STATUS, POS, quit")

    try:
        while True:
            cmd = input("> ").strip().upper()
            if cmd == "QUIT":
                break
            if cmd:
                ctrl.send_command(cmd)
    except (KeyboardInterrupt, EOFError):
        pass

    ctrl.stop()
    time.sleep(0.5)
    ctrl.disconnect()
    print("Disconnected.")
