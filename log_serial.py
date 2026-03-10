"""
Serial CSV logger for ESP32 stepper controller.
Captures serial telemetry to rotating CSV files grouped by session.

Folder structure:
  logs/<session_id>/
    <session_id>_meta.json
    <session_id>_0001.csv
    <session_id>_0002.csv
    ...

Can be used standalone (reads from serial directly) or as a module
imported by the GUI (receives lines via write_line()).

Standalone usage: python log_serial.py [PORT] [MAX_MB]
"""
import json
import os
import time
from datetime import datetime

HEADER = "time_ms,target_mm,actual_mm,step_pos,pid_correction\n"
DEFAULT_MAX_BYTES = 8_000_000  # 8 MB


class SessionLogger:
    """Writes CSV telemetry data to rotating files within a session folder."""

    def __init__(self, base_dir="logs", max_bytes=DEFAULT_MAX_BYTES):
        self.base_dir = base_dir
        self.max_bytes = max_bytes
        self.session_id = None
        self.session_dir = None
        self._file = None
        self._file_num = 0
        self.total_samples = 0
        self.start_time = None

    def begin(self, session_id=None):
        """Start a new logging session. Creates subfolder and metadata."""
        self.session_id = session_id or datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(self.base_dir, self.session_id)
        os.makedirs(self.session_dir, exist_ok=True)

        self.start_time = time.time()
        self._file_num = 0
        self.total_samples = 0

        # Write session metadata
        meta = {
            "session_id": self.session_id,
            "unix_timestamp": self.start_time,
            "iso_time": datetime.fromtimestamp(self.start_time).isoformat(),
            "max_file_bytes": self.max_bytes,
        }
        meta_path = os.path.join(self.session_dir, f"{self.session_id}_meta.json")
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)

        self._open_next_file()
        return self.session_id

    def write_line(self, line):
        """Write a single CSV data line. Auto-rotates files at size limit."""
        if self._file is None:
            return
        row = line.rstrip('\n') + '\n'
        self._file.write(row)
        self._file.flush()
        self.total_samples += 1

        # Check actual file size on disk (handles Windows \r\n correctly)
        try:
            file_size = os.path.getsize(self._file.name)
        except OSError:
            file_size = self._file.tell()

        if file_size >= self.max_bytes:
            self._open_next_file()

    def end(self):
        """Close the current session."""
        if self._file:
            self._file.close()
            self._file = None
        elapsed = time.time() - self.start_time if self.start_time else 0
        result = {
            "session_id": self.session_id,
            "total_samples": self.total_samples,
            "num_files": self._file_num,
            "elapsed_s": elapsed,
        }
        self.session_id = None
        return result

    @property
    def is_active(self):
        return self._file is not None

    @property
    def current_filename(self):
        if self._file and not self._file.closed:
            return self._file.name
        return None

    def _open_next_file(self):
        if self._file:
            self._file.close()
        self._file_num += 1
        filename = os.path.join(
            self.session_dir,
            f"{self.session_id}_{self._file_num:04d}.csv"
        )
        self._file = open(filename, 'w', buffering=1)  # Line-buffered
        self._file.write(HEADER)
        self._file.flush()
        return filename


# --- Standalone mode: read directly from serial ---
def main():
    import sys
    import serial
    import serial.tools.list_ports

    def find_port():
        for p in serial.tools.list_ports.comports():
            if "USB" in p.description or "ESP" in p.description or "JTAG" in p.description:
                return p.device
        ports = serial.tools.list_ports.comports()
        return ports[0].device if ports else None

    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    max_mb = float(sys.argv[2]) if len(sys.argv) > 2 else 8

    if not port:
        print("No serial port found. Specify one: python log_serial.py COM5")
        sys.exit(1)

    logger = SessionLogger(max_bytes=int(max_mb * 1_000_000))

    print(f"Port:      {port}")
    print(f"Max file:  {max_mb:.0f} MB")

    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    session_id = logger.begin()
    print(f"Session:   {session_id}")
    print(f"Log dir:   {os.path.abspath(logger.session_dir)}")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            if not line[0].isdigit():
                continue
            logger.write_line(line)
            if logger.total_samples % 5000 == 0:
                elapsed = time.time() - logger.start_time
                print(f"  {logger.total_samples} samples | "
                      f"file {logger._file_num} | "
                      f"{elapsed:.0f}s")
    except KeyboardInterrupt:
        pass

    result = logger.end()
    ser.close()
    print(f"\nSession {result['session_id']} complete:")
    print(f"  {result['total_samples']} samples in {result['elapsed_s']:.1f}s")
    print(f"  {result['num_files']} file(s)")


if __name__ == "__main__":
    main()
