import subprocess
from pathlib import Path

class BetaflightSITL:
    def __init__(self, executable_path: str | Path):
        self.executable_path = executable_path
        self.process = None

    def __enter__(self):
        self.process = subprocess.Popen([self.executable_path])
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.process:
            self.process.terminate()
            self.process.wait()
            self.process = None