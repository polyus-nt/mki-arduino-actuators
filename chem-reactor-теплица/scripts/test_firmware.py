"""Standalone chemreactor actuator tests; no other repository required."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="chem-actuator-") as directory:
        binary = Path(directory) / "actuator.exe"
        subprocess.run([args.cxx, "-std=c++11", "-Wall", "-Wextra",
                        "-I", str(root / "tests/actuator-stubs"),
                        str(root / "tests/actuator_board_test.cpp"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        subprocess.run([args.cxx, "-std=c++11", "-Wall", "-Wextra",
                        "-I", str(root / "tests/actuator-stubs"),
                        '-DACTUATOR_TEST_HEADER="../arduino-actuator/src/ActuatorTestInput.h"',
                        str(root / "tests/console_test.cpp"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
