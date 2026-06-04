#!/usr/bin/env python3
"""Flash pre-built firmware onto the ESP32-S3.

Usage:
    python flash.py              # auto-detect port
    python flash.py -p COM3      # explicit port
    python flash.py -p COM3 --baud 115200
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

FLASH_DIR = Path(__file__).parent
_ESP32S3_USB_VID = 0x303A
_ESP32S3_USB_PID = 0x1001


def auto_detect_port() -> str:
    try:
        import serial.tools.list_ports
    except ImportError:
        sys.exit("ERROR: pyserial not installed.\n  pip install pyserial")

    matches = [
        p.device
        for p in serial.tools.list_ports.comports()
        if p.vid == _ESP32S3_USB_VID and p.pid == _ESP32S3_USB_PID
    ]
    if not matches:
        sys.exit(
            "ERROR: no ESP32-S3 found (VID 0x303A / PID 0x1001).\n"
            "  Check the USB connection or specify the port with -p."
        )
    if len(matches) > 1:
        ports = ", ".join(matches)
        sys.exit(
            f"ERROR: multiple ESP32-S3 devices found ({ports}).\n"
            "  Specify the port with -p."
        )
    return matches[0]


def find_esptool() -> list[str]:
    import shutil

    # 1. python -m esptool (works whenever esptool is pip-installed)
    try:
        subprocess.run(
            [sys.executable, "-m", "esptool", "version"],
            check=True, capture_output=True,
        )
        return [sys.executable, "-m", "esptool"]
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass

    # 2. esptool.py or esptool on PATH
    for name in ("esptool.py", "esptool"):
        if shutil.which(name):
            return [name]

    # 3. ESP-IDF Python virtualenv under ~/.espressif
    espressif = Path.home() / ".espressif" / "python_env"
    if espressif.is_dir():
        for env_dir in sorted(espressif.glob("idf*_env"), reverse=True):
            python = env_dir / "Scripts" / "python.exe"
            if not python.exists():
                python = env_dir / "bin" / "python"
            if python.exists():
                try:
                    subprocess.run(
                        [str(python), "-m", "esptool", "version"],
                        check=True, capture_output=True,
                    )
                    return [str(python), "-m", "esptool"]
                except (subprocess.CalledProcessError, FileNotFoundError):
                    pass

    raise FileNotFoundError(
        "esptool not found.\n"
        "  Install it with:  pip install esptool"
    )


def load_flasher_args() -> dict:
    args_file = FLASH_DIR / "flasher_args.json"
    if not args_file.exists():
        sys.exit(f"ERROR: flasher_args.json not found in {FLASH_DIR}")
    with args_file.open() as f:
        return json.load(f)


def build_flash_command(esptool: list[str], port: str, baud: int, config: dict) -> list[str]:
    extra    = config["extra_esptool_args"]
    settings = config["flash_settings"]

    cmd = esptool + [
        "--chip",   extra["chip"],
        "--port",   port,
        "--baud",   str(baud),
        "--before", extra["before"],
        "--after",  extra["after"],
        "write_flash",
        "--flash_mode", settings["flash_mode"],
        "--flash_freq", settings["flash_freq"],
        "--flash_size", settings["flash_size"],
    ]

    for offset, rel_path in config["flash_files"].items():
        binary = FLASH_DIR / Path(rel_path)
        if not binary.exists():
            sys.exit(f"ERROR: firmware binary not found: {binary}")
        cmd += [offset, str(binary)]

    return cmd


def main():
    parser = argparse.ArgumentParser(
        description="Flash pre-built ESP32-S3 firmware.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python flash.py               # auto-detect port\n"
            "  python flash.py -p COM3       # explicit port\n"
        ),
    )
    parser.add_argument(
        "-p", "--port",
        default=None,
        help="Serial port (e.g. COM3 or /dev/ttyUSB0). Auto-detected if omitted.",
    )
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate (default: 460800)")
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    print(f"Port: {port}")

    try:
        esptool = find_esptool()
    except FileNotFoundError as e:
        sys.exit(f"ERROR: {e}")

    config = load_flasher_args()
    cmd    = build_flash_command(esptool, port, args.baud, config)

    print(f"Flashing ESP32-S3 at {args.baud} baud...")
    print()

    result = subprocess.run(cmd)
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
