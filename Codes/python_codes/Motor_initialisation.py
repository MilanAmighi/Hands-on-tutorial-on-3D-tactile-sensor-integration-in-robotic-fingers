import time
import sys
import argparse
from pathlib import Path

# Project-root imports
ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move motor to position 10, wait 5s, then return to position 0.")
    parser.add_argument(
        "-p",
        "--com-port",
        default=None,
        help="Serial COM port (default: auto-detect ESP32-S3).",
    )
    parser.add_argument(
        "-s",
        "--speed",
        type=int,
        default=250,
        help="Motor speed for both moves (default: 250).",
    )
    parser.add_argument(
        "-a",
        "--acc",
        type=int,
        default=250,
        help="Motor acceleration for both moves (default: 250).",
    )
    parser.add_argument(
        "-f",
        "--burst-frequency",
        type=int,
        default=200,
        help="Burst force thread frequency in Hz (default: 200).",
    )
    parser.add_argument(
        "-g",
        "--debug",
        action="store_true",
        help="Print each received frame as-is before decoding.",
    )
    return parser.parse_args()


def run_test(args: argparse.Namespace) -> None:
    if args.speed < 0:
        print("Invalid --speed, using default 250.")
        args.speed = 250
    if args.acc < 0:
        print("Invalid --acc, using default 250.")
        args.acc = 250
    if args.burst_frequency < 1:
        print("Invalid --burst-frequency, using default 200 Hz.")
        args.burst_frequency = 200

    from SerialLibrary.lib_esp32 import espDriver

    with espDriver(comPort=args.com_port) as esp:
        esp.debug_frames = args.debug
        esp.startThreadBurstForce(frequency=args.burst_frequency)
        time.sleep(1)

        # --- Move to position 1800 ---
        print(f"Moving to position 1800 (speed={args.speed}, acc={args.acc})...")
        esp.sendCommandMotor(position=1800, speed=args.speed, acc=args.acc)

        status = esp.acq_status
        if status is not None and (
            status.get("imu_last_err", 0) != 0
            or status.get("imu_last_read_ok", 1) == 0
            or status.get("force_last_read_ok", 1) == 0
        ):
            print(
                "ERROR: "
                f"imu_last_err={status.get('imu_last_err', 0)} "
                f"imu_ok={status.get('imu_last_read_ok', 0)} "
                f"force_ok={status.get('force_last_read_ok', 0)}"
            )

        # --- Wait 5 seconds ---
        print("Waiting 5 seconds...")
        time.sleep(5)

        # --- Return to position 1500 ---
        print(f"Moving to position 1500 (speed={args.speed}, acc={args.acc})...")
        esp.sendCommandMotor(position=1500, speed=args.speed, acc=args.acc)

        status = esp.acq_status
        if status is not None and (
            status.get("imu_last_err", 0) != 0
            or status.get("imu_last_read_ok", 1) == 0
            or status.get("force_last_read_ok", 1) == 0
        ):
            print(
                "ERROR: "
                f"imu_last_err={status.get('imu_last_err', 0)} "
                f"imu_ok={status.get('imu_last_read_ok', 0)} "
                f"force_ok={status.get('force_last_read_ok', 0)}"
            )

        print("Waiting 5 seconds...")
        time.sleep(5)
        
        print("Done")


if __name__ == "__main__":
    run_test(parse_args())