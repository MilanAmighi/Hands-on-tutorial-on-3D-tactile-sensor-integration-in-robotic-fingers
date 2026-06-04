import time
import sys
import argparse
from pathlib import Path

# Make project-root imports 
ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from SerialLibrary.lib_esp32 import espDriver


def _fmt_force(value: float) -> str:
    """
    Formats a floating-point force value into a fixed-width string.
    The string is 6 characters wide, signed, and has 2 decimal places.
    Example: 3.14159 -> "+03.14"
    :param value: The force value to format.
    :return: A formatted string.
    """
    return f"{float(value):+06.2f}"

def _render_force_table(
    force_vectors,
    gravity_raw_lsb: tuple[int, int, int] | None = None,
    acq_status: dict | None = None,
    peripheral: dict | None = None,
) -> str:
    """
    Renders a formatted text table of sensor data for printing to the console.
    It displays force vectors for up to 8 sensors, the raw gravity vector from the IMU,
    and data from peripherals like the joystick, potentiometer, and motor.
    :param force_vectors: A list or NumPy array of 3D force vectors.
    :param gravity_raw_lsb: A tuple of (gx, gy, gz) raw IMU values.
    :param acq_status: A dictionary with acquisition status information (not used in this version).
    :param peripheral: A dictionary containing joystick, potentiometer, and motor data.
    :return: A multi-line string representing the formatted table.
    """
    col_count = 8
    headers = [f"S{i}" for i in range(col_count)]

    rows = {"Fx": [], "Fy": [], "Fz": []}
    for col in range(col_count):
        if col < len(force_vectors):
            fx, fy, fz = force_vectors[col]
            rows["Fx"].append(_fmt_force(fx))
            rows["Fy"].append(_fmt_force(fy))
            rows["Fz"].append(_fmt_force(fz))
        else:
            rows["Fx"].append("   n/a")
            rows["Fy"].append("   n/a")
            rows["Fz"].append("   n/a")

    header_line = "     " + " ".join([f"{h:>7s}" for h in headers])
    fx_line = "Fx | " + " ".join([f"{v:>7s}" for v in rows["Fx"]])
    fy_line = "Fy | " + " ".join([f"{v:>7s}" for v in rows["Fy"]])
    fz_line = "Fz | " + " ".join([f"{v:>7s}" for v in rows["Fz"]])
    lines = [header_line, fx_line, fy_line, fz_line]
    if gravity_raw_lsb is not None:
        gx, gy, gz = gravity_raw_lsb
        lines.append(f"G  | X={gx:+06d} Y={gy:+06d} Z={gz:+06d} LSB")
    if peripheral is not None:
        joy_x   = peripheral.get("joy_x")
        joy_y   = peripheral.get("joy_y")
        joy_btn = peripheral.get("joy_btn")
        pot     = peripheral.get("pot")
        motor   = peripheral.get("motor")
        joy_str   = f"joy=({joy_x:+5d},{joy_y:+5d}) btn={joy_btn}" if joy_x is not None else "joy=n/a"
        pot_str   = f"pot={pot:5d}" if pot is not None else "pot=n/a"
        motor_str = f"motor={motor:6d}" if motor is not None else "motor=n/a"
        lines.append(f"P  | {joy_str}  {pot_str}  {motor_str}")
    return "\n".join(lines)

def parse_args() -> argparse.Namespace:
    """
    Parses command-line arguments for the script.
    Currently, it only supports specifying the COM port.
    """
    parser = argparse.ArgumentParser(description="Burst-mode force measurement test.")
    parser.add_argument(
        "-p",
        "--com-port",
        default=None,
        help="Serial COM port (default: auto-detect ESP32-S3).",
    )
    return parser.parse_args()


# Terminal run loop

if __name__ == "__main__":
    # --- Script Entry Point ---
    # This block runs when the script is executed directly.

    # 1. Parse command-line arguments to get the COM port, if specified.
    parser = argparse.ArgumentParser(description="Burst-mode force measurement test.")
    parser.add_argument(
        "-p",
        "--com-port",
        default=None,
        help="Serial COM port (default: auto-detect ESP32-S3).",
    )

    # 2. Use a 'with' statement to manage the espDriver lifecycle.
    # This ensures that the connection is properly started and, more importantly,
    # cleanly shut down (threads stopped, serial port closed) when the block is exited,
    # either normally or due to an error.
    with espDriver(comPort=parser.parse_args().com_port) as esp:
        # 3. Start the high-frequency data streaming threads.
        # This command tells the ESP32 to start sending sensor data and launches
        # background threads on the host PC to read and process this data.
        esp.startThreadBurstForce()
        time.sleep(1)

        # Initialize variables for tracking performance and state.
        counter0 = esp.sample_counter
        last_printed_sample = -1
        host_t0 = time.perf_counter()
        last_motor_cmd_pot = None

        start_time = time.time()
        print("Streaming data... Press Ctrl+C to stop.")
        try:
            # 4. Enter the main application loop.
            # This loop runs indefinitely, continuously processing and displaying data.
            while True:
                current_sample = esp.sample_counter

                # --- Example Logic: Motor Control ---
                # This section demonstrates how to create interactive behavior.
                # It reads the potentiometer value from the ESP and commands the motor
                # to move to that position.
                if esp.potentiometer_value is not None:
                    pot = esp.potentiometer_value
                    # To avoid flooding the ESP with commands, we only send a new one
                    # if the potentiometer has moved by at least 5 units (a "debounce" value)
                    if (last_motor_cmd_pot is None or
                            abs(pot - last_motor_cmd_pot) >= 5):
                        esp.sendCommandMotor(position=pot, speed=1000, acc=20)
                        last_motor_cmd_pot = pot

                # --- Example Logic: Data Retrieval ---
                # The `espDriver` object makes the latest data readily available
                # as attributes. The background threads update these values automatically.
                # `esp.buffer[-1]` gets the most recent full sample of force data.
                force_vectors = esp.buffer[-1]
                # Other peripheral data is also directly accessible.
                host_elapsed_us = int((time.perf_counter() - host_t0) * 1_000_000)
                host_sec = host_elapsed_us // 1_000_000
                host_ms = (host_elapsed_us % 1_000_000) // 1000
                gravity_raw_lsb = esp.gravity_raw_lsb
                acq_status = esp.acq_status

                # We can package the peripheral data into a dictionary for easy passing.
                peripheral = None
                if esp.joystick_x is not None:
                    peripheral = {
                        "joy_x": esp.joystick_x,
                        "joy_y": esp.joystick_y,
                        "joy_btn": esp.joystick_button,
                        "pot": esp.potentiometer_value,
                        "motor": esp.motor_position,
                    }
                
                # --- Example Logic: Data Display ---
                # Finally, we print the collected data to the console in a formatted table.
                # This provides a real-time view of the system's state.
                print(f"Sample #{current_sample:6d} host={host_sec:03d}.{host_ms:03d}")
                print(_render_force_table(force_vectors, gravity_raw_lsb, acq_status, peripheral))

                # Add a small delay to prevent the loop from consuming 100% CPU
                # while just printing data. The actual data acquisition is handled
                # by the high-priority background threads.
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\nInterrupted by user.")

        finally:
            # 5. Cleanup.
            # This block is executed when the loop is exited (e.g., by Ctrl+C).
            # The `with` statement will automatically call `esp.stopThread()` and `esp.close()`,
            # but we can print some final statistics here.
            elapsed = time.time() - start_time
            samples_read = esp.sample_counter - counter0
            if elapsed > 0:
                print(f"Refresh Rate {samples_read / elapsed:.2f} Hz")
            else:
                print("Refresh Rate N/A (elapsed time too short)")
            print("Stopping force stream...")
            esp.stopThread()
