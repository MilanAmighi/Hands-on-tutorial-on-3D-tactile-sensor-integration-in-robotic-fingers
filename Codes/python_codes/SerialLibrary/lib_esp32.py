import time
import struct
from SerialLibrary.comm_LLL import comm_LLL
import numpy as np
import serial
import serial.tools.list_ports
import sys
import crc
import threading
from queue import Queue, Empty

# --- Constants for CAN Communication ---
_ESP32S3_USB_VID = 0x303A
_ESP32S3_USB_PID = 0x1001


def find_esp32s3_port() -> str | None:
    """Return the first serial port matching the ESP32-S3 USB Serial/JTAG VID:PID (303A:1001).

    Returns None if no matching device is found.
    """
    for port in serial.tools.list_ports.comports():
        if port.vid == _ESP32S3_USB_VID and port.pid == _ESP32S3_USB_PID:
            return port.device
    return None


CMD_ID_PING             = 0x01
CMD_ID_PONG             = 0x02
CMD_ID_I2C_ONESHOT      = 0x10
RESP_ID_I2C_ONESHOT     = 0x11
CMD_ID_STREAM_BURST     = 0x14
RESP_ID_STREAM_BURST    = 0x15
CMD_ID_STREAM_STOP      = 0x1F
CMD_ID_MOTOR_CONTROL    = 0x40

# --- Constants for Commands ---
_CMD_SCAN_I2C = 2
_CMD_START_STREAM_BURST = 6
_MANDATORY_SENSOR_ADDRS = [0x10, 0x11, 0x12, 0x13]



class espDriver(comm_LLL):
    """
    A high-level driver for communicating with the ESP32-S3 firmware.

    This class inherits from `comm_LLL` to handle the low-level serial protocol
    and provides methods for specific commands like scanning the I2C bus,
    streaming sensor data, and controlling motors.

    It automatically handles port detection and manages background threads for
    continuous data acquisition.
    """
    def __init__(self, buffer_size: int = 100,comPort =None):
        """
        Initializes the espDriver, finds the serial port, and connects to the device.
        :param buffer_size: The number of samples to store in the internal data buffer.
        :param comPort: The serial port to connect to (e.g., 'COM3'). If None, it will be auto-detected.
        """
        self.Laddr = []
        self.is_streaming = False
        self.buffer_size = buffer_size
        config = crc.Configuration(
            width=8,
            polynomial=0x2f,
            init_value=0x0,
            final_xor_value=0x00,
            reverse_input=False,
            reverse_output=False,
        )
        self.calculator = crc.Calculator(config)

        if comPort is None:
            auto_port = find_esp32s3_port()
            if auto_port is not None:
                print(f"ESP32-S3 detected automatically on {auto_port}")
                self.port = auto_port
            else:
                Pfound = []
                Lserial = serial.tools.list_ports.comports()
                for i, port_tmp in enumerate(Lserial):
                    print(f"Port {port_tmp} has been found, it is the {i+1} device connected")
                    Pfound.append(port_tmp)

                if len(Pfound) == 0:
                    print("No COM port found, please check the connection and retry")
                    sys.exit()
                elif len(Pfound) == 1:
                    port = str(Pfound[0]).split(" ")[0]
                    print(f"1 COM port found, {port} automatically connecting")
                    self.port = port
                else:
                    print("Multiple COM ports found, requiring manual input")
                    for i, k in enumerate(Pfound):
                        print(f"Type {i} for port {k}")
                    print("==================")
                    idx = input("Please type the number and press enter: ")
                    self.port = str(Pfound[int(idx)]).split(" ")[0]
        else:
            self.port = comPort
        super().__init__(port=self.port, baudrate=115200, timeout= 1.0)

        self.scanI2Cbus()

        self.Nbuffer = buffer_size
        self.buffer = np.zeros((buffer_size, len(self.Laddr), 4, 3), dtype=np.float16)
        self.isRunning = False
        self.data_queue = Queue()
        self.processing_thread = None
        self.io_thread = None
        self.isNewValue = False # Flag for simple new data checks
        self.sample_counter = 0 # Counter for robust frequency measurement
        self.gravity_raw_lsb = None
        self.gravity_ms2 = None
        self.acq_status = None
        self.joystick_x = None
        self.joystick_y = None
        self.joystick_button = None
        self.potentiometer_value = None
        self.motor_position = None

    def scanI2Cbus(self):
        """
        Scans the I2C bus for connected devices.

        Sends a command to the ESP32 to perform an I2C scan and reads the response.
        It populates `self.Laddr` with the addresses of the found devices, merged
        with a list of mandatory sensor addresses. This ensures that the system
        always attempts to read from critical sensors even if they are not detected.

        The function will retry the command up to 3 times if no valid response is received.
        If scanning fails, it falls back to using only the mandatory addresses.

        :return: A sorted list of unique I2C addresses to be used for data acquisition.
        """

        payload = bytes([_CMD_SCAN_I2C] + [0]*15) # Mode 2, rest of 16-byte command is 0
        response = None
        for _ in range(3):
            self.send_frame(CMD_ID_I2C_ONESHOT, payload)
            response = self.read_frame(timeout=2.0)
            if response and response[0] == RESP_ID_I2C_ONESHOT:
                break
            time.sleep(0.05)

        if response and response[0] == RESP_ID_I2C_ONESHOT:
            resp_payload = response[1]
            device_count = resp_payload[0]
            if device_count > 0:
                addresses_int = [int(addr) for addr in resp_payload[1:1 + device_count]]
                merged_addresses = sorted(set(addresses_int + _MANDATORY_SENSOR_ADDRS))
                addresses_hex = [hex(addr) for addr in addresses_int]
                if merged_addresses != sorted(set(addresses_int)):
                    print(
                        "Success! Found "
                        f"{device_count} I2C device(s): {', '.join(addresses_hex)}. "
                        "Using mandatory acquisition addresses: "
                        f"{', '.join(hex(addr) for addr in merged_addresses)}"
                    )
                else:
                    print(f"Success! Found {device_count} I2C device(s): {', '.join(addresses_hex)}")
                self.Laddr = merged_addresses
                return merged_addresses
            else:
                print(
                    "Success! No I2C devices found. "
                    "Using mandatory acquisition addresses: "
                    f"{', '.join(hex(addr) for addr in _MANDATORY_SENSOR_ADDRS)}"
                )
                self.Laddr = list(_MANDATORY_SENSOR_ADDRS)
                return self.Laddr
        else:
            print(
                f"Failed. Response: {response}. "
                "Using mandatory acquisition addresses: "
                f"{', '.join(hex(addr) for addr in _MANDATORY_SENSOR_ADDRS)}"
            )
            self.Laddr = list(_MANDATORY_SENSOR_ADDRS)
            return self.Laddr

    def test_ping(self):
        """
        Tests the connection to the ESP32 by sending a PING command.

        A successful test will receive a PONG response. This is useful for
        verifying that the communication link is active and the firmware is responsive.
        """
        print("\n--- Testing Ping (0x01) ---")
        self.send_frame(CMD_ID_PING)
        response = self.read_frame()
        if response and response[0] == CMD_ID_PONG:
            print(f"Success! Received Pong: {response[1].decode()}")
        else:
            print(f"Failed. Response: {response}")


    def decode_data_burst_force(self, msg: bytearray, num_addresses: int) -> tuple[np.ndarray, bool]:
        """
        Decodes the force data burst payload from the ESP32.

        The payload contains force vectors for each sensor, followed by auxiliary data
        like IMU gravity, acquisition status, joystick/potentiometer values, and motor position.

        Payload Structure:
        - [Force Vectors]: A flat array of floats (num_addresses * 2 sensors * 3 axes * 4 bytes/float).
        - [Gravity Vector]: 3x int16_t for raw IMU gravity (Gx, Gy, Gz).
        - [Acquisition Status]: Counts for successes/failures, error codes.
        - [Joystick/Pot/Motor]: Packed struct with joystick X/Y, button, pot value, and motor position.

        This function parses the message, updates the corresponding instance attributes
        (e.g., `self.gravity_ms2`, `self.joystick_x`), and returns the reshaped force data.

        :param msg: The raw bytearray payload received from the ESP32.
        :param num_addresses: The number of I2C addresses being sampled.
        :return: A tuple containing:
                 - A NumPy array of force vectors, shaped (num_sensors, 3).
                 - A boolean `reset_required` flag, which is currently always False but was intended for error handling.
        """
        sensor_count = num_addresses * 2
        expected_length = sensor_count * 3 * 4

        if len(msg) == expected_length + 30:
            gravity_vector = np.frombuffer(msg[expected_length:expected_length + 6], dtype='<i2')
            gx, gy, gz = [int(v) for v in gravity_vector]
            self.gravity_raw_lsb = (gx, gy, gz)
            self.gravity_ms2 = (gx / 100.0, gy / 100.0, gz / 100.0)

            status = msg[expected_length + 6:expected_length + 18]
            self.acq_status = {
                "imu_success_count": int.from_bytes(status[0:2], byteorder='little', signed=False),
                "imu_fail_count": int.from_bytes(status[2:4], byteorder='little', signed=False),
                "force_success_count": int.from_bytes(status[4:6], byteorder='little', signed=False),
                "force_fail_count": int.from_bytes(status[6:8], byteorder='little', signed=False),
                "imu_last_err": int.from_bytes(status[8:10], byteorder='little', signed=True),
                "imu_last_read_ok": int(status[10]),
                "force_last_read_ok": int(status[11]),
            }
            joy_x, joy_y, joy_btn, _pad, pot_val, motor_pos = struct.unpack_from('<hhBBhi', msg, expected_length + 18)
            self.joystick_x = joy_x
            self.joystick_y = joy_y
            self.joystick_button = joy_btn
            self.potentiometer_value = pot_val
            self.motor_position = motor_pos
            msg = msg[:expected_length]


        if len(msg) != expected_length:
            print(f"Error: Incorrect message length. Expected {expected_length}, got {len(msg)}.")
            if self.isRunning:
                print("Stopping data stream due to message length error.")
                self.isRunning = False
            return np.zeros((sensor_count, 3)), True # Return empty array and request reset

        force_vector = np.frombuffer(msg, dtype='<f4')
        return force_vector.reshape((sensor_count, 3)), False


    def startThreadBurstForce(self, frequency: int = 500):
        """
        Starts background threads to continuously read and process sensor data in burst mode.

        This method configures and initiates a high-frequency data stream from the ESP32.
        It performs the following steps:
        1.  Constructs a command payload specifying the stream mode, number of I2C addresses,
            and the desired acquisition frequency.
        2.  Sends the `CMD_ID_STREAM_BURST` command to the firmware.
        3.  Starts two background threads:
            - `_threaded_io_read_burst`: A dedicated I/O thread that reads raw data frames
              from the serial port and places them into a queue.
            - `_threaded_process_data_burst_force`: A processing thread that consumes data from
              the queue, decodes it, and updates the data buffers.
        :param frequency: The target data acquisition frequency in Hz.
        """
        if not self.Laddr:
            print("Error: Cannot start stream with no I2C addresses.")
            return
        self.sample_counter = 0

        self.gravity_raw_lsb = None
        self.gravity_ms2 = None
        self.acq_status = None
        # Calculate loop period in microseconds. 0 means run as fast as possible.
        loop_period_us = 0
        if frequency > 0:
            loop_period_us = int(1_000_000 / frequency)
        print(f"Starting a Force burst thread at a frequency of {frequency}")
        self.isRunning = True
        num_addresses = len(self.Laddr)
        self.data = np.zeros((num_addresses*2, 3), dtype=np.float16)
        self.buffer = np.zeros((self.Nbuffer, num_addresses*2, 3), dtype=np.float16)
        self.io_thread = threading.Thread(target=self._threaded_io_read_burst, args=(num_addresses,))
        self.processing_thread = threading.Thread(target=self._threaded_process_data_burst_force, args=(num_addresses,))

        # Command payload: [mode, n_addr, loop_period_us (2 bytes), addr1, addr2, ...]
        payload = bytearray([_CMD_START_STREAM_BURST, num_addresses])
        payload.extend(loop_period_us.to_bytes(2, 'little')) # Add period as 2 bytes
        payload.extend(self.Laddr)
        padding = [0] * (16 - len(payload))
        payload.extend(padding)
        self.send_frame(CMD_ID_STREAM_BURST, payload)
        self.io_thread.start()
        self.processing_thread.start()
        print("Measurement thread started")

    def _threaded_io_read_burst(self, num_addresses: int):
        """
        Dedicated I/O thread for reading burst data.

        This function runs in a separate thread and its only job is to read
        raw data frames from the serial port and put them into `self.data_queue`.
        This separation prevents the main application or processing logic from being
        blocked by serial I/O waits.
        """

        while self.isRunning:
            # Wait for the first frame of a new data packet
            response = self.read_frame(timeout=2.0)
            if response and response[0] == RESP_ID_STREAM_BURST:
                self.data_queue.put(response[1])
            else :
                if self.isRunning:
                    print(f"Warning: I/O thread missed a packet or timed out. Response: {response}")

    def _threaded_process_data_burst_force(self, num_addresses: int):
        """
        Dedicated processing thread for decoding burst data.

        This function runs in a separate thread. It pulls raw data packets from
        `self.data_queue`, decodes them using `decode_data_burst_force`, and updates
        the main data array (`self.data`) and the circular buffer (`self.buffer`).

        It sets the `isNewValue` flag to True after each new sample is processed.
        """
        while self.isRunning:
            try:
                # Wait for up to 1 second for an item from the I/O thread
                raw_data = self.data_queue.get(timeout=1.0)

                data, reset_required = self.decode_data_burst_force(raw_data, num_addresses=num_addresses)
                self.data = data
                self.buffer = np.roll(self.buffer, -1, axis=0)
                self.buffer[-1] = self.data
                self.isNewValue = True
                self.sample_counter += 1

                self.data_queue.task_done()
            except Empty:
                # This is normal if the stream stops or pauses
                continue
            except Exception as e:
                print(f"Error in processing thread: {e}")



    def stopThread(self):
        """
        Stops the data streaming threads and cleans up the connection.

        This method gracefully terminates the data stream by:
        1.  Sending a `CMD_ID_STREAM_STOP` command to the ESP32.
        2.  Setting the `self.isRunning` flag to False, which signals the
            background threads to exit their loops.
        3.  Joining the I/O and processing threads to wait for them to finish.
        4.  Flushing any remaining data from the serial input buffer to ensure
            a clean state for the next operation.
        """
        # Send stop command FIRST while the I/O thread is still reading, so the
        # ESP32's USB TX buffer stays drained and burst_aux_task can process the stop.
        self.send_frame(CMD_ID_STREAM_STOP)
        time.sleep(0.15)  # Give ESP32 time to process stop before threads exit
        self.isRunning = False
        if self.io_thread and self.io_thread.is_alive():
            self.io_thread.join()
        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join()

        flush_deadline = time.time() + 2.0
        bytesBuffer = self.ser.in_waiting
        while bytesBuffer > 0 and time.time() < flush_deadline:
            self.ser.read(bytesBuffer)
            time.sleep(0.001)
            bytesBuffer = self.ser.in_waiting

    def sendCommandMotor(self,position=0,speed=1000,acc=0x0):
        """
        Sends a command to control the motor.

        The payload is a 5-byte packet:
        - Position (int16_t, big-endian): Target position.
        - Speed (uint16_t, big-endian): Movement speed.
        - Acceleration (uint8_t): Acceleration profile.
        :param position: The target motor position (signed 16-bit integer).
        :param speed: The motor speed (unsigned 16-bit integer).
        :param acc: The motor acceleration profile (unsigned 8-bit integer).
        """
        #s16 Position, u16 Speed, u8 ACC = 0
        bytes_position = position.to_bytes(2, byteorder='big', signed=True)
        bytes_speed = speed.to_bytes(2, byteorder='big', signed=False)
        bytes_acc = bytes([acc])
        payload = bytes_position+bytes_speed+bytes_acc
        self.send_frame(CMD_ID_MOTOR_CONTROL,payload)


    def __enter__(self):
        """
        Enter the runtime context related to the `with` statement.
        :return: The instance of the object.
        """
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Exit the runtime context, ensuring resources are released.
        This method automatically stops any running threads and closes the serial port
        when the `with` block is exited.
        """

        if hasattr(self, 'isRunning') and self.isRunning:
            self.stopThread()
            time.sleep(0.01)
        # Close the serial port
        self.close()
