import serial
import serial.tools.list_ports
import time

# Special bytes for the protocol
START_BYTE = 0x7E
END_BYTE = 0x7F
ESCAPE_BYTE = 0x7D
MAX_PAYLOAD_SIZE = 250

class comm_LLL:
    """
    A class to handle communication with the ESP
    using a custom byte-oriented protocol.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initializes the serial connection.
        :param port: The serial port to connect to (e.g., 'COM3').
        :param baudrate: The communication speed.
        :param timeout: The read timeout in seconds.
        """
        self.ser = serial.Serial(port, baudrate, timeout=timeout,
                                bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                xonxoff=False,       # DISABLE Software Flow Control
                                rtscts=False,        # DISABLE Hardware Flow Control
                                dsrdtr=False         # DISABLE DSR/DTR
        )
        self._crc_table = self._generate_crc8_table()
        self._read_buffer = bytearray()
        print(f"Connected to {port} at {baudrate} baud.")

    @staticmethod
    def find_esp_port():
        """
        Finds the serial port corresponding to an ESP32-S3.
        Looks for a device with USB VID:PID of 0x303A:0x1001.
        """
        esp_ports = [
            p.device
            for p in serial.tools.list_ports.comports()
            if p.vid == 0x303A and p.pid == 0x1001
        ]
        return esp_ports[0]

    def _generate_crc8_table(self) -> list[int]:
        """Generates a lookup table for CRC-8 calculation to match the C implementation."""
        table = []
        poly = 0x2F
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
            table.append(crc & 0xFF)
        return table

    def _crc8(self, data: bytes) -> int:
        """Calculates the CRC-8 checksum for the given data using a lookup table."""
        crc = 0x00
        for byte in data:
            crc = self._crc_table[crc ^ byte]
        return crc

    def send_frame(self, frame_id: int, payload: bytes = b''):
        """
        Encodes and sends a frame to the ESP.

        The frame is structured as follows before byte stuffing:
        [Frame ID (1 byte)] [Payload Length (1 byte)] [Payload (0-250 bytes)] [CRC-8 (1 byte)]

        Byte stuffing is applied to the data packet (ID, Length, Payload, CRC) to ensure
        that START_BYTE and END_BYTE only appear at the beginning and end of the final frame.
        - START_BYTE (0x7E) is replaced with ESCAPE_BYTE (0x7D) followed by 0x5E.
        - END_BYTE (0x7F) is replaced with ESCAPE_BYTE (0x7D) followed by 0x5F.
        - ESCAPE_BYTE (0x7D) is replaced with ESCAPE_BYTE (0x7D) followed by 0x5D.

        :param frame_id: The 8-bit command/frame ID.
        :param payload: The data payload as a bytes object, up to MAX_PAYLOAD_SIZE.
        """
        
        if len(payload) > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload size exceeds MAX_PAYLOAD_SIZE ({MAX_PAYLOAD_SIZE})")

        # Data to be sent (before stuffing): ID, Length, Payload
        payload_len = len(payload)
        crc_data = bytes([frame_id, payload_len]) + payload
        crc = self._crc8(crc_data)
        
        # Full data packet to be stuffed
        data_packet = crc_data + bytes([crc])

        # Start building the final frame with stuffing
        stuffed_frame = bytearray([START_BYTE])
        for byte in data_packet:
            if byte == START_BYTE:
                stuffed_frame.extend([ESCAPE_BYTE, 0x5E])
            elif byte == END_BYTE:
                stuffed_frame.extend([ESCAPE_BYTE, 0x5F])
            elif byte == ESCAPE_BYTE:
                stuffed_frame.extend([ESCAPE_BYTE, 0x5D])
            else:
                stuffed_frame.append(byte)
        
        stuffed_frame.append(END_BYTE)
        self.ser.write(stuffed_frame)

    def read_frame(self, timeout: float = 1.0):
        """
        Reads and decodes a frame from the ESP.

        This function reads available data from the serial port, buffers it,
        and then parses the buffer to find a complete and valid frame. It handles:
        - Searching for START_BYTE and END_BYTE to identify a potential frame.
        - Discarding any garbage data before a START_BYTE.
        - Unstuffing the frame content to restore the original data packet.
        - Validating the unstuffed data's length and CRC-8 checksum.
        - Parsing the validated data into a frame ID and payload.

        :param timeout: Time to wait for a complete frame in seconds.
                        The function will continuously read from the serial port
                        and process the buffer until a valid frame is found or the timeout expires.
        :return: A tuple of (frame_id, payload) or None if a timeout or error occurs.
        """

        start_time = time.time()
        def get_ts(rel=False):
            if rel:
                return f"[{time.time()-start_time:.4f}]"
            return f"[{time.time():.4f}]"
        
        while time.time() - start_time < timeout:
            # Read any available data from the serial port and append to buffer
            bytes_to_read = self.ser.in_waiting
            if bytes_to_read > 0:
                new_data = self.ser.read(bytes_to_read)
                self._read_buffer.extend(new_data)

            while True:
                # Search for a complete frame (from START to END) in the buffer
                start_index = self._read_buffer.find(START_BYTE)
                if start_index == -1:
                    # No start byte, no potential frame. Wait for more data.
                    break

                end_index = self._read_buffer.find(END_BYTE, start_index)
                if end_index == -1:
                    # Found a start but no end yet. Wait for more data.
                    break

                # Discard any garbage data before the start byte
                if start_index > 0:
                    self._read_buffer = self._read_buffer[start_index:]
                    end_index -= start_index

                # Extract the stuffed frame content (between START and END)
                stuffed_content = self._read_buffer[1:end_index]
                #Consume the entire processed frame from the buffer
                
                # Consume the entire processed frame from the buffer
                self._read_buffer = self._read_buffer[end_index + 1:]

                # Unstuff the frame content
                unstuffed_data = bytearray()
                i = 0
                while i < len(stuffed_content):
                    if stuffed_content[i] == ESCAPE_BYTE:
                        if i + 1 < len(stuffed_content):
                            unstuffed_data.append(stuffed_content[i+1] ^ 0x20)
                            i += 2
                        else: # Malformed frame, escape byte at the very end
                            i += 1 
                    else:
                        unstuffed_data.append(stuffed_content[i])
                        i += 1
                
                # We must have at least ID, Length, and CRC
                if len(unstuffed_data) < 3:
                    print(f"{get_ts(rel=True)} read_frame: Frame too short after unstuffing. Discarded.")
                    continue # Look for the next frame in the buffer

                # Validate CRC
                received_crc = unstuffed_data[-1]
                crc_data = unstuffed_data[:-1]
                calculated_crc = self._crc8(crc_data)

                if received_crc == calculated_crc:
                    frame_id = crc_data[0]
                    payload_len = crc_data[1]
                    payload = bytes(crc_data[2:])
                    if payload_len == len(payload):
                        if getattr(self, "debug_frames", False):
                            print(f"RX frame: {(frame_id, payload)}")
                        return frame_id, payload
                else:
                    print(f"{get_ts(rel=True)} read_frame: CRC Mismatch! Got {received_crc:02X}, expected {calculated_crc:02X}. Frame discarded.")
                    print(f"    -> Bad Frame Data (unstuffed): {unstuffed_data.hex(' ')}")
                    
            
            time.sleep(0.001) # No full frame found yet; release GIL while waiting
        
        return None # Timeout

    def close(self):
        """Closes the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")
            
