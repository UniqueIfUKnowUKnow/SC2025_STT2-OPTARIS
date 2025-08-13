import serial
import threading
import queue
import time

class TFminiS:
    """A class to manage communication with the TFmini-S LiDAR sensor."""
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.frame_length = 9
        self.frame_header = 0x59

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.flushInput()
            if self.ser.isOpen():
                print(f"Successfully opened serial port: {self.port} at {self.baudrate} baud.")
                return True
            else:
                print(f"Failed to open serial port: {self.port}.")
                return False
        except serial.SerialException as e:
            print(f"Error opening serial port {self.port}: {e}")
            return False

    def close_serial(self):
        if self.ser and self.ser.isOpen():
            self.ser.close()
            print("Serial port closed.")

    def read_data(self):
        """Reads a 9-byte data frame from the sensor, validates it, and parses it."""
        while True:
            if self.ser.in_waiting >= self.frame_length:
                byte1 = self.ser.read(1)
                if not byte1:
                    continue

                if ord(byte1) == self.frame_header:
                    byte2 = self.ser.read(1)
                    if not byte2:
                        continue

                    if ord(byte2) == self.frame_header:
                        remaining_bytes = self.ser.read(self.frame_length - 2)
                        frame = byte1 + byte2 + remaining_bytes
                        checksum = frame[-1]
                        calculated_checksum = sum(frame[:-1]) & 0xFF

                        if checksum == calculated_checksum:
                            distance = frame[2] + (frame[3] << 8)
                            if distance != 65535:  # Invalid reading check
                                return distance
                        else:
                            self.ser.flushInput()
                            return None
            else:
                time.sleep(0.001)


class LidarReader(threading.Thread):
    """A dedicated thread that continuously reads data from the TFmini-S LiDAR."""
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True)
        self.data_queue = data_queue
        self.lidar = TFminiS(port, baudrate)
        if not self.lidar.open_serial():
            raise RuntimeError("Failed to initialize LiDAR sensor")
        print("LiDAR Reader thread initialized.")

    def run(self):
        while True:
            distance = self.lidar.read_data()
            if distance is not None:
                self.data_queue.put(distance)

    def cleanup(self):
        """Clean up resources when done"""
        self.lidar.close_serial()
