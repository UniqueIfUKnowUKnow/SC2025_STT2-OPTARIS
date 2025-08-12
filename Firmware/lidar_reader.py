# lidar_reader.py
import serial
import threading

class LidarReader(threading.Thread):
    """A dedicated thread that continuously reads data from the TFmini-S LiDAR."""
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.data_queue = data_queue
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.ser.flushInput()
        self.frame_header = 0x59
        print("LiDAR Reader thread initialized.")

    def run(self):
        while True:
            if self.ser.read(1) == b'\x59':
                if self.ser.read(1) == b'\x59':
                    frame = b'\x59\x59' + self.ser.read(7)
                    if len(frame) == 9:
                        checksum = sum(frame[:-1]) & 0xFF
                        if checksum == frame[8]:
                            distance_cm = frame[2] + (frame[3] << 8)
                            if distance_cm > 0:
                                self.data_queue.put(distance_cm)