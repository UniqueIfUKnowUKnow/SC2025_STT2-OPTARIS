# lidar_reader.py
import serial
import threading
import time

class LidarReader(threading.Thread):
    """A dedicated thread that continuously reads data from the TFmini-S LiDAR."""
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.data_queue = data_queue
        self.ser = None
        self.frame_header = 0x59
        self._connect()
        print("LiDAR Reader thread initialized.")

    def _connect(self):
        """Establish serial connection with error handling."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.flushInput()
            print("LiDAR connection established.")
        except Exception as e:
            print(f"Failed to connect to LiDAR: {e}")
            self.ser = None

    def run(self):
        while True:
            try:
                if not self.ser or not self.ser.is_open:
                    print("LiDAR disconnected. Attempting to reconnect...")
                    self._connect()
                    if not self.ser:
                        time.sleep(1)  # Wait before retry
                        continue

                # Read first header byte
                data = self.ser.read(1)
                if not data:  # Handle empty read
                    continue
                    
                if data == b'\x59':
                    # Read second header byte
                    data = self.ser.read(1)
                    if not data:
                        continue
                        
                    if data == b'\x59':
                        frame = b'\x59\x59' + self.ser.read(7)
                        if len(frame) == 9:
                            checksum = sum(frame[:-1]) & 0xFF
                            if checksum == frame[8]:
                                distance_cm = frame[2] + (frame[3] << 8)
                                if distance_cm > 0:
                                    self.data_queue.put(distance_cm)
                                    
            except serial.SerialException as e:
                print(f"LiDAR serial error: {e}")
                self.ser = None
                time.sleep(0.05)  # Brief pause before reconnection attempt
            except Exception as e:
                print(f"Unexpected LiDAR error: {e}")
                time.sleep(0.05)