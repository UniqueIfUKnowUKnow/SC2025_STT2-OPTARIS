import serial
import time

SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 115200
REFRESH_RATE_HZ = 100

class TFminiS:
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
    
    def set_refresh_rate(self, rate_hz):
        if not self.ser or not self.ser.isOpen():
            print("Serial port is not open. Cannot set refresh rate.")
            return False
            
        if not 1 <= rate_hz <= 1000:
            print(f"Invalid refresh rate: {rate_hz}. Rate must be between 1 and 1000 Hz.")
            return False
            
        rate_low_byte = rate_hz & 0xFF
        rate_high_byte = (rate_hz >> 8) & 0xFF
        
        command = [0x42, 0x57, 0x02, 0x00, rate_low_byte, rate_high_byte, 0x00, 0x00]
        
        checksum = (sum(command[2:]) & 0xFF)
        command.append(checksum)
        
        command_bytes = bytes(command)
        
        self.ser.write(command_bytes)
        
        self.ser.flushInput()
        time.sleep(0.1)
        
        print(f"Attempted to set refresh rate to {rate_hz} Hz.")
        return True

    def read_data(self):
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
                            strength = frame[4] + (frame[5] << 8)
                            temperature_raw = frame[6] + (frame[7] << 8)
                            temperature = (temperature_raw / 8.0) - 256.0

                            return distance, strength, temperature
                        else:
                            self.ser.flushInput()
                            print("Checksum error. Flushing buffer and retrying.")
                            return None, None, None
            else:
                time.sleep(0.001)


def main():
    print("--- TFmini-S LiDAR Sensor Test Program ---")
    
    lidar = TFminiS(port=SERIAL_PORT, baudrate=BAUD_RATE)

    if not lidar.open_serial():
        print("Exiting program. Please check your connections and serial port configuration.")
        return

    lidar.set_refresh_rate(REFRESH_RATE_HZ)

    print("\nStarting to read data from TFmini-S...")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            distance_cm, strength, temp_c = lidar.read_data()

            if distance_cm is not None:
                if distance_cm == 65535:
                    print(f"Signal Unreliable. Strength: {strength}")
                else:
                    distance_m = distance_cm / 100.0
                    print(f"Distance: {distance_cm:4d} cm ({distance_m:.2f} m) | Strength: {strength:5d} | Temp: {temp_c:.2f} C")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        lidar.close_serial()
        print("Program terminated.")

if __name__ == '__main__':
    main()













































# import serial
# import time
# import random

# # --- Configuration for TFmini-S LiDAR Sensor ---
# LIDAR_PORT = '/dev/ttyS0'
# LIDAR_BAUD_RATE = 115200
# REFRESH_RATE_HZ = 100

# # --- Configuration for Arduino Communication ---
# ARDUINO_PORT = '/dev/ttyACM0' 
# ARDUINO_BAUD_RATE = 115200
# SEND_INTERVAL_SECONDS = 3

# class TFminiS:
#     def __init__(self, port, baudrate):
#         self.port = port
#         self.baudrate = baudrate
#         self.ser = None
#         self.frame_length = 9
#         self.frame_header = 0x59

#     def open_serial(self):
#         try:
#             self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
#             self.ser.flushInput()
#             if self.ser.isOpen():
#                 print(f"Successfully opened serial port: {self.port} at {self.baudrate} baud.")
#                 return True
#             else:
#                 print(f"Failed to open serial port: {self.port}.")
#                 return False
#         except serial.SerialException as e:
#             print(f"Error opening serial port {self.port}: {e}")
#             return False

#     def close_serial(self):
#         if self.ser and self.ser.isOpen():
#             self.ser.close()
#             print("Serial port closed.")
    
#     def set_refresh_rate(self, rate_hz):
#         if not self.ser or not self.ser.isOpen():
#             print("Serial port is not open. Cannot set refresh rate.")
#             return False
            
#         if not 1 <= rate_hz <= 1000:
#             print(f"Invalid refresh rate: {rate_hz}. Rate must be between 1 and 1000 Hz.")
#             return False
            
#         rate_low_byte = rate_hz & 0xFF
#         rate_high_byte = (rate_hz >> 8) & 0xFF
        
#         command = [0x42, 0x57, 0x02, 0x00, rate_low_byte, rate_high_byte, 0x00, 0x00]
        
#         checksum = (sum(command[2:]) & 0xFF)
#         command.append(checksum)
        
#         command_bytes = bytes(command)
        
#         self.ser.write(command_bytes)
        
#         self.ser.flushInput()
#         time.sleep(0.1)
        
#         print(f"Attempted to set refresh rate to {rate_hz} Hz.")
#         return True

#     def read_data(self):
#         while True:
#             if self.ser.in_waiting >= self.frame_length:
#                 byte1 = self.ser.read(1)
#                 if not byte1:
#                     continue

#                 if ord(byte1) == self.frame_header:
#                     byte2 = self.ser.read(1)
#                     if not byte2:
#                         continue

#                     if ord(byte2) == self.frame_header:
#                         remaining_bytes = self.ser.read(self.frame_length - 2)

#                         frame = byte1 + byte2 + remaining_bytes

#                         checksum = frame[-1]
#                         calculated_checksum = sum(frame[:-1]) & 0xFF

#                         if checksum == calculated_checksum:
#                             distance = frame[2] + (frame[3] << 8)
#                             strength = frame[4] + (frame[5] << 8)
#                             temperature_raw = frame[6] + (frame[7] << 8)
#                             temperature = (temperature_raw / 8.0) - 256.0

#                             return distance, strength, temperature
#                         else:
#                             self.ser.flushInput()
#                             print("Checksum error. Flushing buffer and retrying.")
#                             return None, None, None
#             else:
#                 time.sleep(0.001)


# class dataExtract:
#     def extract_digits(self, received_value: int) -> dict:
#         servo_angle = received_value % (10**4)
#         motor_angle = (received_value // (10**4)) % (10**5)
        
#         return {
#             'motor_angle': motor_angle,
#             'servo_angle': servo_angle
#         }

# def main():
#     print("--- Combined LiDAR and Arduino Test Program ---")
    
#     lidar = TFminiS(port=LIDAR_PORT, baudrate=LIDAR_BAUD_RATE)
#     data_extractor = dataExtract()
#     arduino_ser = None

#     if not lidar.open_serial():
#         print("Exiting program. Please check LiDAR connections and serial port configuration.")
#         return

#     try:
#         arduino_ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD_RATE, timeout=1)
#         print(f"Successfully opened serial port {ARDUINO_PORT} at {ARDUINO_BAUD_RATE} baud.")
#         time.sleep(2) 
#         print("Starting communication with Arduino...")
#     except serial.SerialException as e:
#         print(f"Error: Could not open Arduino serial port {ARDUINO_PORT}. {e}")
#         print("Please ensure the Arduino is connected and the port name is correct.")
#         lidar.close_serial()
#         return

#     lidar.set_refresh_rate(REFRESH_RATE_HZ)

#     print("\nStarting main loop...")
#     print("Press Ctrl+C to exit.")

#     try:
#         while True:
#             # Read data from the LiDAR sensor
#             distance_cm, strength, temp_c = lidar.read_data()

#             if distance_cm is not None:
#                 # Print LiDAR data
#                 if distance_cm == 65535:
#                     print(f"LiDAR Signal Unreliable. Strength: {strength}")
#                 else:
#                     distance_m = distance_cm / 100.0
#                     print(f"LiDAR: Distance: {distance_cm:4d} cm ({distance_m:.2f} m) | Strength: {strength:5d} | Temp: {temp_c:.2f} C")

#             # Communicate with the Arduino
#             value_to_send_pi_to_arduino = random.randint(10000, 99999)
#             message_to_send = f"{value_to_send_pi_to_arduino}\n"
#             arduino_ser.write(message_to_send.encode('utf-8'))
#             print(f"\n[RPi -> Arduino] Sent 5-digit number: {value_to_send_pi_to_arduino}")
#             print("[RPi <- Arduino] Waiting for 8-digit response...")
#             line = arduino_ser.readline().decode('utf-8').rstrip()
            
#             if line:
#                 try:
#                     received_19_digit_number = int(line)
#                     print(f"[RPi <- Arduino] Received 8-digit number: {received_19_digit_number}")
#                     print(f"                          (Type: {type(received_19_digit_number)})")
#                     extracted_data = data_extractor.extract_digits(received_19_digit_number)
                    
#                     motor_angle = extracted_data['motor_angle']
#                     servo_angle = extracted_data['servo_angle']
                    
#                     print(f"Extracted motorAngle: {motor_angle}")
#                     print(f"Extracted servoAngle: {servo_angle}")

#                 except ValueError as e:
#                     print(f"Error: Could not convert received data to integer: '{line}'. Error: {e}")
#                 except Exception as e:
#                     print(f"An error occurred during data extraction: {e}")
#             else:
#                 print("[RPi <- Arduino] No data received from Arduino (timeout).")
            
#             time.sleep(SEND_INTERVAL_SECONDS)


#     except KeyboardInterrupt:
#         print("\nCtrl+C detected. Shutting down.")
#     except Exception as e:
#         print(f"\nAn error occurred: {e}")
#     finally:
#         lidar.close_serial()
#         if arduino_ser and arduino_ser.is_open:
#             arduino_ser.close()
#             print("Arduino serial port closed.")
#         print("Program terminated.")

# if __name__ == '__main__':
#     main()
