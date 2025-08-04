# -*- coding: utf-8 -*-
# /usr/bin/python3
"""
This script is a test program for the Benewake TFmini-S LiDAR sensor,
designed to run on a Raspberry Pi.

It can read data from the sensor and also includes functions to change
the sensor's frame rate (refresh rate).

Hardware Connections (as specified by user):
- Raspberry Pi Pin 4 (5V)    -> TFmini-S Red Wire (+5V)
- Raspberry Pi Pin 6 (Ground) -> TFmini-S Black Wire (GND)
- Raspberry Pi Pin 8 (GPIO 14, TXD) -> TFmini-S White Wire (RXD)
- Raspberry Pi Pin 10 (GPIO 15, RXD) -> TFmini-S Green Wire (TXD)

Before running this script, you must enable the serial port on your Raspberry Pi.
You can do this by:
1. Running `sudo raspi-config` in the terminal.
2. Navigating to 'Interface Options' -> 'Serial Port'.
3. Answering 'No' to the question "Would you like a login shell to be accessible over serial?".
4. Answering 'Yes' to the question "Would you like the serial port hardware to be enabled?".
5. Rebooting the Raspberry Pi.
"""
import serial
import time

# --- Configuration ---
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 115200

class TFminiS:
    """
    A class to manage communication with the TFmini-S LiDAR sensor.
    """
    def __init__(self, port, baudrate):
        """
        Initializes the sensor object.
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.frame_length = 9
        self.frame_header = 0x59

    def open_serial(self):
        """
        Tries to open the serial port. Returns True on success, False on failure.
        """
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
        """
        Closes the serial port if it's open.
        """
        if self.ser and self.ser.isOpen():
            self.ser.close()
            print("Serial port closed.")

    def send_command(self, command):
        """
        Sends a command to the sensor.
        """
        if self.ser and self.ser.isOpen():
            self.ser.write(command)
            print(f"Sent command: {' '.join(hex(b) for b in command)}")
        else:
            print("Cannot send command: serial port is not open.")

    def set_frame_rate(self, rate):
        """
        Sets the frame rate (refresh rate) of the sensor.
        Args:
            rate (int): The desired frame rate in Hz (e.g., 100, 200, 500, 1000).
                        Note: Valid rates are typically 1000/n where n is a positive integer.
        """
        # Command format: 5A 06 03 LL HH SU
        # LL: Low byte of rate, HH: High byte of rate
        ll = rate & 0xFF
        hh = (rate >> 8) & 0xFF
        
        command_bytes = [0x5A, 0x06, 0x03, ll, hh, 0x00]
        # Calculate checksum
        checksum = sum(command_bytes[:-1]) & 0xFF
        command_bytes[-1] = checksum
        
        self.send_command(bytes(command_bytes))
        # It's good practice to wait a moment for the sensor to process the command
        time.sleep(0.1)

    def save_settings(self):
        """
        Sends the "Save Settings" command to make changes permanent.
        """
        # Command: 5A 04 11 6F
        save_command = bytes([0x5A, 0x04, 0x11, 0x6F])
        self.send_command(save_command)
        # The manual recommends waiting 1s after saving.
        time.sleep(1)

    def read_data(self):
        """
        Reads a 9-byte data frame from the sensor, validates it, and parses it.
        Returns a tuple (distance, strength, temperature) or (None, None, None) if data is invalid.
        """
        while True:
            if self.ser.in_waiting >= self.frame_length:
                byte1 = self.ser.read(1)
                if not byte1: continue

                if ord(byte1) == self.frame_header:
                    byte2 = self.ser.read(1)
                    if not byte2: continue

                    if ord(byte2) == self.frame_header:
                        remaining_bytes = self.ser.read(self.frame_length - 2)
                        frame = byte1 + byte2 + remaining_bytes
                        
                        if len(remaining_bytes) < (self.frame_length - 2):
                            # Incomplete frame, try again
                            continue

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
    """
    Main function to initialize the sensor and run the reading loop.
    """
    print("--- TFmini-S LiDAR Sensor Test Program ---")
    
    lidar = TFminiS(port=SERIAL_PORT, baudrate=BAUD_RATE)

    if not lidar.open_serial():
        print("Exiting program. Please check your connections and serial port configuration.")
        return

    lidar.set_frame_rate(10)
    # --- (Optional) Set a new frame rate ---
    # Uncomment the following lines to change the frame rate.
    # The default is 100Hz. Let's change it to 200Hz as an example.
    # -----------------------------------------------------------------
    # print("\nAttempting to change frame rate to 200Hz...")
    # lidar.set_frame_rate(200)
    # print("Saving settings to the sensor...")
    # lidar.save_settings()
    # print("Frame rate change command sent and settings saved.")
    # -----------------------------------------------------------------

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
                    print(f"Distance: {distance_cm:4d} cm ({distance_m:.2f} m) | Strength: {strength:5d} | Temp: {temp_c:.2f} °C")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        lidar.close_serial()
        print("Program terminated.")

if __name__ == '__main__':
    main()
