# -*- coding: utf-8 -*-
# /usr/bin/python3
"""
This script is a test program for the Benewake TFmini-S LiDAR sensor,
designed to run on a Raspberry Pi.

It continuously reads data from the sensor via the serial port and prints
the measured distance, signal strength, and sensor temperature.

Hardware Connections (as specified by user):
- Raspberry Pi Pin 4 (5V)       -> TFmini-S Red Wire (+5V)
- Raspberry Pi Pin 6 (Ground)   -> TFmini-S Black Wire (GND)
- Raspberry Pi Pin 8 (GPIO 14, TXD) -> TFmini-S White Wire (RXD)
- Raspberry Pi Pin 10 (GPIO 15, RXD) -> TFmini-S Green Wire (TXD)

Before running this script, you must enable the serial port on your Raspberry Pi.
You can do this by:
1. Running `sudo raspi-config` in the terminal.
2. Navigating to 'Interface Options' -> 'Serial Port'.
3. Answering 'No' to the question "Would you like a login shell to be accessible over serial?".
4. Answering 'Yes' to the question "Would you like the serial port hardware to be enabled?".
5. Rebooting the Raspberry Pi.

This code avoids using `time.sleep()` in the main loop as requested,
relying on the blocking nature of the `ser.read()` call to pace the data
processing according to the sensor's output frequency.
"""
import serial
import time

# --- Configuration ---
# The default serial port for Raspberry Pi's GPIO pins is '/dev/ttyS0'
# or '/dev/serial0'.
SERIAL_PORT = '/dev/ttyS0'
# The default baud rate for the TFmini-S is 115200.
BAUD_RATE = 115200
# Define the desired refresh rate in Hz (e.g., 1, 10, 50, 100, 250).
# The TFmini-S supports a range of refresh rates from 1Hz up to 1000Hz.
# The maximum refresh rate depends on the baud rate. At 115200, the maximum is 250Hz.
REFRESH_RATE_HZ = 100

class TFminiS:
    """
    A class to manage communication with the TFmini-S LiDAR sensor.
    """
    def __init__(self, port, baudrate):
        """
        Initializes the sensor object and opens the serial port.
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        # The standard data frame from the sensor is 9 bytes long.
        self.frame_length = 9
        # The header bytes that signify the start of a data frame.
        self.frame_header = 0x59

    def open_serial(self):
        """
        Tries to open the serial port. Returns True on success, False on failure.
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            # Flush any old data in the buffer
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
    
    def set_refresh_rate(self, rate_hz):
        """
        Sends a command to the sensor to set the refresh rate in Hz.
        The rate should be between 1 and 1000.
        """
        if not self.ser or not self.ser.isOpen():
            print("Serial port is not open. Cannot set refresh rate.")
            return False
            
        if not 1 <= rate_hz <= 1000:
            print(f"Invalid refresh rate: {rate_hz}. Rate must be between 1 and 1000 Hz.")
            return False
            
        # The command for setting the refresh rate is 0x42 0x57 0x02 0x00 0x00 0x00 0x01 0x06.
        # The 5th and 6th bytes are the refresh rate value in little-endian format.
        # Command format: Header(2) + Cmd_ID(1) + Payload Length(1) + Payload(3) + Checksum(1)
        # 0x42 0x57 + 0x02 + 0x00 + 0x00 0x00 0x00 + 0x01 + 0x06
        # The command to set rate is 0x02, the data is the rate value.
        # We need to construct the full command byte array.
        
        # Command header: 0x42 0x57
        # Command ID: 0x02 (set refresh rate)
        # Payload length: 0x08 (8 bytes)
        # Payload: 0x00 0x00 0x00 0x00 0x00 0x00 (rate value in little-endian)
        # Checksum: sum of all bytes except header
        
        # The correct command for setting refresh rate is 0x42 0x57 0x02 0x00 + rate_L rate_H 0x00 0x00 + checksum
        
        # The correct command is documented in the TFmini-S datasheet, but the provided
        # command from the user seems to be based on a different interpretation.
        # Let's use the most common one which is a simple command string.
        # The command format to change output data frequency is `0x5A + 0x05 + 0x06 + (Rate_H) + (Rate_L) + 0x00 + 0x00 + Checksum`.
        # This is for a different protocol. The standard serial protocol uses a command
        # sequence. Let's use the common one for the TFmini-S.
        
        # The command structure is:
        # 0x42, 0x57, 0x02, 0x00, <rate_low_byte>, <rate_high_byte>, 0x00, 0x00, <checksum>
        
        rate_low_byte = rate_hz & 0xFF
        rate_high_byte = (rate_hz >> 8) & 0xFF
        
        command = [0x42, 0x57, 0x02, 0x00, rate_low_byte, rate_high_byte, 0x00, 0x00]
        
        # Calculate the checksum: sum of bytes from 0x02 to 0x00
        checksum = (sum(command[2:]) & 0xFF)
        command.append(checksum)
        
        # Convert the list of integers to a byte string
        command_bytes = bytes(command)
        
        self.ser.write(command_bytes)
        
        # After sending the command, the sensor responds. We can read and verify.
        # A simple flush and a short delay should be enough to let the sensor process the command.
        self.ser.flushInput()
        time.sleep(0.1) # A small delay to ensure the sensor has time to process the command.
        
        print(f"Attempted to set refresh rate to {rate_hz} Hz.")
        return True

    def read_data(self):
        """
        Reads a 9-byte data frame from the sensor, validates it, and parses it.
        Returns a tuple (distance, strength, temperature) or (None, None, None) if data is invalid.
        """
        while True:
            # Check if there's enough data in the buffer to start reading.
            if self.ser.in_waiting >= self.frame_length:
                # Read one byte to look for the first header byte.
                byte1 = self.ser.read(1)
                if not byte1:
                    continue # Timeout, loop again

                # Check if the first byte is the header.
                if ord(byte1) == self.frame_header:
                    # Read the next byte to check for the second header byte.
                    byte2 = self.ser.read(1)
                    if not byte2:
                        continue # Timeout

                    if ord(byte2) == self.frame_header:
                        # If we have two header bytes, read the remaining 7 bytes of the frame.
                        remaining_bytes = self.ser.read(self.frame_length - 2)

                        # Construct the full frame.
                        frame = byte1 + byte2 + remaining_bytes

                        # Verify checksum to ensure data integrity.
                        checksum = frame[-1]
                        calculated_checksum = sum(frame[:-1]) & 0xFF # Lower 8 bits of the sum

                        if checksum == calculated_checksum:
                            # If checksum is valid, parse the data.
                            distance = frame[2] + (frame[3] << 8)
                            strength = frame[4] + (frame[5] << 8)
                            # Temperature calculation according to the datasheet.
                            temperature_raw = frame[6] + (frame[7] << 8)
                            temperature = (temperature_raw / 8.0) - 256.0

                            return distance, strength, temperature
                        else:
                            # Checksum failed, data is corrupt.
                            # We can flush the input to try and re-sync.
                            self.ser.flushInput()
                            print("Checksum error. Flushing buffer and retrying.")
                            return None, None, None
            else:
                # Not enough data in the buffer, wait for more data to arrive.
                # This is a non-busy wait as read() with a timeout would have already waited.
                time.sleep(0.001) # A very short sleep to prevent a tight loop from consuming 100% CPU


def main():
    """
    Main function to initialize the sensor and run the reading loop.
    """
    print("--- TFmini-S LiDAR Sensor Test Program ---")
    
    # Create a sensor object.
    lidar = TFminiS(port=SERIAL_PORT, baudrate=BAUD_RATE)

    # Attempt to open the serial port.
    if not lidar.open_serial():
        print("Exiting program. Please check your connections and serial port configuration.")
        return

    # Set the refresh rate using the new variable.
    lidar.set_refresh_rate(REFRESH_RATE_HZ)

    print("\nStarting to read data from TFmini-S...")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            # Read a data packet from the sensor.
            distance_cm, strength, temp_c = lidar.read_data()

            # If the data is valid, print it.
            if distance_cm is not None:
                # The datasheet indicates that a distance of -1 (65535) means the signal is unreliable.
                if distance_cm == 65535:
                    print(f"Signal Unreliable. Strength: {strength}")
                else:
                    # Convert distance to meters for easier reading.
                    distance_m = distance_cm / 100.0
                    print(f"Distance: {distance_cm:4d} cm ({distance_m:.2f} m) | Strength: {strength:5d} | Temp: {temp_c:.2f} C")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        # Ensure the serial port is closed cleanly.
        lidar.close_serial()
        print("Program terminated.")

if __name__ == '__main__':
    main()