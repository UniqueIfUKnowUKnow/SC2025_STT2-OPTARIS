import serial
import time
import random

ARDUINO_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200
SEND_INTERVAL_SECONDS = 3

class dataExtract:
    def extract_digits(self, received_value: int) -> dict:
        servo_angle = received_value % (10**4)
        motor_angle = (received_value // (10**4)) % (10**5)
        
        return {
            'motor_angle': motor_angle,
            'servo_angle': servo_angle
        }

def main():
    data_extractor = dataExtract()
    ser = None

    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully opened serial port {ARDUINO_PORT} at {BAUD_RATE} baud.")
        time.sleep(2) 
        print("Starting communication with Arduino...")

        while True:
            value_to_send_pi_to_arduino = random.randint(10000, 99999)
            message_to_send = f"{value_to_send_pi_to_arduino}\n"
            ser.write(message_to_send.encode('utf-8'))
            print(f"\n[RPi -> Arduino] Sent 5-digit number: {value_to_send_pi_to_arduino}")
            print("[RPi <- Arduino] Waiting for 8-digit response...")
            line = ser.readline().decode('utf-8').rstrip()
            
            if line:
                try:
                    received_19_digit_number = int(line)
                    print(f"[RPi <- Arduino] Received 8-digit number: {received_19_digit_number}")
                    print(f"                                  (Type: {type(received_19_digit_number)})")
                    extracted_data = data_extractor.extract_digits(received_19_digit_number)
                    
                    motor_angle = extracted_data['motor_angle']
                    servo_angle = extracted_data['servo_angle']
                    
                    print(f"Extracted motorAngle (digits 1-4):    {motor_angle}")
                    print(f"Extracted servoAngle (digits 5-8):    {servo_angle}")

                except ValueError as e:
                    print(f"Error: Could not convert received data to integer: '{line}'. Error: {e}")
                except Exception as e:
                    print(f"An error occurred during data extraction: {e}")
            else:
                print("[RPi <- Arduino] No data received from Arduino (timeout).")
            
            time.sleep(SEND_INTERVAL_SECONDS)

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {ARDUINO_PORT}. {e}")
        print("Please ensure the Arduino is connected, the port name is correct, and you have appropriate permissions (e.g., add user to 'dialout' group: 'sudo adduser $USER dialout').")
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Closing serial port...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()