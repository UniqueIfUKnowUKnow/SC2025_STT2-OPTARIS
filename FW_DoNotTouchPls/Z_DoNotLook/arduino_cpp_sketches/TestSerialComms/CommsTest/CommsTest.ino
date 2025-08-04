void setup() {
  // Initialize serial communication at 9600 bits per second.
  // This must match the baud rate used by the Python script.
  Serial.begin(9600); 

  // Print a startup message to the serial monitor.
  // This helps confirm the Arduino is ready and listening.
  Serial.println("Arduino Uno R4 Minima Ready!");
  Serial.println("Waiting for integer data from Raspberry Pi...");
}

void loop() {
  // Check if there is any data available to read in the serial buffer.
  // Serial.available() returns the number of bytes available.
  if (Serial.available() > 0) {
    // Read the incoming string until a newline character ('\n') is received.
    // The Python script will send a newline after each number.
    String receivedString = Serial.readStringUntil('\n');
    
    // Attempt to convert the received string to an integer.
    // toInt() will return 0 if the string does not contain a valid integer,
    // or if the integer is out of range for an int type.
    int receivedValue = receivedString.toInt();

    // Print a confirmation that a value was received.
    Serial.print("--- Received an integer: ");
    Serial.println(receivedValue);

    // Simulate controlling a motor with the received value.
    // In a real application, you would use this 'receivedValue'
    // to control a motor's speed (e.g., via PWM on a pin) or position.
    // For this example, we just print the value as if controlling the motor.
    Serial.print("--- Simulating motor control: Setting motor to value ");
    Serial.println(receivedValue);
    Serial.println("--------------------------------");

    // Optional: Add a small delay to prevent rapid processing in case
    // of continuous data flow, though not strictly necessary here.
    // delay(10); 
  }
}