// Teensy 4.1 Built-in LED is on pin 13

void setup() {
  pinMode(13, OUTPUT);      // Set onboard LED as output
  Serial.begin(9600);       // Start serial communication
  // while (!Serial) {
  //   ; // Wait for serial port to connect (important for Teensy)
  // }
  Serial.println("Teensy 4.1 is alive!");
}

void loop() {
  digitalWrite(13, HIGH);   // Turn LED on
  Serial.println("LED ON");
  delay(100);

  digitalWrite(13, LOW);    // Turn LED off
  Serial.println("LED OFF");
  delay(100);
}
