#include <Encoder.h>

// Encoder pin definitions


// Create encoder objects
Encoder rightEncoder(8,7);
Encoder leftEncoder(2,3);

// Pulse count variables
long wheel_pulse_count_left  = 0;
long wheel_pulse_count_right = 0;

void setup() {
  Serial.begin(9600);

  // Optional but safe
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
}

void loop() {

  // Read encoder counts
  wheel_pulse_count_right = rightEncoder.read();
  wheel_pulse_count_left  = leftEncoder.read();

  // Print every 20 ms (same as your code)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 20) {
    lastPrint = millis();
    Serial.print(wheel_pulse_count_left);
    Serial.print(" ; ");
    Serial.println(wheel_pulse_count_right);
  }
}
