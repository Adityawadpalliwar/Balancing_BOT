// Define encoder pins
#define encodPinAR 7  // Encoder A pin (C2) // A1
#define encodPinBR 2   // Encoder B pin (C1) // A2//2

#define encodPinAL 8  // Encoder A pin (C2) ------ working
#define encodPinBL 3   // Encoder B pin (C1) ------ working

// // Define encoder pins
// #define encodPinAR 7  // Encoder A pin (C2) // A1
// #define encodPinBR 3   // Encoder B pin (C1) // A2//2

// #define encodPinAL 8  // Encoder A pin (C2) ------ working
// #define encodPinBL 2   // Encoder B pin (C1) ------ working

// Variable for wheel angle
int wheel_pulse_count_left = 0;
int wheel_pulse_count_right = 0;
float wheel_angle_left = 0.0, wheel_angle_old_left = 0.0, wheel_dot_left = 0.0;
float wheel_angle_right = 0.0, wheel_angle_old_right = 0.0, wheel_dot_right = 0.0;



void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  //------Encoder HW Interrupt setup-----//
  pinMode(8, INPUT_PULLUP);
  digitalWrite(8, HIGH);
  // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

  pinMode(3, INPUT_PULLUP);
  digitalWrite(3, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), mot_rencoder_left, RISING);
  // ---------------------------------------------- ///

  //------Encoder HW Interrupt setup-----//
  pinMode(7, INPUT_PULLUP);  //10
  digitalWrite(7, HIGH);
  // attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);

  pinMode(2, INPUT_PULLUP);  //11
  digitalWrite(2, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), mot_rencoder_right, RISING);
  // ---------------------------------------------- ///
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print( wheel_pulse_count_left);
  Serial.print(" ; ");
  Serial.println(wheel_pulse_count_right);
  // Serial.print(digitalRead(encodPinBR));
  // Serial.print(" ; ");
  // Serial.println(digitalRead(encodPinAR));

}

// ISR for motor encoder
void mot_rencoder_left() {
  // if (digitalRead(encodPinBR) == HIGH) {
  if (digitalRead(encodPinBL) > digitalRead(encodPinAL)) {
    wheel_pulse_count_left = wheel_pulse_count_left + 1;
  } 
  else {
    wheel_pulse_count_left = wheel_pulse_count_left - 1;
  }
}

void mot_rencoder_right() {
  // if (digitalRead(encodPinBR) == HIGH) {
  if (digitalRead(encodPinBR) > digitalRead(encodPinAR)) {
    wheel_pulse_count_right = wheel_pulse_count_right - 1;
  }
  else {
    wheel_pulse_count_right = wheel_pulse_count_right + 1;
  }
}
