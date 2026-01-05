/**************** MOTOR PINS ****************/
#define PWM_L 5
#define DIR_L A2

#define PWM_R 6
#define DIR_R 4

/**************** ENCODER PINS ****************/
#define ENCODER_L_A 2   // interrupt pin
#define ENCODER_R_A 8  // interrupt pin


/**************** ENCODER VARIABLES ****************/
volatile long countL = 0;
volatile long countR = 0;

/**************** TIME CONTROL ****************/
unsigned long prevTime = 0;
const unsigned long interval = 100; // ms

/**************** ENCODER & WHEEL ****************/
const int PPR = 600;               // pulses per revolution
const float WHEEL_RADIUS = 0.03;   // meters

/**************** CONTROL ****************/
int testPWM = 120;   // SAME PWM for both motors
float tolerance = 0.02; // m/s allowed difference

/**************** ISR ****************/
void encoderL_ISR() {
  countL++;
}

void encoderR_ISR() {
  countR++;
}

/**************** SETUP ****************/
void setup() {
  Serial.begin(9600);

  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderR_ISR, RISING);

  // Same direction for both motors
  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, HIGH);

  // SAME PWM APPLIED
  analogWrite(PWM_L, testPWM);
  analogWrite(PWM_R, testPWM);

  Serial.println("Same PWM Velocity Test Started");
}

/**************** LOOP ****************/
void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime >= interval) {

    noInterrupts();
    long pulsesL = countL;
    long pulsesR = countR;
    countL = 0;
    countR = 0;
    interrupts();

    float dt = interval / 1000.0;

    // Angular velocity
    float omegaL = (pulsesL * 2.0 * PI) / (PPR * dt);
    float omegaR = (pulsesR * 2.0 * PI) / (PPR * dt);

    // Linear velocity
    float vL = omegaL * WHEEL_RADIUS;
    float vR = omegaR * WHEEL_RADIUS;

    // Output
    Serial.print("PWM: ");
    Serial.print(testPWM);
    Serial.print(" | Left: ");
    Serial.print(vL, 3);
    Serial.print(" m/s | Right: ");
    Serial.print(vR, 3);
    Serial.print(" m/s | Diff: ");
    Serial.println(abs(vL - vR), 3);
    /*
    if (abs(vL - vR) < tolerance) {
      Serial.println("  --> MOTORS MATCHED");
    } else {
      Serial.println("  --> MOTORS NOT MATCHED");
    }
    */
    prevTime = currentTime;
  }
}
