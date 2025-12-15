#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

#define encodPinAR 8
#define encodPinBR 7
#define encodPinAL 2
#define encodPinBL 3

const int buzz = A1; 
unsigned long imuTimer = 0;
unsigned long printTimer = 0; 

// Right Motor (Motor A)
#define MOTOR_R_PWM 6    
const int MOTOR_R_DIR1 = A2;    
const int MOTOR_R_DIR2 = A3;    

// Left Motor (Motor B)
#define MOTOR_L_PWM 5    
const int MOTOR_L_DIR1 = 4;   
const int MOTOR_L_DIR2 = 9;

// Previous values for differentiation
long prev_encoder_right = 0;
long prev_encoder_left = 0;

// For timings
const float DT = 0.01f;  // 10ms loop time 
unsigned long last_loop_time = 0;
unsigned long prev_time = 0;

// ===== Motor Parameters =====
const float MAX_PWM = 255.0;
const float COUNTS_PER_REV = 700.0;  // Encoder counts per revolution

volatile long wheel_pulse_count_left = 0;
volatile long wheel_pulse_count_right = 0;
int prevA_right = LOW;
int prevA_left = LOW;

// ===== LQR Gains =====
const float K1 = -;
const float K2 = -;
const float K3 = -;
const float K4 = -;


// State variables
float x1 = 0.0;  // Average wheel position (rad)
float x2 = 0.0;  // Body pitch angle (rad)
float x3 = 0.0;  // Average wheel velocity (rad/s)
float x4 = 0.0;  // Body pitch rate (rad/s)

// MPU offset (calibrate this for your robot)
float angleZeroY = 0.0;  // Set this to the angle when robot is balanced

void getimu() {
  mpu.update();
  
  // Get pitch angle and angular velocity
  // Convert to radians and apply offset
  x2 = (mpu.getAngleY() - angleZeroY) * PI / 180.0;  // Body pitch angle
  x4 = mpu.getGyroY() * PI / 180.0;                   // Body pitch rate (rad/s)
}

void updateEncoders() {
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;  // Convert to seconds
  
  // Right encoder - polling method
  int aR = digitalRead(encodPinAR);
  if (prevA_right == LOW && aR == HIGH) {
    if (digitalRead(encodPinBR) == HIGH) {
      wheel_pulse_count_right++;
    } else {
      wheel_pulse_count_right--;
    }
  }
  prevA_right = aR;

  // Left encoder - polling method
  int aL = digitalRead(encodPinAL);
  if (prevA_left == LOW && aL == HIGH) {
    if (digitalRead(encodPinBL) == HIGH) {
      wheel_pulse_count_left--;  // Note: reversed direction
    } else {
      wheel_pulse_count_left++;
    }
  }
  prevA_left = aL;
  
  // Calculate velocities and position at DT intervals
  if (dt >= DT) {
    // Read encoder counts
    long curr_encoder_right = wheel_pulse_count_right;
    long curr_encoder_left = wheel_pulse_count_left;
    
    // Calculate angular displacement (rad)
    float delta_right = (curr_encoder_right - prev_encoder_right) * 2.0 * PI / COUNTS_PER_REV;
    float delta_left = (curr_encoder_left - prev_encoder_left) * 2.0 * PI / COUNTS_PER_REV;
    
    // Calculate angular velocities (rad/s)
    float vel_right = delta_right / dt;
    float vel_left = delta_left / dt;
    
    // Update state variables
    x3 = (vel_right + vel_left) / 2.0;  // Average velocity (rad/s)
    x1 += x3 * dt;  // Integrate to get position (rad)
    
    // Store previous values
    prev_encoder_right = curr_encoder_right;
    prev_encoder_left = curr_encoder_left;
    prev_time = current_time;
  }
}

void setMotorSpeed(bool isRight, float speed) {
  int pwm_pin = isRight ? MOTOR_R_PWM : MOTOR_L_PWM;
  int dir_pin1 = isRight ? MOTOR_R_DIR1 : MOTOR_L_DIR1;
  int dir_pin2 = isRight ? MOTOR_R_DIR2 : MOTOR_L_DIR2;
  
  // Constrain speed to valid PWM range
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  
  // Set direction based on sign
  if (speed >= 0) {
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, HIGH);
    analogWrite(pwm_pin, (int)abs(speed));
  } else {
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
    analogWrite(pwm_pin, (int)abs(speed));
  }
}

void stopMotors() {
  setMotorSpeed(true, 0);
  setMotorSpeed(false, 0);
}

void computeLQRControl() {
  // LQR control law: U = -K * x
  float U_balance = -(K1 * x1 + K2 * x2 + K3 * x3 + K4 * x4);
  
  // Safety check: if robot is too tilted, stop motors
  if (abs(x2 * 180.0 / PI) > 45.0) {  // Convert to degrees for check (45 degrees)
    stopMotors();
    Serial.println("Robot fell! Stopping motors.");
    return;
  }
  
  // Scale the control signal
  // Option 1: Direct scaling
  int U_int = (int)(U_balance * 90);  // Multiply by 100 to preserve precision
  
  // Step 2: Constrain to expected input range
  U_int = constrain(U_int, -600, 600);  // Adjust these limits based on your U_balance range
  
  // Step 3: Map to PWM range
  float U_right = map(U_int, -600, 600, -190, 190);  // Map to ±200 PWM
  float U_left = U_right;
  
  // Option 2: Map method (uncomment to use instead)
  // int U_int = (int)(U_balance * 100);  // Scale up for better mapping
  // float U_right = map(constrain(U_int, -400, 400), -400, 400, -200, 200);
  // float U_left = U_right;
  
  // Apply motor commands
  setMotorSpeed(true, U_right);
  setMotorSpeed(false, U_left);
  
  // Debug output
  Serial.print("x1:");
  Serial.print(x1, 3);
  Serial.print(" x2:");
  Serial.print(x2 * 180.0 / PI, 2);  // Print angle in degrees
  Serial.print(" x3:");
  Serial.print(x3, 3);
  Serial.print(" x4:");
  Serial.print(x4, 3);
  Serial.print(" U:");
  Serial.println(U_balance, 2);
}

void setup() {
  Serial.begin(115200);  // Changed to 115200 for faster serial
  Wire.begin();
  
  // Initialize MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 connection failed! Status: ");
    Serial.println(status);
    while (1) {
      digitalWrite(buzz, HIGH);
      delay(100);
      digitalWrite(buzz, LOW);
      delay(100);
    }
  }
  
  Serial.println("MPU6050 connected!");
  Serial.println("Calibrating... Keep robot still!");
  
  mpu.calcOffsets();  // Calibrate gyro and accelerometer
  
  Serial.println("Calibration complete!");
  
  // Find zero angle - hold robot upright and uncomment these lines:
  // delay(2000);
  // angleZeroY = mpu.getAngleY();
  // Serial.print("Zero angle set to: ");
  // Serial.println(angleZeroY);
  
  // Encoder pins
  pinMode(encodPinAL, INPUT_PULLUP);
  pinMode(encodPinBL, INPUT_PULLUP);
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);
  
  // Motor pins
  pinMode(MOTOR_R_PWM, OUTPUT);   
  pinMode(MOTOR_L_PWM, OUTPUT);   
  pinMode(MOTOR_R_DIR1, OUTPUT);
  pinMode(MOTOR_R_DIR2, OUTPUT);
  pinMode(MOTOR_L_DIR1, OUTPUT);
  pinMode(MOTOR_L_DIR2, OUTPUT);
  
  pinMode(buzz, OUTPUT); 
  
  // Initialize encoder state
  prevA_right = digitalRead(encodPinAR);
  prevA_left = digitalRead(encodPinAL);
  
  // Initialize timers
  imuTimer = millis();
  printTimer = millis();
  prev_time = millis();
  
  // Beep to indicate ready
  digitalWrite(buzz, HIGH);
  delay(200);
  digitalWrite(buzz, LOW);
  
  Serial.println("Self-Balancing Robot Ready!");
}

void loop() {
  unsigned long now = millis();
  
  // 1) Always service encoders (runs every loop iteration for accurate counting)
  updateEncoders();
  
  // 2) IMU update every 5 ms (200 Hz)
  if (now - imuTimer >= 5) {
    imuTimer = now;
    getimu();
  }
  
  // 3) Control update every 10 ms (100 Hz)
  if (now - printTimer >= 10) {
    printTimer = now;
    computeLQRControl();
  }
}
