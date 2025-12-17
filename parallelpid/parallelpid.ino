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
unsigned long controlTimer = 0; 

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
const float dt = 0.01f;
unsigned long last_loop_time = 0;
unsigned long prev_time = 0;

// ===== Motor Parameters =====
const float MAX_PWM = 255.0;
const float COUNTS_PER_REV = 700.0;  // Encoder counts per revolution

volatile long wheel_pulse_count_left = 0;
volatile long wheel_pulse_count_right = 0;
int prevA_right = LOW;
int prevA_left = LOW;

// ===== TWO-LOOP CASCADE PID GAINS =====

// VEL PID (outputs desired pitch angle)
float Kp_vel = 0;      // Position proportional gain
float Ki_vel = 0;      // Position integral gain (keep 0 for balancing)
float Kd_vel = 0;      // Position derivative gain

// INNER LOOP - Pitch PID (outputs motor PWM)
float Kp_pitch = 1400.0;   // Pitch proportional gain (MAIN TUNING PARAMETER)
float Ki_pitch = 0.0;    // Pitch integral gain (usually 0)
float Kd_pitch = 0.0;    // Pitch derivative gain (damping)

// Setpoints
float vel_setpoint = 0.0;    // Desired position (rad) - where robot should be
float pitch_setpoint = 0.0;       // Desired pitch from position PID (rad)


// PID CONSTANTS
float error_pitch=0,err_int_pitch=0,err_der_pitch=0,control_pitch=0,err_prev_pitch=0;
float error_vel=0,err_int_vel=0,err_der_vel=0,control_vel=0,err_prev_vel=0;

// OUTER LOOP - Position PID variables
float position_error = 0.0;
float position_error_sum = 0.0;
float position_error_prev = 0.0;

// INNER LOOP - Pitch PID variables
float pitch_error = 0.0;
float pitch_error_sum = 0.0;
float pitch_error_prev = 0.0;

// State variables
float x1 = 0.0;  // Average wheel position (rad)
float x2 = 0.0;  // Body pitch angle (rad)
float x3 = 0.0;  // Average wheel velocity (rad/s)
float x4 = 0.0;  // Body pitch rate (rad/s)

// MPU offset
float angleZeroY = 0.0;  // Balanced angle offset

void getimu() {
  mpu.update();
  
  // Get pitch angle and angular velocity
  x2 = (mpu.getAngleY() - angleZeroY) * PI / 180.0;  // Body pitch angle (rad)

  Serial.println(x2);
  x4 = mpu.getGyroY() * PI / 180.0;                   // Body pitch rate (rad/s)
}

void updateEncoders() {
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;
  
  // Right encoder
  int aR = digitalRead(encodPinAR);
  if (prevA_right == LOW && aR == HIGH) {
    if (digitalRead(encodPinBR) == HIGH) {
      wheel_pulse_count_right++;
    } else {
      wheel_pulse_count_right--;
    }
  }
  prevA_right = aR;

  // Left encoder
  int aL = digitalRead(encodPinAL);
  if (prevA_left == LOW && aL == HIGH) {
    if (digitalRead(encodPinBL) == HIGH) {
      wheel_pulse_count_left--;
    } else {
      wheel_pulse_count_left++;
    }
  }
  prevA_left = aL;
  
  // Calculate velocities and position
  if (dt >= DT) {
    long curr_encoder_right = wheel_pulse_count_right;
    long curr_encoder_left = wheel_pulse_count_left;
    
    // Angular displacement (rad)
    float delta_right = (curr_encoder_right - prev_encoder_right) * 2.0 * PI / COUNTS_PER_REV;
    float delta_left = (curr_encoder_left - prev_encoder_left) * 2.0 * PI / COUNTS_PER_REV;
    
    // Angular velocities (rad/s)
    float vel_right = delta_right / dt;
    float vel_left = delta_left / dt;
    
    // Update state variables
    x3 = (vel_right + vel_left) / 2.0;  // Average velocity
    x1 += x3 * dt;  // Integrate to get position
    Serial.println(x3);
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
  
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  
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

void computeCascadePID() {
  // Safety check
  if (abs(x2 * 180.0 / PI) > 45.0) {  // 45 degrees
    stopMotors();
    
    // Reset integral terms
    position_error_sum = 0.0;
    pitch_error_sum = 0.0;
    
    //Serial.println("Robot fell! Stopping motors.");
    return;
  }
  // pitch pid
  error_pitch = pitch_setpoint - x2;
  err_int_pitch += (error_pitch * dt);
  err_der_pitch = (error_pitch - err_prev_pitch) / (dt);
  control_pitch = (Kp_pitch * error_pitch) + (Ki_pitch * err_int_pitch) + (Kd_pitch * err_der_pitch);
  err_prev_pitch = error_pitch;


  //vel pid
  error_vel = vel_setpoint - x3;
  err_int_vel += (error_vel * dt);
  err_der_vel = (error_vel - err_prev_vel) / (dt);
  control_vel = (Kp_vel * error_vel) + (Ki_vel * err_int_vel) + (Kd_vel * err_der_vel);
  err_prev_vel = error_vel;

  float U_right = -(control_pitch+control_vel);
  float U_left = -(control_pitch+control_vel);

  // Constrain to valid PWM range
  U_right = constrain(U_right, -MAX_PWM, MAX_PWM);
  U_left = constrain(U_left, -MAX_PWM, MAX_PWM);
  
  // Send to motors
  setMotorSpeed(true, U_right);
  setMotorSpeed(false, U_left);
  
  // ========================================
  // Debug Output
  // ========================================
  /*Serial.print("Pos:");
  Serial.print(x1, 3);
  Serial.print(" Pos_err:");
  Serial.print(position_error, 3);
  Serial.print(" Pitch:");
  Serial.print(x2 * 180.0 / PI, 2);  // degrees
  Serial.print(" Pitch_sp:");
  Serial.print(pitch_setpoint * 180.0 / PI, 2);  // degrees
  Serial.print(" Pitch_err:");
  Serial.print(pitch_error * 180.0 / PI, 2);  // degrees
  Serial.print(" PWM:");
  Serial.println(U_right);*/
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    //Serial.print("MPU6050 connection failed! Status: ");
    //Serial.println(status);
    while (1) {
      digitalWrite(buzz, HIGH);
      delay(100);
      digitalWrite(buzz, LOW);
      delay(100);
    }
  }
  
  //Serial.println("MPU6050 connected!");
  //Serial.println("Calibrating... Keep robot still!");
  
  mpu.calcOffsets();
  
  //Serial.println("Calibration complete!");
  
  // Calibrate zero angle
  delay(2000);
  Serial.println("Hold robot upright NOW!");
  delay(2000);
  angleZeroY = mpu.getAngleY();
  Serial.print("Zero angle set to: ");
  Serial.println(angleZeroY);
  
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
  controlTimer = millis();
  prev_time = millis();
  
  // Beep to indicate ready
  digitalWrite(buzz, HIGH);
  delay(200);
  digitalWrite(buzz, LOW);
  
  Serial.println("2-Loop Cascade PID Ready!");
  Serial.println("Outer: Position PID | Inner: Pitch PID");
}

void loop() {
  unsigned long now = millis();
  
  // Always service encoders
  updateEncoders();
  
  // IMU update every 5 ms (200 Hz)
  if (now - imuTimer >= 5) {
    imuTimer = now;
    getimu();
  }
  
  // Control update every 10 ms (100 Hz)
  if (now - controlTimer >= 10) {
    controlTimer = now;
    computeCascadePID();
  }
}