#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <Encoder.h>


// Create Servo objects (if needed for future use)
Servo servo1;
Servo servo2;


MPU6050 mpu(Wire);
Encoder encoder_right(8, 7);  // assuming encoder 1 ( 7,8 ) are channel a and b
Encoder encoder_left(2, 3);   // Left encoder pins A, B


// Pin definitions
#define encodPinAR 8
#define encodPinBR 7
#define encodPinAL 2
#define encodPinBL 3

const int buzz = A1; 

// Right Motor (Motor A)
#define MOTOR_R_PWM 6    
const int MOTOR_R_DIR1 = A2;    
const int MOTOR_R_DIR2 = A3;    

// Left Motor (Motor B)
#define MOTOR_L_PWM 5    
const int MOTOR_L_DIR1 = 4;   
const int MOTOR_L_DIR2 = 9;

// Timing variables
const float DT = 0.01f;  // 10ms loop time
unsigned long imuTimer = 0;
unsigned long controlTimer = 0;
unsigned long encoderTimer = 0;
unsigned long lastUpdateTime=0;

// Motor Parameters
const float MAX_PWM = 255.0;
const float COUNTS_PER_REV = 700.0;

// Encoder variables
volatile long wheel_pulse_count_left = 0;
volatile long wheel_pulse_count_right = 0;
long prev_encoder_right = 0;
long prev_encoder_left = 0;
int prevA_right = LOW;
int prevA_left = LOW;

// ===== PID PARAMETERS =====

// Velocity PID (start with these, tune as needed)
float Kp_vel = 0.0;      
float Ki_vel = 0.0;     
float Kd_vel = 0.0;      
// Pitch PID 
float Kp_pitch = 5440.0;   
float Ki_pitch = 0.000;    
float Kd_pitch = 0.15;    

// Setpoints
float vel_setpoint = 0.0;    
float pitch_setpoint = 0.0;

// PID variables
float error_pitch = 0;
float err_int_pitch = 0;
float err_der_pitch = 0;
float control_pitch = 0;
float err_prev_pitch = 0;

float error_vel = 0;
float err_int_vel = 0;
float err_der_vel = 0;
float control_vel = 0;
float err_prev_vel = 0;

// Integral windup limits
const float MAX_INT_PITCH = 100.0;
const float MAX_INT_VEL = 100.0;

// State variables
float x1 = 0.0;  // Average wheel position (rad)
float x2 = 0.0;  // Body pitch angle (rad)
float x3 = 0.0;  // Average wheel velocity (rad/s)
float x4 = 0.0;  // Body pitch rate (rad/s)

// MPU offset
//float angleZeroY = 0.0;


void updatesensor() 
{
 

  mpu.update();
  
  // Get pitch angle and angular velocity
  x2 = (mpu.getAngleY()) * PI / 180.0;  // Body pitch angle (rad)
  x4 = (mpu.getGyroY()) * PI / 180.0;                   // Body pitch rate (rad/s)

  long curr_encoder_right = encoder_right.read();
  long curr_encoder_left = encoder_left.read();
  
  x3 = ((curr_encoder_right-prev_encoder_right)+(curr_encoder_left - prev_encoder_left)) / (2.0*DT);  // Average velocity
  //x3= ((enc_A - enc_A_prev) + (enc_B - enc_B_prev)) / (2*dt);
  //x1 += x3 * DT;  // Integrate to get position
  // Store previous values
  prev_encoder_right = curr_encoder_right;
  prev_encoder_left = curr_encoder_left;
}


void setMotorSpeed(bool isRight, float speed) {
  int pwm_pin = isRight ? MOTOR_R_PWM : MOTOR_L_PWM;
  int dir_pin1 = isRight ? MOTOR_R_DIR1 : MOTOR_L_DIR1;
  int dir_pin2 = isRight ? MOTOR_R_DIR2 : MOTOR_L_DIR2;
  
  
  if (speed >= 0) {
    analogWrite(pwm_pin, (int)abs(speed));
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, HIGH);
    
  } else {
    analogWrite(pwm_pin, (int)abs(speed));
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
  }
}

void stopMotors() {
  setMotorSpeed(true, 0);
  setMotorSpeed(false, 0);
}

void computeCascadePID() {
  // Safety check - 35 degrees is more realistic fall threshold
  if (abs(x2 * 180.0 / PI) > 35.0) {
    stopMotors();
    err_int_pitch = 0.0;
    err_int_vel = 0.0;
    return;
  }
  // ===== PITCH PID (Inner Loop) =====
  error_pitch = pitch_setpoint - x2;
  err_int_pitch += (error_pitch * DT);
  
  // Anti-windup for pitch integral
  //err_int_pitch = constrain(err_int_pitch, -MAX_INT_PITCH, MAX_INT_PITCH);
  
  //err_der_pitch = -x4;
  err_der_pitch= (error_pitch - err_prev_pitch)/DT;// got an alternative for der
  control_pitch = (Kp_pitch * error_pitch) + (Ki_pitch * err_int_pitch) + (Kd_pitch * err_der_pitch);
  err_prev_pitch = error_pitch;

  // ===== VELOCITY PID (Outer Loop) =====
  error_vel = vel_setpoint - x3;
  err_int_vel += (error_vel * DT);
  
  // Anti-windup for velocity integral
  //err_int_vel = constrain(err_int_vel, -MAX_INT_VEL, MAX_INT_VEL);
  
  err_der_vel = (error_vel - err_prev_vel) / DT;
  control_vel = (Kp_vel * error_vel) + (Ki_vel * err_int_vel) + (Kd_vel * err_der_vel);
  err_prev_vel = error_vel;

  // Combine control outputs
  float U_right = (control_pitch + control_vel);
  float U_left = (control_pitch + control_vel);

  // Constrain to valid PWM range
  U_right = constrain(U_right, -MAX_PWM, MAX_PWM);
  U_left = constrain(U_left, -MAX_PWM, MAX_PWM);
  
  // Send to motors
  setMotorSpeed(true, U_right);
  setMotorSpeed(false, U_left);
  
  // ===== Debug Output (Uncomment for tuning) =====
  /*
  Serial.print("Pitch:");
  Serial.print(x2 * 180.0 / PI, 2);
  Serial.print(" Vel:");
  Serial.print(x3, 2);
  Serial.print(" PWM:");
  Serial.println(U_right);
  */
}

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  byte status =mpu.begin();
  while(status!=0){ }
  delay(1000);
  // Sample the upright angle
  lastUpdateTime = millis();
  while(millis()-lastUpdateTime <= 1000)
  mpu.calcOffsets();
  lastUpdateTime = millis();
  while(millis()-lastUpdateTime <= 3000) mpu.update();

  
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
  encoderTimer = millis();
  
  // Beep to indicate ready
  digitalWrite(buzz, HIGH);
  delay(200);
  digitalWrite(buzz, LOW);
  
  // Optional: Initialize servos if used
  servo1.attach(10);
  servo2.attach(11);
  servo1.write(0);
  servo2.write(0);


}

void loop() {
  unsigned long now = millis();

  if((now-lastUpdateTime >= DT*1000))
  {
    lastUpdateTime = now;
    updatesensor();
    computeCascadePID();
   
  }

}