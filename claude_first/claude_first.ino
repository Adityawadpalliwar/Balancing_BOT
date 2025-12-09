#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>

// ===== MPU6050 Setup =====
MPU6050 mpu(Wire);

// ===== Motor Pins =====
// Right Motor (Motor A)
#define MOTOR_R_PWM 5    // PWM pin for right motor speed
#define MOTOR_R_DIR1 4   // Direction pin 1
#define MOTOR_R_DIR2 7   // Direction pin 2

// Left Motor (Motor B)
#define MOTOR_L_PWM 6    // PWM pin for left motor speed
#define MOTOR_L_DIR1 8   // Direction pin 1
#define MOTOR_L_DIR2 9   // Direction pin 2

// ===== Encoder Setup =====
// Create Encoder objects (automatically handles interrupts)
Encoder encoder_right(2, 10);  // Right encoder pins A, B
Encoder encoder_left(3, 11);   // Left encoder pins A, B

// ===== State Variables =====
float x1 = 0.0;  // Average wheel position (rad)
float x2 = 0.0;  // Body pitch angle (rad)
float x3 = 0.0;  // Average wheel velocity (rad/s)
float x4 = 0.0;  // Body pitch rate (rad/s)

// Previous values for differentiation
long prev_encoder_right = 0;
long prev_encoder_left = 0;
unsigned long prev_time = 0;

// ===== LQR Gains =====
// K = [k1, k2, k3, k4]
const float K1 = 1.1;   // Position gain
const float K2 = 25.0;  // Angle gain
const float K3 = 1.2;   // Velocity gain
const float K4 = 4.5;   // Angular velocity gain

// ===== Motor Parameters =====
const float MAX_PWM = 255.0;
const float COUNTS_PER_REV = 960.0;  // N20 motor encoder counts per revolution (adjust for your motor)
const float WHEEL_RADIUS = 0.033;     // Wheel radius in meters (33mm)

// ===== Timing =====
const float DT = 0.01;  // 10ms loop time (100Hz)
unsigned long last_loop_time = 0;

// ===== Motor Control Functions =====
void setMotorSpeed(bool isRight, float speed) {
  // speed range: -255 to 255
  int pwm_pin = isRight ? MOTOR_R_PWM : MOTOR_L_PWM;
  int dir_pin1 = isRight ? MOTOR_R_DIR1 : MOTOR_L_DIR1;
  int dir_pin2 = isRight ? MOTOR_R_DIR2 : MOTOR_L_DIR2;
  
  // Constrain speed
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  
  // Set direction
  if (speed >= 0) {
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
  } else {
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, HIGH);
    speed = -speed;
  }
  
  // Set PWM
  analogWrite(pwm_pin, (int)speed);
}

void stopMotors() {
  setMotorSpeed(true, 0);
  setMotorSpeed(false, 0);
}

// ===== MPU6050 Initialization =====
void initMPU6050() {
  Wire.begin();
  Serial.println("Initializing MPU6050...");
  
  byte status = mpu.begin();
  
  if (status != 0) {
    Serial.print("MPU6050 connection failed! Error code: ");
    Serial.println(status);
    Serial.println("Check your wiring!");
    while (1);
  }
  
  Serial.println("MPU6050 connected!");
  
  // Calibrate gyroscope and accelerometer
  // IMPORTANT: Keep the robot perfectly still during calibration!
  Serial.println("Calibrating... Keep robot PERFECTLY still!");
  delay(1000);
  
  mpu.calcOffsets();  // This calculates gyro and accelerometer offsets
  
  Serial.println("Calibration complete!");
  Serial.println("Robot ready to balance!");
}

// ===== Read and Process IMU Data =====
void updateIMU() {
  // Update MPU6050 readings (this updates internal angle calculations)
  mpu.update();
  
  // Get pitch angle in radians (MPU6050_light uses complementary filter internally)
  // getAngleX() returns roll (rotation around X-axis, which is pitch for our setup)
  float pitch_degrees = mpu.getAngleX();  // Returns angle in degrees
  float pitch_radians = pitch_degrees * PI / 180.0;
  
  // Get gyroscope data (angular velocity in degrees/s)
  float gyro_x = mpu.getGyroX();  // deg/s
  float gyro_pitch_rate = gyro_x * PI / 180.0;  // Convert to rad/s
  
  // Update state variables
  x2 = -pitch_radians;      // Body pitch angle (negative to match convention)
  x4 = -gyro_pitch_rate;    // Body pitch rate
}

// ===== Update Encoder-based State Variables =====
void updateEncoders() {
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;  // Convert to seconds
  
  if (dt >= DT) {
    // Read encoder counts (Encoder library handles everything automatically)
    long curr_encoder_right = encoder_right.read();
    long curr_encoder_left = encoder_left.read();
    
    // Calculate velocities (rad/s)
    float delta_right = (curr_encoder_right - prev_encoder_right) * 2.0 * PI / COUNTS_PER_REV;
    float delta_left = (curr_encoder_left - prev_encoder_left) * 2.0 * PI / COUNTS_PER_REV;
    
    float vel_right = delta_right / dt;
    float vel_left = delta_left / dt;
    
    // Update state variables
    x3 = (vel_right + vel_left) / 2.0;  // Average velocity
    
    // Update position (integrate velocity)
    x1 += x3 * dt;  // Average wheel position
    
    // Store previous values
    prev_encoder_right = curr_encoder_right;
    prev_encoder_left = curr_encoder_left;
    prev_time = current_time;
  }
}

// ===== LQR Control =====
void computeLQRControl() {
  // LQR control law: U = -K * x
  float U_balance = -(K1 * x1 + K2 * x2 + K3 * x3 + K4 * x4);
  
  // Convert control signal to PWM (scale appropriately)
  float pwm_scale = 8.0;  // Tune this value based on your robot's response
  float U_right = U_balance * pwm_scale;
  float U_left = U_balance * pwm_scale;
  
  // Safety check: if robot is too tilted, stop motors
  if (abs(x2) > 0.8) {  // ~45 degrees
    stopMotors();
    Serial.println("Robot fell! Stopping motors.");
    return;
  }
  
  // Apply motor commands
  setMotorSpeed(true, U_right);
  setMotorSpeed(false, U_left);
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  
  // Motor pins
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_DIR1, OUTPUT);
  pinMode(MOTOR_R_DIR2, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_DIR1, OUTPUT);
  pinMode(MOTOR_L_DIR2, OUTPUT);
  
  // Initialize motors to stopped
  stopMotors();
  
  // Initialize IMU
  initMPU6050();
  
  // Initialize timing
  prev_time = millis();
  last_loop_time = micros();
  
  Serial.println("Self-Balancing Robot Ready!");
  Serial.println("Starting in 2 seconds...");
  delay(2000);
}

// ===== Main Loop =====
void loop() {
  unsigned long current_time = micros();
  
  // Run control loop at fixed rate (100Hz)
  if (current_time - last_loop_time >= DT * 1000000) {
    last_loop_time = current_time;
    
    // Update sensor readings
    updateIMU();
    updateEncoders();
    
    // Compute and apply control
    computeLQRControl();
    
    // Debug output (comment out for better performance)
    // Uncomment the following lines to see state variables in Serial Monitor
    /*
    Serial.print("Pos:"); Serial.print(x1, 3);
    Serial.print(" Angle:"); Serial.print(x2 * 180.0 / PI, 2);  // Show in degrees
    Serial.print(" Vel:"); Serial.print(x3, 3);
    Serial.print(" Rate:"); Serial.println(x4, 3);
    */
  }
}