#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>
MPU6050 mpu(Wire);


//float angleZeroY;
//float gyroOffsetY;

const int buzz  = A1; 

// Right Motor (Motor A)(right) you view gripper in your frame of reference 
#define MOTOR_R_PWM 6    
const int MOTOR_R_DIR1  = A2;    
const int MOTOR_R_DIR2 = A3;    

// Left Motor (Motor B)
#define MOTOR_L_PWM 5    
const int MOTOR_L_DIR1 =9;   
const int MOTOR_L_DIR2 =4;

// Create Encoder objects (automatically handles interrupts)
Encoder encoder_right(7, 8);  // assuming encoder 1 ( 7,8 ) are channel a and b
Encoder encoder_left(2, 3);   // Left encoder pins A, B

// Previous values for differentiation
long prev_encoder_right = 0;
long prev_encoder_left = 0;

// for timmings
const float DT = 0.01f;  // 10ms loop time 
unsigned long last_loop_time = 0;
unsigned long prev_time = 0;

// ===== Motor Parameters =====
const float MAX_PWM = 255.0;
const float COUNTS_PER_REV = 960.0;  // N20 motor encoder counts per revolution (adjust for your motor)
 

// ===== LQR Gains =====
// K = [k1, k2, k3, k4]
const float K1 = 0;   // Position gain
const float K2 = -25.0;  // Angle gain
const float K3 = -1.2;   // Velocity gain
const float K4 = -4.5;   // Angular velocity gain

// state variables
float x1 = 0.0;  // Average wheel position (rad)
float x2 = 0.0;  // Body pitch angle (rad)
float x3 = 0.0;  // Average wheel velocity (rad/s)
float x4 = 0.0;  // Body pitch rate (rad/s)

void getimu()
{
  mpu.update();
  x2 = mpu.getAngleY()* PI / 180.0;      // Body pitch angle (check sign)// if this doesn't work the use 
  x4 = mpu.getGyroY()*PI/180.0;   // cheak the orientation  (mpu.getAngleY()- angleZeroY)* PI / 180.0; 

}


void updateEncoders() {
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;  // Convert to seconds
  
  if (dt >= DT) {
    // Read encoder counts (Encoder library handles everything automatically)
    long curr_encoder_right = encoder_right.read();
    long curr_encoder_left = encoder_left.read();
    
    // Calculate velocities (rad/s)
    float delta_right = (curr_encoder_right - prev_encoder_right)  * 2.0* PI / COUNTS_PER_REV;
    float delta_left = (curr_encoder_left - prev_encoder_left) * 2.0* PI/ COUNTS_PER_REV;
    
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

void stopMotors()
{
  setMotorSpeed(true, 0);
  setMotorSpeed(false, 0);
}

void computeLQRControl() {
  
  float U_balance = -(K1 * x1 + K2 * x2 + K3 * x3 + K4 * x4);
  
  // Convert control signal to PWM (scale appropriately)
  float pwm_scale = 8.0;  // Tune this accordingly
  float U_right = U_balance * pwm_scale; //(add forward and backward velocity here itself)
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




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.setAddress(0x68);
  mpu.begin(0, 0);
  //mpu.setFilterGyroCoef(0.95);(change if wanted)
  //mpu.setFilterAccCoef(0.02);
  mpu.calcOffsets();
 
  pinMode(buzz, OUTPUT); 
  pinMode(MOTOR_R_PWM , OUTPUT);   
  pinMode(MOTOR_L_PWM , OUTPUT);   
  pinMode(MOTOR_R_DIR1, OUTPUT);
  pinMode(MOTOR_R_DIR2, OUTPUT);
  pinMode(MOTOR_L_DIR1, OUTPUT);
  pinMode(MOTOR_L_DIR2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long current_time = micros();
  
  // Run control loop at fixed rate (100Hz)
  if (current_time - last_loop_time >= DT * 1000000) {
    last_loop_time = current_time;
  
  getimu();
  updateEncoders();
  computeLQRControl();
  }
  
}

