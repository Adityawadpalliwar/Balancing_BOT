#include <Wire.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
void getimu();
float angleZeroX;
float angleZeroY;
float angleZeroZ;
float gyroOffsetX;
float gyroOffsetY;
float gyroOffsetZ;


// Create Encoder objects (automatically handles interrupts)
Encoder encoder_right(2, 10);  // Right encoder pins A, B
Encoder encoder_left(3, 11);   // Left encoder pins A, B

// for timmings
const float DT = 0.01;  // 10ms loop time 
unsigned long last_loop_time = 0;


// ===== LQR Gains =====
// K = [k1, k2, k3, k4]
const float K1 = 0;   // Position gain
const float K2 = 0;  // Angle gain
const float K3 = 0;   // Velocity gain
const float K4 = 0;   // Angular velocity gain

// state variables
float x1 = 0.0;  // Average wheel position (rad)
float x2 = 0.0;  // Body pitch angle (rad)
float x3 = 0.0;  // Average wheel velocity (rad/s)
float x4 = 0.0;  // Body pitch rate (rad/s)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.setAddress(0x68);
  mpu.begin(0, 0);
  //mpu.setFilterGyroCoef(0.95);(change if wanted)
  //mpu.setFilterAccCoef(0.02);
  mpu.calcOffsets();
  angleZeroX = mpu.getAngleX();
  angleZeroY = mpu.getAngleY();
  angleZeroZ = mpu.getAngleZ();
  gyroOffsetX = mpu.getGyroX();
  gyroOffsetY = mpu.getGyroY();
  gyroOffsetZ = mpu.getGyroZ();
}

void loop() {
  // put your main code here, to run repeatedly:
  getimu();
  updateEncoders();

  
}

void getimu()
{
  mpu.update();
  float angle[3] ={mpu.getAngleX() - angleZeroX,mpu.getAngleY()- angleZeroY,mpu.getAngleZ()- angleZeroZ};
  float gyro[3] = {mpu.getGyroX()-gyroOffsetX,mpu.getGyroY()-gyroOffsetY,mpu.getGyroZ()-gyroOffsetZ};
  x2 = angle[0];      // Body pitch angle (negative to match convention)
  x4 =  gyro[0];   // cheak the orientation

}

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
