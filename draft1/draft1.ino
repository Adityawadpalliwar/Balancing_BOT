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
  getimu()
  
}

void getimu()
{
  mpu.update();
  float angle[3] ={mpu.getAngleX() - angleZeroX,mpu.getAngleY()- angleZeroY,mpu.getAngleZ()- angleZeroZ};
  float gyro[3] = {mpu.getGyroX()-gyroOffsetX,mpu.getGyroY()-gyroOffsetY,mpu.getGyroZ()-gyroOffsetZ};
}
