#include <Wire.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
float angleZeroX;
float angleZeroY;
float angleZeroZ;
float gyroOffsetX;
float gyroOffsetY;
float gyroOffsetZ;


void setup(){
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
void loop(){
  mpu.update();
  float angle[3] ={mpu.getAngleX() - angleZeroX,mpu.getAngleY()- angleZeroY,mpu.getAngleZ()- angleZeroZ};
  float gyro[3] = {mpu.getGyroX()-gyroOffsetX,mpu.getGyroY()-gyroOffsetY,mpu.getGyroZ()-gyroOffsetZ};
  Serial.println("angle along x is :");
  Serial.print(angle[0]);
  Serial.println("angle along y is :");
  Serial.print(angle[1]);
  Serial.println("angle along z is :");
  Serial.print(angle[2]);
  Serial.println("velocity along x is :");
  Serial.print(gyro[0]);
  Serial.println("velocity along y is :");
  Serial.print(gyro[1]);
  Serial.println("velocity along z is :");
  Serial.print(gyro[2]);

  return 0;
  
}


