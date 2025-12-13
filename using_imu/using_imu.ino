#include <Wire.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);



void setup(){
  Serial.begin(9600);
  Wire.begin();
  mpu.setAddress(0x68);
  mpu.begin(0, 0);
  //mpu.setFilterGyroCoef(0.95);(change if wanted)
  //mpu.setFilterAccCoef(0.02);
  mpu.calcOffsets();
 
}
void loop(){
  mpu.update();
  float angle = mpu.getAngleY();
  float gyro = mpu.getGyroY();
  delay(100);
  Serial.print("angle along y is : ");
  Serial.println(angle);
  delay(100);
  Serial.print("velocity along y is :");
  Serial.println(gyro);
  

  return 0;
  
}


