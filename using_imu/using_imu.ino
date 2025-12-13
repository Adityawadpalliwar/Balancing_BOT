#include <Wire.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
unsigned long timer = 0;


void setup(){
  Serial.begin(9600);
  Wire.begin();
  mpu.setAddress(0x68);
  mpu.begin(0,0);
  //mpu.setFilterGyroCoef(0.95);(change if wanted)
  //mpu.setFilterAccCoef(0.02);
  mpu.calcOffsets();
 
}
void loop(){
  mpu.update();
  if((millis()-timer)>10){ // print data every 10ms
	float angle = mpu.getAngleY();
  float gyro = mpu.getGyroY();
  Serial.print("angle along y is : ");
  Serial.println(angle);
  Serial.print("velocity along y is :");
  Serial.println(gyro);
	timer = millis();  
  }

  

  return 0;
  
}


