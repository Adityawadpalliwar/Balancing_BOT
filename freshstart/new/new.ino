#include <Wire.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
unsigned long timer = 0;
#define encodPinAR 2   
#define encodPinBR 3   
#define encodPinAL 8  
#define encodPinBL 7  

volatile long wheel_pulse_count_left = 0;
volatile long wheel_pulse_count_right = 0;

int prevA_right = LOW;
int prevA_left  = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);
  pinMode(encodPinAL, INPUT_PULLUP);
  pinMode(encodPinBL, INPUT_PULLUP);
  prevA_right = digitalRead(encodPinAR);
  prevA_left  = digitalRead(encodPinAL);
  Serial.begin(9600);
  Wire.begin();
  mpu.setAddress(0x68);
  mpu.begin(0,0);
  //mpu.setFilterGyroCoef(0.95);(change if wanted)
  //mpu.setFilterAccCoef(0.02);
  mpu.calcOffsets();
}

void get_encoder()
{
  int aR = digitalRead(encodPinAR);
  if (prevA_right == LOW && aR == HIGH) {     
    if (digitalRead(encodPinBR) == HIGH) wheel_pulse_count_right++;
    else wheel_pulse_count_right--;
  }
  prevA_right = aR;
 int aL = digitalRead(encodPinAL);
  if (prevA_left == LOW && aL == HIGH) {     
    if (digitalRead(encodPinBL) == HIGH) wheel_pulse_count_left++;
    else wheel_pulse_count_left--;
  }
  prevA_left = aL;
 static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 20) { 
    lastPrint = millis();
    Serial.print(wheel_pulse_count_left);
    Serial.print(" ; ");
    Serial.println(wheel_pulse_count_right);
  }
}

void get_imu()
{
  mpu.update();
  if((millis()-timer)>10)
  { // print data every 10ms
	float angle = (mpu.getAngleY())*PI/180.0;
  float gyro = mpu.getGyroY()*PI/180.0;
  Serial.print("angle along y is : ");
  Serial.println(angle);
  Serial.print("velocity along y is :");
  Serial.println(gyro);
	timer = millis();  
  }
}
void loop() {
  
 get_encoder();
 get_imu();

}