#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// MPU Initialization
Adafruit_MPU6050 mpu;

// Left encoder
#define encodPinAL 8  
#define encodPinBL 7  

// Right encoder
#define encodPinAR 2 
#define encodPinBR 3  

//Left motor
const int MotorL1 = A2;
const int MotorL2 = A3;
const int EN1 = 6;

//Right motor
const int MotorR1 = 4;
const int MotorR2 = 9;
const int EN2 = 5;

// Variables for complementary filter (Outer PID)
float accelPitch = 0.0;  // Pitch angle from accelerometer
float gyroPitch = 0.0;   // Pitch angle from gyroscope
float pitch = 0.0;       // Final pitch angle from complementary filter
float alpha = 0.95;      // Complementary filter constant
float pitchN=0;
float prevgyroX=0;
float prevgyroY=0;
float prevgyroZ=0;
float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;
float accelXF=0;
float accelYF=0;
float accelZF=0;
float gyroXF=0;
float gyroYF=0;
float gyroZF=0;
float alphaH=0.95;
float alphaL=0.95;

float moment_of_inertia = 0.08765;

volatile int wheel_pulse_count_left = 0;
volatile int wheel_pulse_count_right = 0;

volatile float left_RPM;
volatile float right_RPM;

volatile int left_d;
volatile int right_d;

volatile int left_prev_count;
volatile int right_prev_count;

volatile float velocityLeft;
volatile float velocityRight;

float velocity  = 0;
float reqVelocity = 0;

float old_distance = 0;
float distance_n = 0;
float distance = 0;
float reqDistance = 0;

float angle = 0;
float reqAngle =0;

float omega = 0;
float reqOmega = 0;

float errorV, errorD, errorA, errorO;

volatile float left_angularVelocity = 0;
volatile float right_angularVelocity = 0;

float U;
float U_new;

int leftOffset = 0;
int rightOffset = 0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float deltaTime;

float liner_left_v = 0;
float liner_right_v = 0;
float r = 0.0225;

void setup() 
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(1000);

  calibrateOffsets();
  
  Serial.println("Offsets calculated!");
  Serial.print("Accel X Offset: "); Serial.println(accelXOffset);
  Serial.print("Accel Y Offset: "); Serial.println(accelYOffset);
  Serial.print("Accel Z Offset: "); Serial.println(accelZOffset);
  Serial.print("Gyro X Offset: "); Serial.println(gyroXOffset);
  Serial.print("Gyro Y Offset: "); Serial.println(gyroYOffset);
  Serial.print("Gyro Z Offset: "); Serial.println(gyroZOffset);
  Serial.println();

  pinMode(MotorL1, OUTPUT);
  pinMode(MotorL2, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(encodPinAL, INPUT_PULLUP);
  digitalWrite(encodPinAL, HIGH);
  pinMode(encodPinBL, INPUT_PULLUP);
  digitalWrite(encodPinBL, HIGH);
  attachInterrupt(digitalPinToInterrupt(encodPinBL), motor_encoder_left, RISING);

  pinMode(MotorR1, OUTPUT);
  pinMode(MotorR2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(encodPinAR, INPUT_PULLUP);
  digitalWrite(encodPinAR, HIGH);
  pinMode(encodPinBR, INPUT_PULLUP);
  digitalWrite(encodPinBR, HIGH);
  attachInterrupt(digitalPinToInterrupt(encodPinBR), motor_encoder_right, RISING);
}



void motor_encoder_right() 
{
  if (digitalRead(encodPinBR) > digitalRead(encodPinAR)) 
  {
    wheel_pulse_count_right = wheel_pulse_count_right + 1;
  }
  else
  {
    wheel_pulse_count_right = wheel_pulse_count_right - 1;
  }
}

void motor_encoder_left() 
{
  if (digitalRead(encodPinBL) > digitalRead(encodPinAL)) 
  {
    wheel_pulse_count_left = wheel_pulse_count_left - 1;
  } 
  else 
  {
    wheel_pulse_count_left = wheel_pulse_count_left + 1;
  }
}


void calibrateOffsets() {
  sensors_event_t a, g, temp;

  // //Number of samples for calibration
  // const int numSamples = 5000;
  // float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

  // Serial.println("Calculating offsets...");
  // for (int i = 0; i < numSamples; i++) {
  //   mpu.getEvent(&a, &g, &temp);
  //   sumX += a.acceleration.x;
  //   sumY += a.acceleration.y;
  //   sumZ += a.acceleration.z;
  //   gyroXOffset += g.gyro.x;
  //   gyroYOffset += g.gyro.y;
  //   gyroZOffset += g.gyro.z;
  // }
  //   delay(5); // Short delay for sensor sampling
  //   gyroXOffset /= numSamples;
  //   gyroYOffset /= numSamples;
  //   gyroZOffset /= numSamples;
  //   accelXOffset = sumX / numSamples;
  //   accelYOffset = sumY / numSamples;
  //   accelZOffset = (sumZ / numSamples)-9.81; // Subtract 9.81 for gravity
  accelXOffset = 0.09;
  accelYOffset = -0.04;
  accelZOffset = -15.47; // Subtract 9.81 for gravity
  gyroXOffset = 0.37;
  gyroYOffset = 0.07;
  gyroZOffset = -0.01;
  // accelXOffset = 0;
  // accelYOffset = 0;
  // accelZOffset = 0; // Subtract 9.81 for gravity
  // gyroXOffset = 0;
  // gyroYOffset = 0;
  // gyroZOffset = 0;
  // for(int y=0; y<2; y++){
  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW 
  // delay(1000);        
  // }       // wait for a second
  // Serial.print("accelX:")
  // Serial.print()
}


void lqr(int leftOffset,int rightOffset)
{
  float kx=-3.4030,kv=-4.7857,ka=31.8192,ko=9.8235; 
  // float kx=-6.9207,kv=-9.7720,ka=46.2100,ko=-9.0454;                            
                 
  errorV = (reqVelocity - velocity);  
  errorD = (reqDistance - distance_n); 
  errorA = (reqAngle - pitchN);
  errorO = (reqOmega - omega);
  U = 2*(kv*errorV + kx*errorD + ka*errorA + ko*errorO);
  // U = (U * currentTime / (moment_of_inertia) / 10000);
  int U_int = (int)U;
  U_new = map(constrain(U_int, -800, 800),-800,800,-200,200);
  moveMotor(U_new - leftOffset,U_new - rightOffset);
  Serial.println(U_int);
 }

void moveMotor(int Left, int Right)
{
  Serial.print("Speed: ");
  // Serial.println(Left);
  // Serial.println(Right);
  analogWrite(EN1, abs(Left));
  analogWrite(EN2, abs(Right));

  if(Left>0 && Right>0)
  {
    digitalWrite(MotorL1, HIGH);
    digitalWrite(MotorL2, LOW);
    digitalWrite(MotorR1, HIGH);
    digitalWrite(MotorR2, LOW);
  }

  else if(Left<0 && Right<0)
  {
    digitalWrite(MotorL1, LOW);
    digitalWrite(MotorL2, HIGH);
    digitalWrite(MotorR1, LOW);
    digitalWrite(MotorR2, HIGH);
  }

  else if(Left==0 && Right==0)
  {
    digitalWrite(MotorL1, LOW);
    digitalWrite(MotorL2, LOW);
    digitalWrite(MotorR1, LOW);
    digitalWrite(MotorR2, LOW);
  }
}

void discal()
{
  volatile int current_countLeft = wheel_pulse_count_left;
  volatile int current_countRight = wheel_pulse_count_right;
  
  left_RPM = (float)((current_countLeft - left_prev_count)/(0.02*700));
  right_RPM = (float)((current_countRight - right_prev_count)/(0.02*700));

  // left_d = (float)((current_countLeft)/(1*700));
  // right_d = (float)((current_countRight)/(1*700));
  
  left_prev_count = current_countLeft;
  right_prev_count = current_countRight;

  left_angularVelocity = left_RPM * 2 * 3.14;
  right_angularVelocity = right_RPM * 2 * 3.14;

  liner_left_v = left_angularVelocity * r;
  liner_right_v = right_angularVelocity * r;
}

void botVelocity()
{
  velocity = (liner_right_v + liner_left_v)/2.0;  
}

void botDistance()
{
  distance = velocity *0.02;
  distance_n = distance_n + distance;
}

void loop() 
{

  discal();
  botVelocity();
  botDistance();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelX = a.acceleration.x - accelXOffset;
  float accelY = a.acceleration.y - accelYOffset;
  float accelZ = a.acceleration.z - accelZOffset;
  float gyroX = g.gyro.x - gyroXOffset;
  float gyroY = g.gyro.y - gyroYOffset;
  float gyroZ = g.gyro.z - gyroZOffset;

// Low Pass Filter
  accelXF=alphaL*accelXF + (1-alphaL)*accelX;
  accelYF=alphaL*accelYF + (1-alphaL)*accelY;
  accelZF=alphaL*accelZF + (1-alphaL)*accelZ;

// High Pass Filter
  gyroXF = alphaH * (gyroXF + (gyroX - prevgyroX));
  gyroYF = alphaH * (gyroYF + (gyroY - prevgyroY));
  gyroZF = alphaH * (gyroZF + (gyroZ - prevgyroZ));

  prevgyroX=gyroX;
  prevgyroY=gyroY;  
  prevgyroZ=gyroZ; 

  currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime; 

  accelPitch = atan2(accelXF, sqrt(accelYF * accelYF + accelZF * accelZF)) * 180 / 3.14;
  gyroPitch = gyroPitch + gyroYF * deltaTime; 

//Complementary Filter
  pitch = alpha * gyroPitch + (1 - alpha) * accelPitch;

  pitchN = (pitch * 41.66);
  omega = (gyroYF * 41.66);
  
  Serial.println(" Pitch angle: ");
  Serial.println(pitchN);
  // Serial.print("Distance:    ");
  // Serial.println(distance_n);
  // Serial.print("Velocity:    ");
  // Serial.println(velocity);
  // Serial.print("Thetadot:    ");
  // Serial.println(omega);
    Serial.print(" ErrorV: ");
    Serial.print(errorV);
    Serial.print(" ,ErrorD");
    Serial.print(errorD);
    Serial.print(" ,ErrorA");
    Serial.print(errorA);
    Serial.print(" ,ErrorO");
    Serial.println(errorO);

  leftOffset = velocity - liner_left_v;
  rightOffset = velocity - liner_right_v;

  // Serial.println(leftOffset);
  // Serial.println(rightOffset);

  lqr(leftOffset,rightOffset);
}