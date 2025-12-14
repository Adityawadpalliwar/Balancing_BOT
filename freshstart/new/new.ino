#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// Encoder pins
#define encodPinAR 8
#define encodPinBR 7
#define encodPinAL 2
#define encodPinBL 3

volatile long wheel_pulse_count_left  = 0;
volatile long wheel_pulse_count_right = 0;

int prevA_right = LOW;
int prevA_left  = LOW;

// Timers
unsigned long imuTimer    = 0;  // for IMU update
unsigned long printTimer  = 0;  // for serial prints

void setup() {
  Serial.begin(9600);

  // Encoder pins
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);
  pinMode(encodPinAL, INPUT_PULLUP);
  pinMode(encodPinBL, INPUT_PULLUP);

  prevA_right = digitalRead(encodPinAR);
  prevA_left  = digitalRead(encodPinAL);

  // IMU init
  Wire.begin();
  mpu.setAddress(0x68);
  mpu.begin(0, 0);
  mpu.setFilterGyroCoef(0.95);
  mpu.setFilterAccCoef(0.05);
  mpu.calcOffsets();

  imuTimer   = millis();
  printTimer = millis();
}

void updateEncoders()
{
  // Right encoder
  int aR = digitalRead(encodPinAR);
  if (prevA_right == LOW && aR == HIGH) {
    if (digitalRead(encodPinBR) == HIGH) wheel_pulse_count_right++;
    else                                 wheel_pulse_count_right--;
  }
  prevA_right = aR;

  // Left encoder
  int aL = digitalRead(encodPinAL);
  if (prevA_left == LOW && aL == HIGH) {
    if (digitalRead(encodPinBL) == HIGH) wheel_pulse_count_left++;
    else                                 wheel_pulse_count_left--;
  }
  prevA_left = aL;
}

void updateIMU()
{
  // Call as often as you like; internal lib handles timing
  mpu.update();

  // Optional: compute angle/gyro (don’t print every time)
  float angle = -(mpu.getAngleY()) * PI / 180.0;
  float gyro  =  (mpu.getGyroY())  * PI / 180.0;

  // For control code, you would store angle & gyro in globals here
  // and NOT print inside this function in the final balancing code.
  //Serial.print("angle along y: ");
  //Serial.println(angle);
  //Serial.print("gyro y: ");
  //Serial.println(gyro);
}

void loop() {
  unsigned long now = millis();

  // 1) Always service encoders
  updateEncoders();

  // 2) IMU update every 5 ms
  if (now - imuTimer >= 5) {     // 5 ms period → 200 Hz
    imuTimer = now;
    updateIMU();
  }

  // 3) Debug print of encoder counts every 20 ms (optional)
  if (now - printTimer >= 20) {
    printTimer = now;
    Serial.print("L: ");
    Serial.print(wheel_pulse_count_left);
    Serial.print("  R: ");
    Serial.println(wheel_pulse_count_right);
  }

  // No delay() here → loop runs as fast as possible between events
}
