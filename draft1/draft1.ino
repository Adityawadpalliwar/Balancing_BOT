#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

#define encodPinAR 8
#define encodPinBR 7
#define encodPinAL 2
#define encodPinBL 3

//float angleZeroY;
//float gyroOffsetY;

const int buzz  = A1; 
unsigned long imuTimer= 0;
unsigned long printTimer  = 0; 

// Right Motor (Motor A)(right) 
// in your view gripper should be on left  your frame of reference 
#define MOTOR_R_PWM 6    
const int MOTOR_R_DIR1 = A2;    
const int MOTOR_R_DIR2 = A3;    

// Left Motor (Motor B)G%HY ZSV
#define MOTOR_L_PWM 5    
const int MOTOR_L_DIR1 =4;   
const int MOTOR_L_DIR2 =9;



// Previous values for differentiation
long prev_encoder_right = 0;
long prev_encoder_left = 0;

// for timmings
const float DT = 0.01f;  // 10ms loop time 
unsigned long last_loop_time = 0;
unsigned long prev_time = 0;

// ===== Motor Parameters =====
const float MAX_PWM = 255.0;
const float COUNTS_PER_REV = 700.0;  // N20 motor encoder counts per revolution (adjust for your motor)//correct


volatile long wheel_pulse_count_left = 0;
volatile long wheel_pulse_count_right = 0;
int prevA_right = LOW;
int prevA_left  = LOW;

// ===== LQR Gains =====
// K = [k1, k2, k3, k4]
const float K1 = -13.823;   // Position gain
const float K2 = -142.5969;  // Angle gain
const float K3 = -5.4460;   // Velocity gain
const float K4 = -28.880;   // Angular velocity gain

// state variables
float x1 = 0.0;  // Average wheel position (rad)
float x2 = 0.0;  // Body pitch angle (rad)
float x3 = 0.0;  // Average wheel velocity (rad/s)
float x4 = 0.0;  // Body pitch rate (rad/s)

void getimu()
{
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
	x2 = -mpu.getAngleY()* PI/180.0;      // Body pitch angle (check sign)// if this doesn't work the use 
  x4 = -mpu.getGyroY()*PI/180.0;   // cheak the orientation  (mpu.getAngleY()- angleZeroY)* PI / 180.0; 
	timer = millis();  
  }
}


void updateEncoders() {
  unsigned long current_time = millis();

  float dt = (current_time - prev_time) / 1000.0;  // Convert to seconds
  
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
 //static unsigned long lastPrint = 0;
  //if (millis() - lastPrint >= 20) { 
  //  lastPrint = millis();
   // Serial.print(wheel_pulse_count_left);
   // Serial.print(" ; ");
   // Serial.println(wheel_pulse_count_right);
  //}
  
  if (dt >= DT) {
    // Read encoder counts (Encoder library handles everything automatically)
    long curr_encoder_right = wheel_pulse_count_right;
    long curr_encoder_left = wheel_pulse_count_left;
    
    //Serial.print("the right encoder is at: ");
    //Serial.print(curr_encoder_right);
    //Serial.print(" , ");
    //Serial.print("the left encoder is at: ");
    //Serial.println(curr_encoder_left);
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
    //Serial.print(x3);
    //Serial.print(" , ");
    //Serial.print(x1);
  }
    
}
void setMotorSpeed(bool isRight, float speed) {
  // speed range: -255 to 255
  int pwm_pin = isRight ? MOTOR_R_PWM : MOTOR_L_PWM;
  int dir_pin1 = isRight ? MOTOR_R_DIR1 : MOTOR_L_DIR1;
  int dir_pin2 = isRight ? MOTOR_R_DIR2 : MOTOR_L_DIR2;
  
  // Constrain speed

  analogWrite(pwm_pin, (int)speed);
  // Set direction
  if (speed >= 0) {
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, HIGH);
  } else {
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
    speed = -speed;
  }
  
  
}

void stopMotors()
{
  setMotorSpeed(true, 0);
  setMotorSpeed(false, 0);
}

void computeLQRControl() {
  
  float U_balance = -(K1 * x1 + K2 * x2 + K3 * x3 + K4 * x4);

  int U_int = (int)U_balance;
  float U_new = map(constrain(U_int, -800, 800), -800, 800, -200, 200);
  
  // Convert control signal to PWM (scale appropriately)
  //float pwm_scale = 10.0;  // Tune this accordingly
  float U_right = (U_balance); //(cheack this)
  float U_left = (U_balance);
  
  // Safety check: if robot is too tilted, stop motors
  if (abs(x2) > 22) {  // 22degrees
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

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
 
  pinMode(buzz, OUTPUT); 
  pinMode(MOTOR_R_PWM , OUTPUT);   
  pinMode(MOTOR_L_PWM , OUTPUT);   
  pinMode(MOTOR_R_DIR1, OUTPUT);
  pinMode(MOTOR_R_DIR2, OUTPUT);
  pinMode(MOTOR_L_DIR1, OUTPUT);
  pinMode(MOTOR_L_DIR2, OUTPUT);
  
   prevA_right = digitalRead(encodPinAR);
   prevA_left  = digitalRead(encodPinAL);
   imuTimer   = millis();
   printTimer = millis();
   


}

void loop() {
  // put your main code here, to run repeatedly:

  
  unsigned long now = millis();

  // 1) Always service encoders
  updateEncoders();

  // 2) IMU update every 5 ms
  if (now - imuTimer >= 5) {     // 5 ms period → 200 Hz
    imuTimer = now;
    getimu();
  }
  /*if (now - printTimer >= 20) {
    printTimer = now;
    Serial.print("L: ");
    Serial.print(wheel_pulse_count_left);
    Serial.print("  R: ");
    Serial.println(wheel_pulse_count_right);
  }*/
  
  
}

