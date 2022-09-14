
// ------ROS Serial------
//#define USE_USBCON  //Used with Arduino Micro Pro
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

void setPubFreq(const std_msgs::UInt16&);
void setSpeed(const std_msgs::Int16MultiArray&);
void caliMPU6050(const std_msgs::Empty&);

ros::NodeHandle nh;

std_msgs::Int16 int16_msg;
std_msgs::Float32 float32_msg;
std_msgs::String str_msg;

ros::Publisher pub_voltage("battery/voltage", &float32_msg);
ros::Publisher pub_temperature("IMU/temperature", &float32_msg);
ros::Publisher pub_orientation("IMU/orientation", &str_msg);
ros::Publisher pub_encoderTick_R("motor/encoderTick_R", &int16_msg);
ros::Publisher pub_encoderTick_L("motor/encoderTick_L", &int16_msg);

ros::Subscriber<std_msgs::UInt16> sub_pubFreq("CmdSetPubFreq", setPubFreq);
ros::Subscriber<std_msgs::Int16MultiArray> sub_speed("motor/CmdSetSpeed", setSpeed);
ros::Subscriber<std_msgs::Empty> sub_caliIMU("IMU/CmdCaliIMU", caliMPU6050);

// ------Voltmeter------
float voltage = 0.00;

// -------MPU6050-------
#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);
float temperature;
String orientation_string;

// -------Motor-------
/*const int ENC_COUNT_REV = 620; // Motor encoder output pulses per 360 degree revolution (measured manually)   https://automaticaddison.com/calculate-pulses-per-revolution-for-a-dc-motor-with-encoder/
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;*/

// MOTOR RIGHT
const int motorR_in1 = 4;
const int motorR_in2 = 5;
const int motorR_pwm = A2; //A0 not working??
const int motorR_encoderA = 2;
const int motorR_encoderB = 3;
  
volatile int pos_R = 0;

/*const float wheelRadius_R = 0.01;   // [m]

// Variable for RPM measuerment
float rpm_R = 0;
 
// Variable for angular velocity measurement
float angVelocity_R = 0;      // [rad/s]
float angVelocity_R_deg = 0;  // [deg/s]

// Variable for linear velocity measurement
float linVelocity_R = 0;      // [m/s]*/

// MOTOR LEFT
const int motorL_in1 = 8;
const int motorL_in2 = 9;
const int motorL_pwm = A3;
const int motorL_encoderA = 10;
const int motorL_encoderB = 11;
  
volatile int pos_L = 0;

/*const float wheelRadius_L = 0.01;   // [m]

// Variable for RPM measuerment
float rpm_L = 0;
 
// Variable for angular velocity measurement
float angVelocity_L = 0;      // [rad/s]
float angVelocity_L_deg = 0;  // [deg/s]

// Variable for linear velocity measurement
float linVelocity_L = 0;      // [m/s]

// PID
double Pk_R = 0.5;  
double Ik_R = 0;
double Dk_R = 0.01;
double Setpoint_R, Input_R, Output_R, Output_Ra;    // PID variables
//PID PID_R(&Input_R, &Output_R, &Setpoint_R, Pk_R, Ik_R , Dk_R, DIRECT);    // PID Setup

double Pk_L = 0.5;  
double Ik_L = 0;
double Dk_L = 0.01;
double Setpoint_L, Input_L, Output_L, Output_La;    // PID variables
//PID PID_L(&Input_L, &Output_L, &Setpoint_L, Pk_L, Ik_L , Dk_L, DIRECT);    // PID Setup
*/

// -------Timer-------
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

// -------Setup-------
void setup() {

  //Serial.begin(9600);

  setupROSSerial();
  
  setupMPU6050();

  setupMotor();
}

void loop() {

  //Input_R = pos_R
  //PID_R.Compute();

  //Input_L = pos_L
  //PID_L.Compute();

  
  currentMillis = millis();

  /*// Calculate angular and linear velocity every 1sec
  if (currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;

    rpm_R = (float)(pos_R * 60 / ENC_COUNT_REV);
    angVelocity_R = rpm_R * rpm_to_radians;
    angVelocity_R_deg = angVelocity_R * rad_to_deg;
    linVelocity_R = wheelRadius_R * angVelocity_R;

    rpm_L = (float)(pos_L * 60 / ENC_COUNT_REV);
    angVelocity_L = rpm_L * rpm_to_radians;
    angVelocity_L_deg = angVelocity_L * rad_to_deg;
    linVelocity_L = wheelRadius_L * angVelocity_L;
  }*/

  if (abs(currentMillis - previousMillis) > interval) {
    previousMillis = currentMillis;
 
    getVoltage();
    getDataFromMPU6050();
    
    publishData();
  }


  int16_msg.data = pos_R;
  pub_encoderTick_R.publish(&int16_msg);

  int16_msg.data = pos_L;
  pub_encoderTick_L.publish(&int16_msg);

  pos_R = 0;
  pos_L = 0;

  nh.spinOnce();
}

// -------IMU-------
void setupMPU6050() {
  Wire.begin();
  mpu6050.begin();

  caliIMU();
}

void caliMPU6050(const std_msgs::Empty&) {
  // caliMPU6050() is a callback function and can't be called from other functions.
  // For other functions to calibrate the IMU a new function has been made: caliIMU()
  caliIMU();
}

void caliIMU() {
  //mpu6050.calcGyroOffsets(true);
  mpu6050.setGyroOffsets(2.32, 0.22, 0.11);
}

void getDataFromMPU6050() {
  mpu6050.update();
  
  orientation_string = String(mpu6050.getAngleX()) + ";" + String(mpu6050.getAngleY()) + ";" + String(mpu6050.getAngleZ());
             
  temperature = mpu6050.getTemp();
}

// -------Motor-------
void setupMotor() {
  pinMode(motorR_in1, OUTPUT);
  pinMode(motorR_in2, OUTPUT);
  pinMode(motorL_in1, OUTPUT);
  pinMode(motorL_in2, OUTPUT);

  pinMode(motorR_pwm, OUTPUT); 
  pinMode(motorL_pwm, OUTPUT);

  digitalWrite(motorR_in1, LOW); digitalWrite(motorR_in2, LOW);
  digitalWrite(motorL_in1, LOW); digitalWrite(motorL_in2, LOW);
  
  attachInterrupt(digitalPinToInterrupt(motorR_encoderA), readEncoder_R, RISING);
  attachInterrupt(digitalPinToInterrupt(motorL_encoderA), readEncoder_L, RISING);
}

void setSpeed(const std_msgs::Int16MultiArray& cmd_msg){

  //MAX speed = 600rpm
  // 600 * rpm_to_radians * wheelRadius => 3.1415926536
  //MAX speed = 255
  // 255/3.1415926536 = 81.1690209766

  //Controlling speed (0 = off and 255 = max speed):
  int speed1 = cmd_msg.data[0]*81.1690209766;
  int speed2 = cmd_msg.data[1]*81.1690209766;

  if (speed1 < 0 && speed2 > 0) {
    digitalWrite(motorR_in1, LOW); digitalWrite(motorR_in2, HIGH);
    digitalWrite(motorL_in1, LOW); digitalWrite(motorL_in2, HIGH);
  } else if (speed1 > 0 && speed2 < 0) {
    digitalWrite(motorR_in1, HIGH); digitalWrite(motorR_in2, LOW);
    digitalWrite(motorL_in1, HIGH); digitalWrite(motorL_in2, LOW);
  } else if (speed1 < 0 && speed2 < 0) {
    //Backward
    digitalWrite(motorR_in1, LOW); digitalWrite(motorR_in2, HIGH);
    digitalWrite(motorL_in1, HIGH); digitalWrite(motorL_in2, LOW);
  } else {
    //Forward
    digitalWrite(motorR_in1, HIGH); digitalWrite(motorR_in2, LOW);
    digitalWrite(motorL_in1, LOW); digitalWrite(motorL_in2, HIGH);
  }

  if (speed1 < 0) {
    speed1 = speed1*(-1);
  }

  if (speed2 < 0) {
    speed2 = speed2*(-1);
  }
  
  analogWrite(motorR_pwm, speed1);
  analogWrite(motorL_pwm, speed2);
}

void readEncoder_R(){
  if (digitalRead(motorR_encoderA) > 0) {
    pos_R++;
  } else {
    pos_R--;
  }
}

void readEncoder_L(){
  if (digitalRead(motorL_encoderA) > 0) {
    pos_L++;
  } else {
    pos_L--;
  }
}

// -------Voltage-------
void getVoltage() {
  voltage = (analogRead(A0) * 5.0) / 1024.00;
}

// -------Ros-------
void setupROSSerial() {

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  
  nh.advertise(pub_voltage);
  nh.advertise(pub_temperature);
  nh.advertise(pub_orientation);
  nh.advertise(pub_encoderTick_R);
  nh.advertise(pub_encoderTick_L);

  nh.subscribe(sub_speed);
  nh.subscribe(sub_pubFreq);
  nh.subscribe(sub_caliIMU);
}

void setPubFreq(const std_msgs::UInt16& cmd_msg){
  interval = cmd_msg.data;
}

void publishData() {
  float32_msg.data = voltage;
  pub_voltage.publish(&float32_msg);
  
  float32_msg.data = temperature;
  pub_temperature.publish(&float32_msg);

  char Buf[50];
  orientation_string.toCharArray(Buf, 50);
  str_msg.data = Buf;
  pub_orientation.publish(&str_msg);
}

