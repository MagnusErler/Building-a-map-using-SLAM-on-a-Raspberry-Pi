
// ------ROS Serial------
//#define USE_USBCON  //Used with Arduino Micro Pro
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::Float32 float32_msg;
std_msgs::String str_msg;

ros::Publisher pub_voltage("battery/voltage", &float32_msg);
ros::Publisher pub_temperature("IMU/temperature", &float32_msg);
ros::Publisher pub_orientation("IMU/orientation", &str_msg);

// ------Voltmeter------
float voltage = 0.00;

// -------MPU6050-------
#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);
float temperature;
String orientation_string;

// -------Motor-------
const int motor1pin1 = 4;
const int motor1pin2 = 5;

const int motor2pin1 = 6;
const int motor2pin2 = 7;

const int motor1_en = A2; //A0 not working??
const int motor2_en = A3;

// -------Timer-------
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

void setup() {

  //Serial.begin(9600);

  setupROSSerial();
  
  setupMPU6050();

  setupMotor();
}

void setSpeed(const std_msgs::Int16MultiArray& cmd_msg){

  //Controlling speed (0 = off and 255 = max speed):
  int speed1 = cmd_msg.data[0];
  int speed2 = cmd_msg.data[1];

  if (speed1 < 0 && speed2 > 0) {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
  } else if (speed1 > 0 && speed2 < 0) {
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
  } else if (speed1 < 0 && speed2 < 0) {
    //Backward
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
  } else {
    //Forward
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
  }

  if (speed1 < 0) {
    speed1 = speed1*(-1);
  }

  if (speed2 < 0) {
    speed2 = speed2*(-1);
  }
  
  analogWrite(motor1_en, speed1);
  analogWrite(motor2_en, speed2);
}

void setPubFreq( const std_msgs::UInt16& cmd_msg){
  interval = cmd_msg.data;
}

void caliMPU6050(const std_msgs::Bool& dummy) {
  //mpu6050.calcGyroOffsets(true);
  mpu6050.setGyroOffsets(2.32, 0.22, 0.11);
}

void setupMPU6050() {
  Wire.begin();
  mpu6050.begin();

  //caliMPU6050(true);  //not working
}

ros::Subscriber<std_msgs::UInt16> sub_pubFreq("CmdSetPubFreq", setPubFreq);
ros::Subscriber<std_msgs::Int16MultiArray> sub_speed("motor/CmdSetSpeed", setSpeed);
ros::Subscriber<std_msgs::Bool> sub_caliIMU("IMU/CmdCaliIMU", caliMPU6050);

void loop() {

  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
 
    getVoltage();
    getDataFromMPU6050();

    publishData();
  }

  nh.spinOnce();
}

void setupROSSerial() {

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  
  nh.advertise(pub_voltage);
  nh.advertise(pub_temperature);
  nh.advertise(pub_orientation);

  nh.subscribe(sub_speed);
  nh.subscribe(sub_pubFreq);
  nh.subscribe(sub_caliIMU);
}

void setupMotor() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(motor1_en, OUTPUT); 
  pinMode(motor2_en, OUTPUT);

  //Forward
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void getVoltage() {
  voltage = (analogRead(A0) * 5.0) / 1024.00; // formula for calculating voltage out i.e. V+, here 5.0
}

void getDataFromMPU6050() {
  mpu6050.update();
  
  orientation_string = String(mpu6050.getAngleX()) + ";" + String(mpu6050.getAngleY()) + ";" + String(mpu6050.getAngleZ());
             
  temperature = mpu6050.getTemp();
}

void publishData() {
  float32_msg.data = voltage;
  pub_voltage.publish( &float32_msg );
  
  float32_msg.data = temperature;
  pub_temperature.publish( &float32_msg );

  char Buf[50];
  orientation_string.toCharArray(Buf, 50);

  str_msg.data = Buf;
  pub_orientation.publish( &str_msg );
}
