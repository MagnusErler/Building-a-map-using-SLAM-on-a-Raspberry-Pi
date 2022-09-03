
// ------ROS Serial------
#define USE_USBCON  //Used with Arduino Micro Pro
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::Float32 float32_msg;
std_msgs::String str_msg;

ros::Publisher pub_voltage("voltage", &float32_msg);
ros::Publisher pub_temperature("temperature", &float32_msg);
ros::Publisher pub_orientation("orientation", &str_msg);

// ------Voltmeter------
float voltage = 0.00;

// -------MPU6050-------
#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);
float temperature;
String orientation_string;

// -------Motor-------
int motor1pin1 = 4;
int motor1pin2 = 5;

int motor2pin1 = 6;
int motor2pin2 = 7;

int motor1_en = 9;
int motor2_en = 10;

void setup() {

  setupROSSerial();
  
  setupMPU6050();

  setupMotor();
}

void setMotor( const std_msgs::UInt16& cmd_msg){
  //Controlling speed (0 = off and 255 = max speed):
  int x = cmd_msg.data;
  analogWrite(motor1_en, x);
  analogWrite(motor2_en, x);
}

ros::Subscriber<std_msgs::UInt16> sub("CmdSetMotor", setMotor);

void loop() {

  getVoltage();
  getDataFromMPU6050();

  publishData();

  nh.spinOnce();
  
  delay(1000);
}

void setupROSSerial() {
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  
  nh.advertise(pub_voltage);
  nh.advertise(pub_temperature);
  nh.advertise(pub_orientation);

  nh.subscribe(sub);
}

void setupMPU6050() {
  Wire.begin();
  mpu6050.begin();

  //Serial.begin(9600);
  //mpu6050.calcGyroOffsets(true);
  mpu6050.setGyroOffsets(2.32, 0.22, 0.11);
}

void setupMotor() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(motor1_en, OUTPUT); 
  pinMode(motor2_en, OUTPUT);

  //Controlling spin direction of motors:
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void getVoltage() {
  voltage = (analogRead(A0) * 3.3) / 1024.00; // formula for calculating voltage out i.e. V+, here 3.30
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
