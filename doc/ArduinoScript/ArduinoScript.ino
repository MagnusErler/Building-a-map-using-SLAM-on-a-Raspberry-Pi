
// ------ROS Serial------
//#define USE_USBCON  //Used with Arduino Micro Pro
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

void setVelocity(const std_msgs::Int16MultiArray&);
void calibrateMPU9250(const std_msgs::Empty&);

ros::NodeHandle nh;

std_msgs::Int16MultiArray int16MultiArray;
std_msgs::Float32 float32_msg;
std_msgs::String str_msg;

ros::Publisher pub_voltage("/battery/voltage", &float32_msg);
ros::Publisher pub_temperature("/IMU/temperature", &float32_msg);
ros::Publisher pub_orientation("/IMU/orientation", &int16MultiArray);
ros::Publisher pub_encoderTicks("/motor/encoderTicks", &int16MultiArray);

ros::Subscriber<std_msgs::Int16MultiArray> sub_velocity("/motor/CmdSetVelocityPWM", setVelocity);
ros::Subscriber<std_msgs::Empty> sub_caliIMU("/IMU/CmdCaliIMU", calibrateMPU9250);

// ------Voltmeter------
const int voltage_in = A0
float voltage = 0.00;

// -------MPU9250-------
#include <MPU9250.h>
MPU9250 mpu9250;
float temperature;
int roll, pitch, yaw;

// -------Motor-------
// MOTOR RIGHT
const int motorR_in1 = 2;
const int motorR_in2 = 3;
const int motorR_pwm_pin = 4;
const int motorR_encoderA = 5;
const int motorR_encoderB = 6;
  
volatile int pos_R = 0;

// MOTOR LEFT
const int motorL_in1 = 7;
const int motorL_in2 = 8;
const int motorL_pwm_pin = 9;
const int motorL_encoderA = 10; //19 on Mega
const int motorL_encoderB = 11; //18 on Mega
  
volatile int pos_L = 0;

// -------Timer-------
long currentMillis = 0;
long previousMillis1 = 0;
long previousMillis2 = 0;

// -------Setup-------
void setup() {

  //Serial.begin(9600);

  setupROSSerial();
  
  setupMPU9250();

  setupMotor();
}

void loop() {
  
  currentMillis = millis();

  // Publish encoder Ticks every 0.5sec
  if (abs(currentMillis - previousMillis1) > 500) {
    previousMillis1 = currentMillis;
 
    getVoltage();
    getDataFromMPU9250();
    
    publishData();
  }

  // Publish encoder Ticks every 0.1sec
  if (abs(currentMillis - previousMillis2) > 100) {
    previousMillis2 = currentMillis;
 
    int16_t value[2] = {pos_L, pos_R};
    int16MultiArray.data = value;
    int16MultiArray.data_length = 2;
    pub_encoderTicks.publish(&int16MultiArray);

    pos_L = 0;
    pos_R = 0;
  }

  nh.spinOnce();
}

// -------IMU-------
void setupMPU9250() {
  Wire.begin();
  mpu9250.setup(0x68);

  mpu9250.calibrateAccelGyro();
  mpu9250.calibrateMag();
}

void calibrateMPU9250(const std_msgs::Empty&) {
  mpu9250.calibrateAccelGyro();
  mpu9250.calibrateMag();
}

void getDataFromMPU9250() {
  mpu9250.update();
  
  temperature = mpu9250.getTemperature(); // [°C]

  pitch = mpu9250.getEulerX();            // [deg]
  roll = mpu9250.getEulerY();             // [deg]
  yaw = mpu9250.getEulerZ();              // [deg]
}

// -------Motor-------
void setupMotor() {
  pinMode(motorL_in1, OUTPUT); pinMode(motorL_in2, OUTPUT);
  pinMode(motorR_in1, OUTPUT); pinMode(motorR_in2, OUTPUT);

  pinMode(motorR_pwm_pin, OUTPUT); 
  pinMode(motorL_pwm_pin, OUTPUT);

  digitalWrite(motorL_in1, LOW); digitalWrite(motorL_in2, LOW);
  digitalWrite(motorR_in1, LOW); digitalWrite(motorR_in2, LOW);
  
  attachInterrupt(digitalPinToInterrupt(motorL_encoderA), readEncoderA_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorL_encoderB), readEncoderB_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorR_encoderA), readEncoderA_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorR_encoderB), readEncoderB_R, CHANGE);
}

void setVelocity(const std_msgs::Int16MultiArray& cmd_msg){
  //Controlling speed (0 = off and 255 = max speed):
  int velocity_L = cmd_msg.data[0];
  int velocity_R = cmd_msg.data[1];

  if (velocity_L < 0 && velocity_R > 0) {
    digitalWrite(motorL_in1, LOW); digitalWrite(motorL_in2, HIGH);
    digitalWrite(motorR_in1, LOW); digitalWrite(motorR_in2, HIGH);
  } else if (velocity_L > 0 && velocity_R < 0) {
    digitalWrite(motorL_in1, HIGH); digitalWrite(motorL_in2, LOW);
    digitalWrite(motorR_in1, HIGH); digitalWrite(motorR_in2, LOW);
  } else if (velocity_L < 0 && velocity_R < 0) {
    //Backward
    digitalWrite(motorL_in1, HIGH); digitalWrite(motorL_in2, LOW);
    digitalWrite(motorR_in1, LOW); digitalWrite(motorR_in2, HIGH);
  } else if (velocity_L > 0 && velocity_R > 0) {
    //Forward
    digitalWrite(motorL_in1, LOW); digitalWrite(motorL_in2, HIGH);
    digitalWrite(motorR_in1, HIGH); digitalWrite(motorR_in2, LOW);
  } 

  if (velocity_L < 0) {
    velocity_L = velocity_L*(-1);
  }

  if (velocity_R < 0) {
    velocity_R = velocity_R*(-1);
  }
  
  analogWrite(motorL_pwm_pin, velocity_L);
  analogWrite(motorR_pwm_pin, velocity_R);
}

void readEncoderA_L(){  
  if (digitalRead(motorL_encoderA) == HIGH) { 
    if (digitalRead(motorL_encoderB) == LOW) {  
      pos_L++;         // CW
    } else {
      pos_L--;         // CCW
    }
  } else { 
    if (digitalRead(motorL_encoderB) == HIGH) {   
      pos_L++;          // CW
    } else {
      pos_L--;          // CCW
    }
  }
}

void readEncoderB_L(){  
  if (digitalRead(motorL_encoderB) == HIGH) {   
    if (digitalRead(motorL_encoderA) == HIGH) {  
      pos_L ++;         // CW
    } else {
      pos_L--;         // CCW
    }
  } else { 
    if (digitalRead(motorL_encoderA) == LOW) {   
      pos_L++;          // CW
    } else {
      pos_L--;          // CCW
    }
  }
}

void readEncoderA_R(){  
  if (digitalRead(motorR_encoderA) == HIGH) { 
    if (digitalRead(motorR_encoderB) == LOW) {  
      pos_R--;         // CW
    } else {
      pos_R++;         // CCW
    }
  } else { 
    if (digitalRead(motorR_encoderB) == HIGH) {   
      pos_R--;          // CW
    } else {
      pos_R++;          // CCW
    }
  }
 
}

void readEncoderB_R(){  
  if (digitalRead(motorR_encoderB) == HIGH) {   
    if (digitalRead(motorR_encoderA) == HIGH) {  
      pos_R--;         // CW
    } else {
      pos_R++;         // CCW
    }
  } else { 
    if (digitalRead(motorR_encoderA) == LOW) {   
      pos_R--;          // CW
    } else {
      pos_R++;          // CCW
    }
  }
}

// -------Voltage-------
void getVoltage() {
  voltage = (analogRead(voltage_in) * 5.0) / 1024.00;
}

// -------Ros-------
void setupROSSerial() {

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  
  nh.advertise(pub_voltage);
  nh.advertise(pub_temperature);
  nh.advertise(pub_orientation);
  nh.advertise(pub_encoderTicks);

  nh.subscribe(sub_velocity);
  nh.subscribe(sub_caliIMU);
}

void publishData() {
  float32_msg.data = voltage;
  pub_voltage.publish(&float32_msg);
  
  float32_msg.data = temperature;
  pub_temperature.publish(&float32_msg);

  int16_t value[3] = {pitch, roll, yaw};
  int16MultiArray.data = value;
  int16MultiArray.data_length = 3;
  pub_orientation.publish(&int16MultiArray);
}
