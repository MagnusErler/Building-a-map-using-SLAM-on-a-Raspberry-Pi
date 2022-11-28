
// ------ROS Serial------
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

void calibrateMPU6050(const std_msgs::Empty&);
void setVelocity(const std_msgs::Int16MultiArray&);

ros::NodeHandle nh;

std_msgs::Int16MultiArray int16MultiArray;
std_msgs::Float32 float32_msg;
std_msgs::Float32MultiArray float32MultiArray;

ros::Publisher pub_voltage("/battery/voltage", &float32MultiArray);
ros::Publisher pub_temperature("/IMU/temperature", &float32_msg);
ros::Publisher pub_orientation("/IMU/orientation", &int16MultiArray);
ros::Publisher pub_encoderTicks("/motor/encoderTicks", &int16MultiArray);

ros::Subscriber<std_msgs::Int16MultiArray> sub_velocity("/motor/CmdSetVelocityPWM", setVelocity);
ros::Subscriber<std_msgs::Empty> sub_caliIMU("/IMU/CmdCaliIMU", calibrateMPU6050);

// ------Voltmeter------
const int voltageRP_pin = A0;
const int voltageMotor_pin = A2;
float voltageRP = 0.00;
float voltageRP_previous = 0.00;
float voltageMotor = 0.00;
float voltageMotor_previous = 0.00;

// -------MPU6050-------
#include <MPU6050_tockn.h>
//#include <Wire.h>     MPU6050_tockn.h is already using Wire.h
MPU6050 mpu6050(Wire);
float temperature = 0.00;
float temperature_previous = 0.00;
int roll, pitch, yaw;

// -------Motor-------
// MOTOR LEFT
const int motorL_in1 = 10;
const int motorL_in2 = 9;
const int motorL_pwm_pin = 8;
const int motorL_encoderA = 18;
const int motorL_encoderB = 19;
  
volatile int pos_L = 0;
volatile int pos_L_previous = 0;

// MOTOR RIGHT
const int motorR_in1 = 11;
const int motorR_in2 = 12;
const int motorR_pwm_pin = 13;
const int motorR_encoderA = 3;
const int motorR_encoderB = 2;
  
volatile int pos_R = 0;
volatile int pos_R_previous = 0;

bool differentFromLastValue = false;

// -------Timer-------
long currentMillis = 0;
long previousMillis1 = 0;
long previousMillis2 = 0;
long previousMillis3 = 0;

// -------Setup-------
void setup() {

  setupROSSerial();
  
  setupMPU6050();

  setupMotor();
}

void loop() {
  
  currentMillis = millis();

  // Publish voltage and temperature every 2sec
  if (abs(currentMillis - previousMillis1) > 2000) {
    previousMillis1 = currentMillis;

    getVoltage();
    // temperature is already calculated in below if-statement when calling getDataFromMPU6050()
    
    publishData_temperature();
    publishData_voltage();

    voltageRP_previous = voltageRP;
    voltageMotor_previous = voltageMotor;
  }

  // Publish orientation every 0.25sec
  if (abs(currentMillis - previousMillis2) > 250) {
    previousMillis2 = currentMillis;
 
    getDataFromMPU6050();

    publishData_orientation();
  }

  // Publish encoder Ticks every 0.1sec
  if (abs(currentMillis - previousMillis3) > 100) {
    previousMillis3 = currentMillis;

    publishData_encoderTicks();

    pos_L = 0;
    pos_R = 0;

    pos_L_previous = pos_L;
    pos_R_previous = pos_R;
  }

  nh.spinOnce();
}

// -------IMU-------
void setupMPU6050() {
  Wire.begin();
  mpu6050.begin();

  mpu6050.calcGyroOffsets();
}

void calibrateMPU6050(const std_msgs::Empty&) {
  mpu6050.calcGyroOffsets();
}

void getDataFromMPU6050() {
  mpu6050.update();
  
  temperature = mpu6050.getTemp();  // [°C]

  pitch = mpu6050.getAngleX();      // [deg]
  roll = mpu6050.getAngleY();       // [deg]
  yaw = mpu6050.getAngleZ();        // [deg]
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
    //Turning left
    digitalWrite(motorL_in1, LOW); digitalWrite(motorL_in2, HIGH);
    digitalWrite(motorR_in1, LOW); digitalWrite(motorR_in2, HIGH);
  } else if (velocity_L > 0 && velocity_R < 0) {
    //Turning right
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
  
  analogWrite(motorL_pwm_pin, abs(velocity_L));
  analogWrite(motorR_pwm_pin, abs(velocity_R));
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
  voltageRP = (analogRead(A0) * 11.1) / 1024.00;
  voltageMotor = (analogRead(A2) * 11.1) / 1024.00;
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

void publishData_temperature() {
  if (temperature_previous != temperature) {
    float32_msg.data = temperature;
    pub_temperature.publish(&float32_msg);
  }
}

void publishData_voltage() {
  if (voltageRP_previous != voltageRP || voltageMotor_previous != voltageMotor) {
    float value_voltage[2] = {voltageRP, voltageMotor};
    float32MultiArray.data = value_voltage;
    float32MultiArray.data_length = 2;
    pub_voltage.publish(&float32MultiArray);
  }
}

void publishData_orientation() {
  int value[3] = {pitch, roll, yaw};
  int16MultiArray.data = value;
  int16MultiArray.data_length = 3;
  pub_orientation.publish(&int16MultiArray);
}

void publishData_encoderTicks() {
  if (pos_L_previous != pos_L || pos_R_previous != pos_R) {
    int value[2] = {pos_L, pos_R};
    int16MultiArray.data = value;
    int16MultiArray.data_length = 2;
    pub_encoderTicks.publish(&int16MultiArray);

    differentFromLastValue = true;
  } else {
    if (differentFromLastValue) {
      //Setting the encoder ticks to 0
      int value[2] = {0, 0};
      int16MultiArray.data = value;
      int16MultiArray.data_length = 2;
      pub_encoderTicks.publish(&int16MultiArray);

      differentFromLastValue = false;
    }
  }
}
