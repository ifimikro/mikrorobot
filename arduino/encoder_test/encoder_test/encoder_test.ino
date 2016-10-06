/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <ros.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

#define PIN_INPUT 0
#define FRONT_RIGHT_OUTPUT 2
#define FRONT_LEFT_OUTPUT 3
#define BACK_RIGHT_OUTPUT 4
#define BACK_LEFT_OUTPUT 5


ros::NodeHandle_<ArduinoHardware, 2, 2, 1000, 1000> nh;

//Define Variables we'll be connecting to
double Input[4], Output[4], Setpoint[4];

//Specify the links and initial tuning parameters
double Kp=2.15, Ki=0.1, Kd=0;
PID frontRightPID(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT);
PID frontLeftPID(&Input[1],&Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT);
PID backRightPID(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT);
PID backLeftPID(&Input[3],&Output[3], &Setpoint[3], Kp, Ki, Kd, DIRECT);

float data[4];

std_msgs::Float32MultiArray msg;

ros::Publisher pub("pid_outputs", &msg);

//Callback function on new setpoint from ROS
void motor_cb( const std_msgs::Float32MultiArray& motor_cmds){
  Setpoint[0] = motor_cmds.data[0];
  Setpoint[1] = motor_cmds.data[1];
  Setpoint[2] = motor_cmds.data[2];
  Setpoint[3] = motor_cmds.data[3];
}
//Callback function on new motor speeds from ROS
void motor_speed( const sensor_msgs::JointState& motor_speeds){
  Input[0] = motor_speeds.velocity[0];
  Input[1] = motor_speeds.velocity[1];
  Input[2] = motor_speeds.velocity[2];
  Input[3] = motor_speeds.velocity[3];
}

ros::Subscriber<std_msgs::Float32MultiArray> commands_sub("motor_cmds", motor_cb);
ros::Subscriber<sensor_msgs::JointState> speeds_sub("motor_speeds", motor_speed);

void setup()
{
  //msg.layout.data_offset = 4;
  msg.data_length = 4;
  //initialize the variables we're linked to
  for(int i = 0; i < 4; i++) {
    Input[i] = 0;
    Output[i] = 0;
    Setpoint[i] = 0;
    data[i] = 0;
  }

  pinMode(2, OUTPUT); 
  pinMode(3, OUTPUT); 
  pinMode(4, OUTPUT); 
  pinMode(5, OUTPUT); 

  //turn the PID on
  frontRightPID.SetMode(AUTOMATIC);
  frontLeftPID.SetMode(AUTOMATIC);
  backRightPID.SetMode(AUTOMATIC);
  backLeftPID.SetMode(AUTOMATIC);

  frontRightPID.SetSampleTime(1);
  frontLeftPID.SetSampleTime(1);
  backRightPID.SetSampleTime(1);
  backLeftPID.SetSampleTime(1);


  frontRightPID.SetOutputLimits(-120,120);
  frontLeftPID.SetOutputLimits(-120,120);
  backRightPID.SetOutputLimits(-120,120);
  backLeftPID.SetOutputLimits(-120,120);

  //Initialize ROS subscription
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(commands_sub);
  nh.subscribe(speeds_sub);
}

void loop()
{

  //Update setpoint and speeds
  nh.spinOnce();
  
  //Compute outputs
  frontRightPID.Compute();
  frontLeftPID.Compute();
  backRightPID.Compute();
  backLeftPID.Compute();

  //analogWrite(3,Output[1]);

  for(int i = 0; i < 4; i++) {
    data[i] = 2 * Setpoint[i] + Output[i];
    analogWrite(i+2, data[i]);
    digitalWrite(i+6,HIGH);
  }

  msg.data = data;

  pub.publish(&msg);
  Serial.flush();
}




