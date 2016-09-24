/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/UInt32MultiArray.h>


#define PIN_INPUT 0
#define FRONT_RIGHT_OUTPUT 2
#define FRONT_LEFT_OUTPUT 3
#define BACK_RIGHT_OUTPUT 4
#define BACK_LEFT_OUTPUT 5

ros::NodeHandle  nh;

//Define Variables we'll be connecting to
double Input[4], Output[4], Setpoint[4];

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID frontRightPID(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT);
PID frontLeftPID(&Input[1],&Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT);
PID backRightPID(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT);
PID backLeftPID(&Input[3],&Output[3], &Setpoint[3], Kp, Ki, Kd, DIRECT);


//Callback function on new setpoint from ROS
void motor_cb( const std_msgs::UInt32MultiArray& motor_cmds){
  Setpoint[0] = motor_cmds.data[0];
  Setpoint[1] = motor_cmds.data[1];
  Setpoint[2] = motor_cmds.data[2];
  Setpoint[3] = motor_cmds.data[3];
}

ros::Subscriber<std_msgs::UInt32MultiArray> sub("motor_cmds", motor_cb);


void setup()
{
  //initialize the variables we're linked to
  Input[0] = 0;
  Input[1] = 0;
  Input[2] = 0;
  Input[3] = 0;
  
  Setpoint[0] = 0;
  Setpoint[1] = 0;
  Setpoint[2] = 0;
  Setpoint[3] = 0;

  //turn the PID on
  frontRightPID.SetMode(AUTOMATIC);
  frontLeftPID.SetMode(AUTOMATIC);
  backRightPID.SetMode(AUTOMATIC);
  backLeftPID.SetMode(AUTOMATIC);
  
  //Initialize ROS subscription
  nh.initNode();
  nh.subscribe(sub);
  
  // start serial port at 9600 bps:
  Serial1.begin(9600);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial2.begin(9600);
  while (!Serial2) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}

void loop()
{
  //Update setpoint
  nh.spinOnce();
  
  //Get current speed
  if (Serial1.available() > 0) {
    // get incoming byte:
    Input[0] = Serial1.read();
    Input[2] = Serial1.read();
  }
  if (Serial2.available() > 0) {
    // get incoming byte:
    Input[1] = Serial2.read();
    Input[3] = Serial2.read();
  }
  
  //Compute outputs
  frontRightPID.Compute();
  frontLeftPID.Compute();
  backRightPID.Compute();
  backLeftPID.Compute();
  
  //Write to motor controllers
  analogWrite(FRONT_RIGHT_OUTPUT, Output[0]);
  analogWrite(FRONT_LEFT_OUTPUT, Output[1]);
  analogWrite(BACK_RIGHT_OUTPUT, Output[2]);
  analogWrite(BACK_LEFT_OUTPUT, Output[3]);
}


