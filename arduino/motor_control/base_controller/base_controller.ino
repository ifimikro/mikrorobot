
//digitalWrite(13, HIGH-digitalRead(13));  //toggle led

#include <AFMotor.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

ros::NodeHandle  nh;

// Global variables to fiddle with motor speeds.

int motor1_speed;
int motor2_speed;
int speed_multiplier;


void motor_cb( const geometry_msgs::Twist& cmd_msg){

     motor1_speed = (cmd_msg.linear.x * speed_multiplier);
     motor2_speed = (cmd_msg.linear.x * speed_multiplier);

     motor1.setSpeed(motor1_speed);
     motor2.setSpeed(motor2_speed);
     
     if (true) {
       motor1.run(BACKWARD);
       motor2.run(FORWARD);
     } else {
       motor1.run(FORWARD);
       motor2.run(BACKWARD);
     }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor_cb);


void setup()
{
    pinMode(13, OUTPUT);

    nh.initNode();
    nh.subscribe(sub);

    motor1.setSpeed(100);
    motor1.run(RELEASE);
    motor2.setSpeed(100);
    motor2.run(RELEASE);

// init state variables
    motor1_speed = 100;
    motor2_speed = 100;
    speed_multiplier = 80;
}

void loop()
{
    nh.spinOnce();
    delay(1);
}
