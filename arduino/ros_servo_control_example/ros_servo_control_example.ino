
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
int servoPin = 3; // digital pin connected to the servo

void servo_cb( const std_msgs::UInt16& cmd_msg){
  myservo.write(cmd_msg.data);
}

ros::Subscriber<std_msgs::UInt16> sub("servo_position", servo_cb);

void setup() 
{ 
  myservo.attach(servoPin);  // attaches the servo on servoPin to the servo object
  
  nh.initNode();
  nh.subscribe(sub);
} 
 
 
void loop() 
{ 
  nh.spinOnce();
  delay(1);
} 
