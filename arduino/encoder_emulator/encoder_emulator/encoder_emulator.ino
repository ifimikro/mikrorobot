#include <ros.h>
#include <sensor_msgs/JointState.h>


ros::NodeHandle nh;


sensor_msgs::JointState msg;

ros::Publisher pub("motor_speeds", &msg);

void setup()
{
  msg.velocity_length = 4;
  
  //Initialize ROS subscription
  nh.initNode();
  nh.advertise(pub);
}

void loop()
{
  nh.spinOnce();
  
  float data[4];

  for (int i = 0; i < 4; i++) {
    data[i] = (float)analogRead(i);
  }
  
  msg.velocity = data;
  
  pub.publish(&msg);
  delay(10);
}



