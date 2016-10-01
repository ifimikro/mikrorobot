#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>


ros::NodeHandle_<ArduinoHardware, 1, 1, 1024, 1024>  nh;


std_msgs::Float32MultiArray msg;

ros::Publisher pub("motor_speeds", &msg);

void setup()
{
  msg.data_length = 4;
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
  
  msg.data = data;
  
  pub.publish(&msg);
  delay(10);
}



