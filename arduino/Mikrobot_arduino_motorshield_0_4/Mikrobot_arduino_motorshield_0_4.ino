//This sketch is designed to control the shield after receiving signal from the Processing sketch through serial communication
//Upload this sketch on Arduino first before opening the Processing sketch
//
// - 06/25/2015 -


#include <AFMotor.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

ros::NodeHandle  nh;


void motor_cb( const std_msgs::UInt16& cmd_msg){
  if (cmd_msg.data == 0) {
    motor1.run(RELEASE); //STOP
    motor2.run(RELEASE);
  } else if (cmd_msg.data == 1) { //FORWARD
   motor1.run(FORWARD); 
   motor2.run(FORWARD); 
  } else if (cmd_msg.data == 2) {
   motor1.run(BACKWARD); //RIGHT
   motor2.run(FORWARD);
  } else if (cmd_msg.data == 3) {
   motor1.run(FORWARD);  //LEFT
   motor2.run(BACKWARD);
  } else if (cmd_msg.data == 4) {
   motor1.run(BACKWARD); //BACKWARD
   motor2.run(BACKWARD);
  } else if (cmd_msg.data == 4) {
   motor1.run(BACKWARD); //BACKWARD
   motor2.run(BACKWARD);
  }
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  

}


ros::Subscriber<std_msgs::UInt16> sub("motor", motor_cb);


void setup()
{
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  motor1.setSpeed(255);
  motor1.run(RELEASE);
  motor2.setSpeed(200);
  motor2.run(RELEASE);
  
}
     
void loop()
{
  
  
  nh.spinOnce();
  delay(1);
//    String input = "";
//     
//    // Read any serial input
//    while (Serial.available() > 0)
//    {
//        input += (char) Serial.read(); // Read in one char at a time
//        delay(5); // Delay for 5 ms so the next char has time to be received
//    }
//     
//    if (input == "0")
//    {   
//        motor1.run(RELEASE);
//        motor2.run(RELEASE);
//      
//    }
//    else if (input == "1")
//    {
//       motor1.setSpeed(200); //I'm not sure if the motor is turning left or not
//       motor1.run(FORWARD); //If it is not, change this function to run(BACKWARD)
//    }
//
//    else if (input == "2")
//    {
//       motor1.setSpeed(200);
//       motor1.run(BACKWARD); // and change this function to .run(FORWARD)
//    }
//    
//    else if (input == "3")
//    {
//       motor2.setSpeed(200); //I'm not sure if the motor is turning left or not
//       motor2.run(FORWARD); //If it is not, change this function to run(BACKWARD)
//    }
//
//    else if (input == "4")
//    {
//       motor2.setSpeed(200); 
//       motor2.run(BACKWARD); // and change this function to .run(FORWARD)
//    }
}


