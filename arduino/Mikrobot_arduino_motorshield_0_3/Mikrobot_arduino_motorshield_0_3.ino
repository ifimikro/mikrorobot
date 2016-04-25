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

// Global variables to fiddle with motor speeds.
int speed_accelerate = 15; // How much to increment speed with.
int speed_standard = 50; // Standard starting speed when changing from L/R/F/B.

int prev_state; 
int motor1_speed;
int motor2_speed;
// State 0 = STOP
// State 1 = FORWARD
// State 2 = LEFT
// State 3 = RIGHT
// State 4 = BACKWARD


void motor_cb( const std_msgs::UInt16& cmd_msg){
  if (cmd_msg.data == 0) { //STOP
    motor1.run(RELEASE); 
    motor2.run(RELEASE);
    prev_state = 0;
    // END OF STOP

  } else if (cmd_msg.data == 1) { //FORWARD
    if (prev_state == 1) {
      // Increase motor speed.
      if (motor1_speed <= (255-speed_accelerate)) {
        motor1_speed += speed_accelerate;
        motor1.setSpeed(motor1_speed);
      }
      if (motor2_speed <= (255-speed_accelerate)) {
        motor2_speed += speed_accelerate;
        motor2.setSpeed(motor2_speed);
      }

    } else {
      // Start going FORWARD at pre-defined speed.
      prev_state = 1;

      motor1_speed = speed_standard;
      motor2_speed = speed_standard;

      motor1.setSpeed(motor1_speed);
      motor2.setSpeed(motor2_speed);

      motor1.run(FORWARD); 
      motor2.run(FORWARD); 
    } 
    // END OF FORWARD

  } else if (cmd_msg.data == 2) { //RIGHT
    if (prev_state == 2) {
      // Increase motor speed.
      if (motor1_speed <= (255-speed_accelerate)) {
        motor1_speed += speed_accelerate;
        motor1.setSpeed(motor1_speed);
      }
      if (motor2_speed <= (255-speed_accelerate)) {
        motor2_speed += speed_accelerate;
        motor2.setSpeed(motor2_speed);
      }

    } else {
      // Start going RIGHT at pre-defined speed.
      prev_state = 2;

      motor1_speed = speed_standard;
      motor2_speed = speed_standard;

      motor1.setSpeed(motor1_speed);
      motor2.setSpeed(motor2_speed);

      motor1.run(BACKWARD); 
      motor2.run(FORWARD); 
    } 
   // END OF RIGHT

  } else if (cmd_msg.data == 3) { //LEFT
    if (prev_state == 3) {
      // Increase motor speed.
      if (motor1_speed <= (255-speed_accelerate)) {
        motor1_speed += speed_accelerate;
        motor1.setSpeed(motor1_speed);
      }
      if (motor2_speed <= (255-speed_accelerate)) {
        motor2_speed += speed_accelerate;
        motor2.setSpeed(motor2_speed);
      }

    } else {
      // Start going LEFT at pre-defined speed.
      prev_state = 3;

      motor1_speed = speed_standard;
      motor2_speed = speed_standard;

      motor1.setSpeed(motor1_speed);
      motor2.setSpeed(motor2_speed);

      motor1.run(FORWARD); 
      motor2.run(BACKWARD); 
   // END OF LEFT

  } else if (cmd_msg.data == 4) { //BACKWARD
    if (prev_state == 4) {
      // Increase motor speed.
      if (motor1_speed <= (255-speed_accelerate)) {
        motor1_speed += speed_accelerate;
        motor1.setSpeed(motor1_speed);
      }
      if (motor2_speed <= (255-speed_accelerate)) {
        motor2_speed += speed_accelerate;
        motor2.setSpeed(motor2_speed);
      }

    } else {
      // Start going BACKWARD at pre-defined speed.
      prev_state = 4;

      motor1_speed = speed_standard;
      motor2_speed = speed_standard;

      motor1.setSpeed(motor1_speed);
      motor2.setSpeed(motor2_speed);

      motor1.run(BACKWARD); 
      motor2.run(BACKWARD); 
   //END OF BACKWARD

  } 

  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  

}


ros::Subscriber<std_msgs::UInt16> sub("motor", motor_cb);


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
  prev_state = 0;
  motor1_speed = 100;
  motor2_speed = 100;
  
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


