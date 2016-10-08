#include <ros.h>
#include <sensor_msgs/JointState.h>

// FR, FL, BR, BL
const int pinAforEncoder[] = {10,8,2,12}; //Array of pins set up as A pin -> the interrupt pin
const int pinBforEncoder[] = {11,9,3,13}; //Array of pins set up as B pin

//Noen av motorene går motsatt vei, så noen av verdiene må flippes
int flippedDirections[4] = { -1, 1, -1, 1 };


byte pinAPrev[4]; //previous value of the pins 1-4
int pulses[4]; //the number of the pulses from motor 1-4
int Direction[4]; //the rotation direction of motor 1-4
float rpm[4]; //rotations per minute of motor 1-4
float speeds[4]; //speed for motor 1-4 in m/s
long timeDelay;
long interruptsStartedTime;
long deltaTime;
float period[4];
float frequency[4];
float test[4];

float ppr; //pulse per rotation for our motor
float wheelradi;
float gearRatio;
float pi;

ros::NodeHandle_<ArduinoHardware, 2, 2, 1000, 1000> nh;

sensor_msgs::JointState msg;

ros::Publisher pub("motor_speeds", &msg);

void setup()
{
  msg.velocity_length = 4;
  
  //Initialize ROS subscription
  nh.initNode();
  nh.advertise(pub);

  //Encoderkode:
  EncoderInit();//Initialize the module
  ppr = 1440; 
  gearRatio = 0.667;
  wheelradi = (0.2032/2); //metres
  pi = 3.14159265;
  timeDelay = 10;
}

void loop()
{
  nh.spinOnce();

  //start encoderkode:
  noInterrupts();
  deltaTime = micros() - interruptsStartedTime;
  
  for (int i = 0; i<4;i++){
    period[i] =  (float)deltaTime * 2 / (float)((float)pulses[i] * (float)1000000);
    pulses[i] = 0;
    
    if (period[i] != 0) {
      frequency[i] = (float)1/(float)period[i]; //frekvensen i pulser/sekund
    }
    rpm[i] = flippedDirections[i] * Direction[i] * frequency[i] * 60 / ppr; //Må muligens adde gearration
  }
  interruptsStartedTime = micros();
  interrupts();
  /**
  //slutt encoderkode
  for ( int i = 0; i< 4; i++) {
    noInterrupts();
    test[i] = (float)pulses[i];
    pulses[i] = 0;
    interrupts();
  }
**/
/**
  for ( int i = 0; i < 4; i++) {
    test[i] = analogRead(i);
  }
  **/
  
  msg.velocity = rpm;
  
  pub.publish(&msg);
  Serial.flush();
  delay(10);
}

void EncoderInit()
{
  for (int i = 0; i<4;i++){
    Direction[i] = 1;//default -> Forward
    pinMode(pinBforEncoder[i],INPUT);
    interruptsStartedTime = micros();
    deltaTime = micros();
    period[i] = 1;
    //attachInterrupt(pinAforEncoder[i], wheelSpeed(i), CHANGE);
  }
  attachInterrupt(pinAforEncoder[0], wheel0, CHANGE);
  attachInterrupt(pinAforEncoder[1], wheel1, CHANGE);
  attachInterrupt(pinAforEncoder[2], wheel2, CHANGE);
  attachInterrupt(pinAforEncoder[3], wheel3, CHANGE);
}

void wheel0(){
  wheelSpeed(0);
  //pulses[0]++;
}
void wheel1(){
  wheelSpeed(1);
  //pulses[1]++;
}
void wheel2(){
  wheelSpeed(2);
  //pulses[2]++;
}
void wheel3(){
  wheelSpeed(3);
  //pulses[3]++;  
}
  
void wheelSpeed(int n){
  
  int Lstate = digitalRead(pinAforEncoder[n]);
  if((pinAPrev[n] == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(pinBforEncoder[n]);
    if(val == HIGH && Direction[n] == -1)
    {
      Direction[n] = 1; //Reverse
    }
    else if(val == LOW && Direction[n] == 1)
    {
      Direction[n] = -1;  //Forward
    }
  }
  pinAPrev[n] = Lstate;

  pulses[n]++;
}


