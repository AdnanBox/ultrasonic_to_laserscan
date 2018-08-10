#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <wiringSerial.h> 
#include <iostream>

#define trig 3 //pin 15
#define echo 6 //pin 16
long travelTime, startTime;
static double clearance = 0;

void setup()
{
  wiringPiSetup();
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);
  delay(30);
}

int calculate()
{
  digitalWrite(trig, HIGH);
  delay(20);
  digitalWrite(trig, LOW);

  while(digitalRead(echo) == LOW)
    {
      startTime = micros();
    }

  while(digitalRead(echo) == HIGH)
    {
      travelTime = micros() - startTime;
    }

  double distance = travelTime/58;  //distance in cm
  return distance;
}

int main(int argc, char **argv)
{

  setup();
  ros::init(argc, argv, "clearance_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float32>("clearance", 1000);
  ros::Rate rate(10);

  while (ros::ok())
  {

    std_msgs::Float32 msg;
    clearance = calculate();
    msg.data = clearance;
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
    
  }

  return 0;

}
