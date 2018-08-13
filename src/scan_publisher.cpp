#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <wiringSerial.h> 
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

#define trig 3 //pin 15
#define echo 6 //pin 22
static long travelTime, startTime;

void setup()
{
  wiringPiSetup();
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);
  delay(30);
}

double calculate()
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

  double distance = travelTime/58;  //distance in m
  return distance;
}

int main(int argc, char **argv)
{

  setup();
  ros::init(argc, argv, "scan_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::Rate rate(10);

  while (ros::ok())
  {

    sensor_msgs::LaserScan msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "scan_link";
    msg.angle_min = 0.785-0.05; //30 degrees
    msg.angle_max = 0.785+0.05; //150 degrees
    msg.angle_increment = 0.1; //30 degrees
    msg.time_increment = 0;
    msg.scan_time = (2.0);
    msg.range_min = 0.02; //2cm
    msg.range_max = 2.00; //2m 

    uint32_t ranges_size = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);
    msg.ranges.assign(ranges_size, msg.range_max -0.01);

    geometry_msgs::Vector3 range; 
    range.z = calculate()/100;
    range.z = range.z*1.0f;
    //msg.ranges[0];
    msg.ranges.push_back(range.z);
    //msg.ranges[0]=range.z;
    
    pub.publish(msg);
    //msg.ranges.pop_back();
    ros::spinOnce();
    rate.sleep();
    
  }

  return 0;

}
