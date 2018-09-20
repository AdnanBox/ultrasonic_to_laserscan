#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <wiringSerial.h> 
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
/*
#define trig1 3 //pin 15
#define echo1 6 //pin 22

#define trig2 2 //pin 13
#define echo2 10 //pin 24

#define trig3 7 //pin 7
#define echo3 11 //pin 26

#define trig4 21 //pin 29
#define echo4 23 //pin 33

#define trig5 22 //pin 31
#define echo5 26 //pin 32
*/

int trig[5]={3, 2, 7, 21, 22};
int echo[5]={6, 10, 11, 23, 26};

static long travelTime, startTime;


void setup()
{
  wiringPiSetup();
  for (int i=0; i<5; i++)
  {
    pinMode(trig[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }
}


int main(int argc, char **argv)
{

  setup();
  ros::init(argc, argv, "scan");
  ros::NodeHandle n;
  

  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::Rate rate(1);

  while (ros::ok())
  {
    


    sensor_msgs::LaserScan msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "scan_link";
    msg.angle_min = 0.87; //50 degrees
    msg.angle_max = 2.26; //150 degrees
    msg.angle_increment = 0.35; //20 degrees
    msg.time_increment = 0;
    msg.scan_time = (2.0);
    msg.range_min = 0.02; //2cm
    msg.range_max = 2.00; //2m 


    uint32_t ranges_size = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);
    msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    ROS_INFO_STREAM("test range_size :   "<<ranges_size);
    

    for (int j=0; j<5; j++)
    {
      digitalWrite(trig[j], LOW);
      delay(5);
      digitalWrite(trig[j], HIGH);
      delay(20);
      digitalWrite(trig[j], LOW);
      long check = micros();
      while(digitalRead(echo[j]) == LOW)
      {
        startTime = micros();
        if (startTime - check >= 12000)
        {
          pinMode(echo[j], OUTPUT);
          digitalWrite(echo[j], LOW);
          delay(10);
          pinMode(echo[j], INPUT);
        }
      }

      while(digitalRead(echo[j]) == HIGH)
      {
        travelTime = micros() - startTime;
      }

      double distance = travelTime/58;
    
      geometry_msgs::Vector3 range; 
      range.z = distance/100;
      range.z = range.z*1.0f;
      //msg.ranges={};
      if (range.z <= msg.range_max && range.z >= msg.range_min)
      {
        msg.ranges.push_back(range.z);
      }
      else
      {
        msg.ranges.push_back(std::numeric_limits<double>::infinity());
      }

      ROS_INFO_STREAM("test "<<j+1<<"  "<< range.z);
    }

    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
    
  }

  return 0;

}
