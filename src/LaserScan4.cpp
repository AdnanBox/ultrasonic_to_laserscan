#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <wiringSerial.h> 
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

#define trig 22 //pin 31
#define echo 26 //pin 32

static long travelTime, startTime;
double range_min_, range_max_;
int freq_;


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
  ros::init(argc, argv, "scan_4");
  ros::NodeHandle n;
  ros::param::get("/scan_4/freq", freq_);

  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan_4", 1000);
  ros::Rate rate(freq_);

  while (ros::ok())
  {
    ros::param::get("/scan_4/min_range_4", range_min_);
    ros::param::get("/scan_4/max_range_4", range_max_);
    ROS_INFO_STREAM("test   4  "<<range_max_);

    sensor_msgs::LaserScan msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "scan_link_4";
    msg.angle_min = 1.92-0.05; //117.5 degrees
    msg.angle_max = 1.92+0.05; //122.5 degrees
    msg.angle_increment = 0.1; //5 degrees
    msg.time_increment = 0;
    msg.scan_time = (2.0);
    msg.range_min = range_min_; //2cm
    msg.range_max = range_max_; //2m 
    //ROS_INFO_STREAM("test   "<<msg.range_max);

    uint32_t ranges_size = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);
    msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    geometry_msgs::Vector3 range; 
    range.z = calculate()/100;
    range.z = range.z*1.0f;
    //msg.ranges={};
    if (range.z <= range_max_ && range.z >= range_min_)
    {
      msg.ranges.push_back(range.z);
    }
    else
    {
      msg.ranges.push_back(std::numeric_limits<double>::infinity());
    }

    //msg.ranges[0]=range.z;
    
    pub.publish(msg);
    //msg.ranges.pop_back();
    ros::spinOnce();
    rate.sleep();
    
  }

  return 0;

}
