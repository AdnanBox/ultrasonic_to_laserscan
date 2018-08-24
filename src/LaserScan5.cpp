#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <wiringSerial.h> 
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

#define trig 23 //pin 33
#define echo 27 //pin 36

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
  ros::init(argc, argv, "scan_5");
  ros::NodeHandle n;
  //ros::param::get("/scan_5/freq", freq_);

  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan_5", 1000);
  ros::Rate rate(10);

  while (ros::ok())
  {
    /*ros::param::get("/scan_5/min_range_5", range_min_);
    ros::param::get("/scan_5/max_range_5", range_max_);*/
    //ROS_INFO_STREAM("test  5  "<<range_max_);

    sensor_msgs::LaserScan msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "scan_link_5";
    msg.angle_min = 2.27-0.05; //127,5 degrees
    msg.angle_max = 2.27+0.05; //132,5 degrees
    msg.angle_increment = 0.1; //5 degrees
    msg.time_increment = 0;
    msg.scan_time = (2.0);
    msg.range_min = 0.02; //2cm
    msg.range_max = 2.00; //2m 
    //ROS_INFO_STREAM("test   "<<msg.range_max);

    uint32_t ranges_size = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);
    msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    geometry_msgs::Vector3 range; 
    range.z = calculate()/100;
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

    //msg.ranges[0]=range.z;
    
    pub.publish(msg);
    //msg.ranges.pop_back();
    ros::spinOnce();
    rate.sleep();
    
  }

  return 0;

}
