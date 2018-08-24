#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <wiringSerial.h> 
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

#define trig_1 3 //pin 15
#define echo_1 6 //pin 22

#define trig_2 2 //pin 13
#define echo_2 11 //pin 26

static long travelTime_1, startTime, travelTime_2, startTime_2;
double distance[2]={0,0};
void setup()
{
  wiringPiSetup();
  pinMode(trig_1, OUTPUT);
  pinMode(echo_1, INPUT);
  pinMode(trig_2, OUTPUT);
  pinMode(echo_2, INPUT);

  digitalWrite(trig_1, LOW);
  digitalWrite(trig_2, LOW);

  delay(30);
}

double calculate()
{ ROS_INFO_STREAM("test 4");
  digitalWrite(trig_1, HIGH);
  digitalWrite(trig_2, HIGH);
  delay(20);
  digitalWrite(trig_1, LOW);
  digitalWrite(trig_2, LOW);
  ROS_INFO_STREAM("test 5");
  while(digitalRead(echo_1) == LOW && digitalRead(echo_2) == LOW)
    {
      startTime = micros();
    }
  ROS_INFO_STREAM("test 5.5  "<< startTime<<"   "<<micros());
  /*while(digitalRead(echo_2) == LOW)
    {
      startTime_2 = micros();
      ROS_INFO_STREAM("startTime_2  :  "<<startTime_2);
    }
  ROS_INFO_STREAM("test 6  "<< startTime_2<<"   "<<micros());*/

  while(digitalRead(echo_1) == HIGH)
    {
      travelTime_1 = micros() - startTime;
    }
  ROS_INFO_STREAM("test 6.5   "<<micros()<<"   "<<travelTime_1);
  while(digitalRead(echo_2) == HIGH)
    {
      travelTime_2 = micros() - startTime;
    }

  ROS_INFO_STREAM("test 7    "<<micros()<<"   "<<travelTime_2);

  distance[0] = travelTime_1/58;  //distance in m
  distance[1] = travelTime_2/58;  //distance in m
  ROS_INFO_STREAM("test 7.5 : "<< distance[0]);
  //return distance;
}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM("test 1");
  setup();
  ROS_INFO_STREAM("test 2");
  ros::init(argc, argv, "scan");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::Rate rate(10);

  while (ros::ok())
  {
    ROS_INFO_STREAM("test 3");
    sensor_msgs::LaserScan msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "scan_link";
    msg.angle_min = 0.785; //30 degrees
    msg.angle_max = 2.355; //150 degrees
    msg.angle_increment = 0.785; //30 degrees
    msg.time_increment = 0;
    msg.scan_time = (2.0);
    msg.range_min = 0.02; //2cm
    msg.range_max = 2.00; //2m 

    uint32_t ranges_size = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);
    msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    geometry_msgs::Vector3 range; 
    calculate();
    ROS_INFO_STREAM("test 8");
    for(int i=0; i<2; i++)
    {
      
      range.z = distance[i]/100;
      range.z = range.z*1.0f;
      ROS_INFO_STREAM("test 9 : "<< range.z << i);
      //msg.ranges={};
      if (range.z <= msg.range_max && range.z >= msg.range_min)
      {
        msg.ranges.push_back(range.z);
      }
      else
      {
        msg.ranges.push_back(std::numeric_limits<double>::infinity());
      }
    }
    //msg.ranges[0]=range.z;
    
    pub.publish(msg);
    ROS_INFO_STREAM("test 10");    
    //msg.ranges.pop_back();
    //msg.ranges={};
    ros::spinOnce();
    rate.sleep();
    
  }

  return 0;

}
