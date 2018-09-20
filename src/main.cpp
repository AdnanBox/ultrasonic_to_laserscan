#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include <wiringSerial.h> 
#include <iostream>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

#define trig1 3 //pin 15
#define echo1 6 //pin 22

#define trig2 2 //pin 13
#define echo2 10 //pin 24

#define trig3 7 //pin 7
#define echo3 11 //pin 26

#define trig4 22 //pin 31
#define echo4 26 //pin 32

#define trig5 4 //pin 16
#define echo5 5 //pin 18


static long travelTime1, startTime1;
static long travelTime2, startTime2;
static long travelTime3, startTime3;
static long travelTime4, startTime4;
static long travelTime5, startTime5;

void setup()
{
  wiringPiSetup();

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  pinMode(trig4, OUTPUT);
  pinMode(echo4, INPUT);
  pinMode(trig5, OUTPUT);
  pinMode(echo5, INPUT);

}

double calculate1()
{  

  digitalWrite(trig1, LOW);
  delay(5);
  digitalWrite(trig1, HIGH);
  delay(20);
  digitalWrite(trig1, LOW);
  long check1 = micros();
  while(digitalRead(echo1) == LOW)
    {
      startTime1 = micros();
      if (startTime1 - check1 > 12000)
      {
        pinMode(echo1, OUTPUT);
        digitalWrite(echo1, LOW);
        delay(10);
        pinMode(echo1, INPUT);
      }
    }

  while(digitalRead(echo1) == HIGH)
    {
      travelTime1 = micros() - startTime1;
    }

  double distance1 = travelTime1/58;  //distance in cm
  return distance1;
}

double calculate2()
{

  digitalWrite(trig2, LOW);
  delay(5);
  digitalWrite(trig2, HIGH);
  delay(20);
  digitalWrite(trig2, LOW);
  long check2 = micros();
  while(digitalRead(echo2) == LOW)
    {
      startTime2 = micros();
      if (startTime2 - check2 > 12000)
      {
        pinMode(echo2, OUTPUT);
        digitalWrite(echo2, LOW);
        delay(10);
        pinMode(echo2, INPUT);
      }
    }

  while(digitalRead(echo2) == HIGH)
    {
      travelTime2 = micros() - startTime2;
    }

  double distance2 = travelTime2/58;  //distance in cm
  return distance2;
}

double calculate3()
{

  digitalWrite(trig3, LOW);
  delay(5);
  digitalWrite(trig3, HIGH);
  delay(20);
  digitalWrite(trig3, LOW);
  long check3 = micros();
  while(digitalRead(echo3) == LOW)
    {
      startTime3 = micros();
      if (startTime3 - check3 > 12000)
      {
        pinMode(echo3, OUTPUT);
        digitalWrite(echo3, LOW);
        delay(10);
        pinMode(echo3, INPUT);
      }
    }

  while(digitalRead(echo3) == HIGH)
    {
      travelTime3 = micros() - startTime3;
    }

  double distance3 = travelTime3/58;  //distance in cm
  return distance3;
}

double calculate4()
{

  digitalWrite(trig4, LOW);
  delay(5);
  digitalWrite(trig4, HIGH);
  delay(20);
  digitalWrite(trig4, LOW);
  long check4 =micros();
  while(digitalRead(echo4) == LOW)
    {
      startTime4 = micros();
      if (startTime4 - check4 > 12000)
      {
        pinMode(echo4, OUTPUT);
        digitalWrite(echo4, LOW);
        delay(10);
        pinMode(echo4, INPUT);
      }
    }

  while(digitalRead(echo4) == HIGH)
    {
      travelTime4 = micros() - startTime4;
    }

  double distance4 = travelTime4/58;  //distance in cm
  return distance4;
}

double calculate5()
{

  digitalWrite(trig5, LOW);
  delay(5);
  digitalWrite(trig5, HIGH);
  delay(20);
  digitalWrite(trig5, LOW);
  long check5 = micros();
  while(digitalRead(echo5) == LOW)
    {
      startTime5 = micros();
      if (startTime5 - check5 > 12000)
      {
        pinMode(echo5, OUTPUT);
        digitalWrite(echo5, LOW);
        delay(10);
        pinMode(echo5, INPUT);
      }
    }

  while(digitalRead(echo5) == HIGH)
    {
      travelTime5 = micros() - startTime5;
    }

  double distance5 = travelTime5/58;  //distance in cm
  return distance5;
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

    geometry_msgs::Vector3 range1; 
    range1.z = calculate1()/100;
    range1.z = range1.z*1.0f;
    //msg.ranges={};
    if (range1.z <= msg.range_max && range1.z >= msg.range_min)
    {
      msg.ranges.push_back(range1.z);
    }
    else
    {
      msg.ranges.push_back(std::numeric_limits<double>::infinity());
    }

ROS_INFO_STREAM("test 1  " << range1.z);


    geometry_msgs::Vector3 range2; 
    range2.z = calculate2()/100;
    range2.z = range2.z*1.0f;
    //msg.ranges={};
    if (range2.z <= msg.range_max && range2.z >= msg.range_min)
    {
      msg.ranges.push_back(range2.z);
    }
    else
    {
      msg.ranges.push_back(std::numeric_limits<double>::infinity());
    }

ROS_INFO_STREAM("test 2  " << range2.z);



    geometry_msgs::Vector3 range3; 
    range3.z = calculate3()/100;
    range3.z = range3.z*1.0f;
    //msg.ranges={};
    if (range3.z <= msg.range_max && range3.z >= msg.range_min)
    {
      msg.ranges.push_back(range3.z);
    }
    else
    {
      msg.ranges.push_back(std::numeric_limits<double>::infinity());
    }

ROS_INFO_STREAM("test 3  " << range3.z);


    geometry_msgs::Vector3 range4; 
    range4.z = calculate4()/100;
    range4.z = range4.z*1.0f;
    //msg.ranges={};
    if (range4.z <= msg.range_max && range4.z >= msg.range_min)
    {
      msg.ranges.push_back(range4.z);
    }
    else
    {
      msg.ranges.push_back(std::numeric_limits<double>::infinity());
    }

ROS_INFO_STREAM("test 4  " << range4.z);


    geometry_msgs::Vector3 range5; 
    range5.z = calculate5()/100;
    range5.z = range5.z*1.0f;
    //msg.ranges={};
    if (range5.z <= msg.range_max && range5.z >= msg.range_min)
    {
      msg.ranges.push_back(range5.z);
    }
    else
    {
      msg.ranges.push_back(std::numeric_limits<double>::infinity());
    }

ROS_INFO_STREAM("test 5  " << range5.z);



    //msg.ranges[0]=range.z;
   
    pub.publish(msg);
    //msg.ranges.pop_back();
    ros::spinOnce();
    rate.sleep();
    
  }

  return 0;

}
