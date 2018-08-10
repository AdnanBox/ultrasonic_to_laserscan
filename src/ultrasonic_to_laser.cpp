#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"




void ultrasonicCB(std_msgs::Float32 msg)
{ 
  ros::NodeHandle np;
  ros::Publisher laserpub = np.advertise<sensor_msgs::LaserScan>("entfernung", 1000);
  sensor_msgs::LaserScan output;
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "Ultrasonic_1";
  output.angle_min = 0.785-0.05; //30 degrees
  output.angle_max = 0.785+0.05; //150 degrees
  output.angle_increment = 0.1; //30 degrees
  output.time_increment = 0;
  output.scan_time = (2.0);
  output.range_min = 0.02; //2cm
  output.range_max = 20.00; //20m 
  double range = msg.data;
  
  
  //uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
  //output.ranges.assign(ranges_size, output.range_max + 0.1);

  output.ranges = [msg.data/100];
  /*for(int i=0; i <= output.ranges.size(); i++)
  {
    if(range < output.ranges[i])
    {
      output.ranges[i] = range;
    }
    laserpub.publish(output);
  }*/
  
  laserpub.publish(output);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ultrasonic_to_laserscan");
  ros::NodeHandle n;
  
  ros::Subscriber odroidSub = n.subscribe("clearance", 1000, ultrasonicCB);
  ros::spin();
  return 0; 
} 
