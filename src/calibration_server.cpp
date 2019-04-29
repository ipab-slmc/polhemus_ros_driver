#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool calibrate(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calibration", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}