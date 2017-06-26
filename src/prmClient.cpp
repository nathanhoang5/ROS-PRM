#include "ros/ros.h"
#include "beginner_tutorials/PRM.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PRM_Client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::PRM>("PRM");
  beginner_tutorials::PRM srv;
  //srv.request.a = 0;

  if (client.call(srv))
  {
    //std::cout<<((double)srv.response.runTime)<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
