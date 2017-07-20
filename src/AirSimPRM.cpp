#include "ros/ros.h"
#include "beginner_tutorials/PRM.h"
#include "beginner_tutorials/PRMQuery.h"
#include <beginner_tutorials/node.h>
#include <beginner_tutorials/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>


float x = -3;
float y = -3;

void prmCallback(const nav_msgs::OccupancyGrid& o){
  if(x!=-3&&y!=-3){

      ros::NodeHandle nh;

      ros::ServiceClient client = nh.serviceClient<beginner_tutorials::PRM>("PRM");
      beginner_tutorials::PRM srv;

      ros::ServiceClient clientQ = nh.serviceClient<beginner_tutorials::PRMQuery>("PRMQuery");
      beginner_tutorials::PRMQuery srvQ;

      srv.request.startX = x;
      srv.request.startY = y;
      srv.request.endX = 460;
      srv.request.endY = 270;
      srv.request.numNodes = 500;
      srv.request.maxDistance = 50;

      srvQ.request.startX = 20;
      srvQ.request.startY = 20;
      srvQ.request.endX = 460;
      srvQ.request.endY = 270;
      srvQ.request.numNodes = srv.request.numNodes;
      srvQ.request.maxDistance = srv.request.maxDistance;

      srv.request.o = o;
      srvQ.request.o = o;

      if (client.call(srv))
      {
        srvQ.request.nA = srv.response.nA;
      }

      else
      {
        ROS_ERROR("Failed to call service PRM");

      }



      if(clientQ.call(srvQ)){
        std::cout<<"Path:"<<std::endl;
        for(std::vector<beginner_tutorials::node>::const_iterator it = srvQ.response.nFinal.nodeLst.begin(); it != srvQ.response.nFinal.nodeLst.end(); ++it)
        {
            beginner_tutorials::node g;
            g = *it;
            std::cout<<"Node: "<<g.id<<" xPos: "<<g.x<<" yPos: "<<g.y<<std::endl;
        }
        //std::cout<<"Query called successfully!"<<std::endl;
      }

      else
      {
        ROS_ERROR("Failed to call service QUERY");

      }
  }

}

void quadPosSetter(const geometry_msgs::Pose& p){
    x = p.position.x;
    y = p.position.y;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "PRM_Client");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("projected_map", 1000, prmCallback);
  ros::Subscriber quadPosSub = n.subscribe("Airsim/quadPos",100, quadPosSetter);

  ros::spin();
  return 0;
}
