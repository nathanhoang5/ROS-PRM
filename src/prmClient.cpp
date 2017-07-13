#include "ros/ros.h"
#include "beginner_tutorials/PRM.h"
#include "beginner_tutorials/PRMQuery.h"
#include <beginner_tutorials/node.h>
#include <beginner_tutorials/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PRM_Client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::PRM>("PRM");
  beginner_tutorials::PRM srv;

  ros::NodeHandle nh;
  ros::ServiceClient clientQ = nh.serviceClient<beginner_tutorials::PRMQuery>("PRMQuery");
  beginner_tutorials::PRMQuery srvQ;
  //srv.request.a = 0;
  srv.request.startX = 20;
  srv.request.startY = 20;
  srv.request.endX = 460;
  srv.request.endY = 270;
  srv.request.numNodes = 500;
  srv.request.maxDistance = 50;

  srvQ.request.numNodes = srv.request.numNodes;
  srvQ.request.maxDistance = srv.request.maxDistance;
  srvQ.request.startX = 20;
  srvQ.request.startY = 270;
  srvQ.request.endX = 460;
  srvQ.request.endY = 20;

  const int sw = 500;
  const int sh = 300;
  nav_msgs::OccupancyGrid obs;
  obs.info.width = sw;
  obs.info.height = sh;
  int occupancyGrid[sw][sh];
  //Create Obstacle: x1, x2, y1, y2
  for(int i = 0; i<sw; i++){
        for(int j = 0; j<sh; j++){
            if(i<170&&i>110&&j<250&&j>0){
                occupancyGrid[i][j]=1;
            }
            else{
                occupancyGrid[i][j]=0;
            }
        }
  }

  for(int i = 0; i<sw; i++){
        for(int j = 0; j<sh; j++){
            if(i<380&&i>350&&j<300&&j>50){
                occupancyGrid[i][j]=1;
            }
        }
  }

  for(int j = 0; j<sh; j++){
    for(int i = 0; i<sw; i++){
        obs.data.push_back(occupancyGrid[i][j]);

    }
  }
  srv.request.o = obs;
  srvQ.request.o = srv.request.o;

  if (client.call(srv))
  {
    /*
    for(std::vector<beginner_tutorials::node>::const_iterator it = srv.response.nA.nodeLst.begin(); it != srv.response.nA.nodeLst.end(); ++it)
    {
	    beginner_tutorials::node g;
	    g = *it;
	    std::cout<<g.id<<std::endl;
    }
    */
    srvQ.request.nA = srv.response.nA;
    std::cout<<"Populate and connect called successfully!"<<std::endl;

  }
  else
  {
    ROS_ERROR("Failed to call service PRM");
    return 1;
  }



  if(clientQ.call(srvQ)){
    std::cout<<"Path:"<<std::endl;
    for(std::vector<beginner_tutorials::node>::const_iterator it = srvQ.response.nFinal.nodeLst.begin(); it != srvQ.response.nFinal.nodeLst.end(); ++it)
    {
	    beginner_tutorials::node g;
	    g = *it;
	    std::cout<<"Node: "<<g.id<<" xPos: "<<g.x<<" yPos: "<<g.y<<std::endl;
    }
    std::cout<<"Query called successfully!"<<std::endl;
  }

  else
  {
    ROS_ERROR("Failed to call service QUERY");
    return 1;
  }
  return 0;
}
