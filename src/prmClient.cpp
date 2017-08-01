#include "ros/ros.h"
#include "prm/PRM.h"
#include "prm/PRMQuery.h"
#include <prm/node.h>
#include <prm/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>
/*
void prmCallback(const nav_msgs::OccupancyGrid& o){


}
*/

/* 
max distance = 50, resolution = .1 : good fly distance = 5m
grid area = length*width*resolution^2
nodes: .4 nodes/m^2
nodes needed = .4 * grid area
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PRM_Client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<prm::PRM>("PRM");
  prm::PRM srv;

  ros::ServiceClient clientQ = n.serviceClient<prm::PRMQuery>("PRMQuery");
  prm::PRMQuery srvQ;
  //srv.request.a = 0;
  srv.request.startX = 20;
  srv.request.startY = 20;
  srv.request.endX = 460;
  srv.request.endY = 270;
  srv.request.numNodes = 500;
  srv.request.maxDistance = 50;

  srvQ.request.numNodes = srv.request.numNodes;
  srvQ.request.maxDistance = srv.request.maxDistance;
  srvQ.request.startX = srv.request.startX;
  srvQ.request.startY = srv.request.startY;
  srvQ.request.endX = 460;
  srvQ.request.endY = 20;

  const int sw = 500;
  const int sh = 300;
  nav_msgs::OccupancyGrid obs;
  obs.info.width = sw;
  obs.info.height = sh;
  obs.info.resolution = 1;

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

  for(int j = sh-1; j>=0; j--)
    {

        for(int i = 0; i<sw; i++)
        {
        obs.data.push_back(occupancyGrid[i][j]);
    }
  }
  srv.request.o = obs;
  srvQ.request.o = srv.request.o;

  if (client.call(srv))
  {
    /*
    for(std::vector<prm::node>::const_iterator it = srv.response.nA.nodeLst.begin(); it != srv.response.nA.nodeLst.end(); ++it)
    {
	    prm::node g;
	    g = *it;
	    std::cout<<g.id<<std::endl;
    }
    */
    srvQ.request.nA = srv.response.nA;
    //std::cout<<"Populate and connect called successfully!"<<std::endl;

  }
  else
  {
    ROS_ERROR("Failed to call service PRM");
    return 1;
  }



  if(clientQ.call(srvQ)){
    std::cout<<"Path:"<<std::endl;
    for(std::vector<prm::node>::const_iterator it = srvQ.response.nFinal.nodeLst.begin(); it != srvQ.response.nFinal.nodeLst.end(); ++it)
    {
	    prm::node g;
	    g = *it;
	    std::cout<<"Node: "<<g.id<<" xPos: "<<g.x<<" yPos: "<<g.y<<std::endl;
    }
    //std::cout<<"Query called successfully!"<<std::endl;
  }

  else
  {
    ROS_ERROR("Failed to call service QUERY");
    return 1;
  }
  return 0;
}
