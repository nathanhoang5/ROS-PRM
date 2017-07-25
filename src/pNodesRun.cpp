#include "populateNodes.h"
#include "ros/ros.h"
#include "prm/PRM.h"
#include <prm/node.h>
#include <prm/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>


using namespace std;

bool timeTaken(prm::PRM::Request  &req,
               prm::PRM::Response &res){
    std::cout<<"running"<<std::endl;
    populateNodes nodeCreator(req.numNodes, req.o);
	
	nodeCreator.run(req.startX, req.startY, req.endX, req.endY, req.maxDistance);
	res.nA = nodeCreator.n;
	/*
	for(std::vector<prm::node>::const_iterator it = mainGame.n.nodeLst.begin(); it != mainGame.n.nodeLst.end(); ++it)
	{
	    prm::node g;
	    g = *it;
	    cout<<g.id<<endl;
    }
    */
    return true;
}



int main(int argc, char** argv) {

	ros::init(argc, argv, "PRM_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("PRM",timeTaken);

	ROS_INFO("Ready to run PRM");
	ros::spin();

	return 0;
}
