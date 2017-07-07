#include "MainGame.h"
#include "ros/ros.h"
#include "beginner_tutorials/PRM.h"
#include <beginner_tutorials/node.h>
#include <beginner_tutorials/nodeArray.h>

using namespace std;

bool timeTaken(beginner_tutorials::PRM::Request  &req,
               beginner_tutorials::PRM::Response &res){
    std::cout<<"running"<<std::endl;
    MainGame mainGame;
	res.runTime = mainGame.run();
	/*
	for(std::vector<beginner_tutorials::node>::const_iterator it = mainGame.n.nodeLst.begin(); it != mainGame.n.nodeLst.end(); ++it)
	{
	    beginner_tutorials::node g;
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
