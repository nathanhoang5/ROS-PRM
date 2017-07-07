#include "MainGame.h"
#include "ros/ros.h"
#include "beginner_tutorials/PRM.h"

bool timeTaken(beginner_tutorials::PRM::Request  &req,
               beginner_tutorials::PRM::Response &res){
    std::cout<<"running"<<std::endl;
    MainGame mainGame;
	res.runTime = mainGame.run();
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
