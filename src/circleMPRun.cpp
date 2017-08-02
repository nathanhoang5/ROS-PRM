#include "ros/ros.h"
#include "circleMP.h"
#include "prm/circularMissionPlan.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

using namespace std;
circleMP* missionPlanner;
bool instantiated = false;

bool getPoint(prm::circularMissionPlan::Request  &req,
               prm::circularMissionPlan::Response &res){

	if(!instantiated){
		missionPlanner = new circleMP(req.dr, req.dx, req.maxRadius);
		instantiated = true;
		ROS_INFO("Created new mission planner");
	}
	ROS_INFO("Fetching point");
    
	res.p = missionPlanner->getNextPoint(req.o);

    return true;
}



int main(int argc, char** argv) {

	ros::init(argc, argv, "circleMP_Server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("circleMP", getPoint);

	ROS_INFO("Ready to run MISSION PLANNER");
	ros::spin();
	delete missionPlanner;
	return 0;
}