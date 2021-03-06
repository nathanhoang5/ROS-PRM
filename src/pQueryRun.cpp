#include "pQuery.h"
#include "ros/ros.h"
#include "prm/PRMQuery.h"
#include <prm/node.h>
#include <prm/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

bool timeTaken(prm::PRMQuery::Request  &req,
               prm::PRMQuery::Response &res){
    std::cout<<"running"<<std::endl;
    pQuery queryGame(req.numNodes, req.maxDistance, req.o);
    queryGame.n = req.nA;
    queryGame.sx = req.startX;
    queryGame.sy = req.startY;
    queryGame.ex = req.endX;
    queryGame.ey = req.endY;
	//res.runTime = mainGame.run();
	queryGame.run();
	res.nFinal = queryGame.nodePath;
	//res.nA = mainGame.n;
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

	ros::init(argc, argv, "PRMQuery_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("PRMQuery",timeTaken);

	ROS_INFO("Ready to run QUERY");
	ros::spin();

	return 0;
}
