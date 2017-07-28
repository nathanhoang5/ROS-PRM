#include "ros/ros.h"
#include "prm/PRM.h"
#include "prm/PRMQuery.h"
#include <prm/node.h>
#include <prm/nodeArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <px4_control/PVA.h>
#include <px4_control/PVAarray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <prm/moveQuadAction.h>
#include <cstdlib>

using namespace std;
/* 
max distance = 50, resolution = .1 : good fly distance = 5m
grid area = length*width*resolution^2
nodes: .4 nodes/m^2
nodes needed = .4 * grid area
*/

float x = -3;
float y = -3;
float z = 5.25;
bool needsReset = true;

void prmCallback(const nav_msgs::OccupancyGrid& o)
{
    
    if(!needsReset)
    {
        
        ros::NodeHandle nh;
        needsReset = true;

        int maxDistanceWorldFrame = 5;

        ros::ServiceClient client = nh.serviceClient<prm::PRM>("PRM");
        prm::PRM srv;

        ros::ServiceClient clientQ = nh.serviceClient<prm::PRMQuery>("PRMQuery");
        prm::PRMQuery srvQ;

        srv.request.startX = x;
        srv.request.startY = y;
        srv.request.endX = 0;
        srv.request.endY = 0;
        srv.request.numNodes = .2 * o.info.width * o.info.height * o.info.resolution * o.info.resolution;
        srv.request.maxDistance = maxDistanceWorldFrame/o.info.resolution;

        cout<<"Running with  "<<srv.request.numNodes<< " nodes"<<endl;

        srvQ.request.startX = 20;
        srvQ.request.startY = 20;
        srvQ.request.endX = 0;
        srvQ.request.endY = 0;
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



        if(clientQ.call(srvQ))
        {
            if(srvQ.response.nFinal.nodeLst.size()>0){
                px4_control::PVAarray targetArray;
                std::cout<<"Path:"<<std::endl;
                for(std::vector<prm::node>::const_iterator it = srvQ.response.nFinal.nodeLst.begin(); it != srvQ.response.nFinal.nodeLst.end(); ++it)
                {
                    prm::node g;
                    g = *it;
                    px4_control::PVA curTarget;
                    curTarget.Pos.x = g.x;
                    curTarget.Pos.y = g.y;
                    curTarget.Pos.z = z;
                    targetArray.data.push_back(curTarget);
                    std::cout<<"Node: "<<g.id<<" xPos: "<<g.x<<" yPos: "<<g.y<<std::endl;
                }
                actionlib::SimpleActionClient<prm::moveQuadAction> ac("movingQuad", true);

                ROS_INFO("Waiting for move quad server to start.");
                // wait for the action server to start
                ac.waitForServer(); //will wait for infinite time

                ROS_INFO("Sent quad waypoints.");
                // send a goal to the action
                prm::moveQuadGoal goal;
                goal.target = targetArray;
                ac.sendGoal(goal);

                //wait for the action to return
                bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

                if (finished_before_timeout)
                {
                    actionlib::SimpleClientGoalState state = ac.getState();
                    ROS_INFO("Quad reached position: %s",state.toString().c_str());
                }
                else
                    ROS_INFO("PRM failed.");
                //std::cout<<"Query called successfully!"<<std::endl;
        
            }
        }

        else
        {
            ROS_ERROR("Failed to call service QUERY");

        }
    }
}

void quadPosSetter(const geometry_msgs::Pose& p)
{
    // ROS_INFO("Grabbed new Pose");
    x = p.position.x;
    y = p.position.y;
    needsReset = false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "PRM_Client");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("projected_map", 1, prmCallback);
    ros::Subscriber quadPosSub = n.subscribe("Airsim/quadPos",1, quadPosSetter);

    ros::spin();
    return 0;
}
