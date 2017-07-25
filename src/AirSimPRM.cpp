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

float x = -3;
float y = -3;
float z = 5;

void prmCallback(const nav_msgs::OccupancyGrid& o)
{
    if(x!=-3&&y!=-3)
    {
        ros::NodeHandle nh;

        ros::ServiceClient client = nh.serviceClient<prm::PRM>("PRM");
        prm::PRM srv;

        ros::ServiceClient clientQ = nh.serviceClient<prm::PRMQuery>("PRMQuery");
        prm::PRMQuery srvQ;

        srv.request.startX = x;
        srv.request.startY = y;
        srv.request.endX = 0;
        srv.request.endY = 0;
        srv.request.numNodes = 500;
        srv.request.maxDistance = 50;

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

            ROS_INFO("Waiting for action server to start.");
            // wait for the action server to start
            ac.waitForServer(); //will wait for infinite time

            ROS_INFO("Action server started, sending goal.");
            // send a goal to the action
            prm::moveQuadGoal goal;
            goal.target = targetArray;
            ac.sendGoal(goal);

            //wait for the action to return
            bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("Action finished: %s",state.toString().c_str());
            }
            else
                ROS_INFO("Action did not finish before the time out.");
            //std::cout<<"Query called successfully!"<<std::endl;
        }

        else
        {
            ROS_ERROR("Failed to call service QUERY");

        }
    }
}

void quadPosSetter(const geometry_msgs::Pose& p)
{
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
