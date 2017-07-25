#include "ros/ros.h"
#include "std_msgs/String.h"
#include <px4_control/PVA.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pubPVA");

    ros::NodeHandle n;

    ros::Publisher PVAControl = n.advertise<px4_control::PVA>("/px4_control/PVA_Ref", 10);

    ros::Rate loop_rate(1);

    px4_control::PVA msg;
    msg.Pos.x = 10;
    msg.Pos.y = 10;
    msg.Pos.z = 10;


    while(ros::ok())
    {
        PVAControl.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }




    return 0;
}
