#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <prm/moveQuadAction.h>
#include <px4_control/PVA.h>
#include <px4_control/PVAarray.h>

class moveQuadAction
{
public:

  moveQuadAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&moveQuadAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&moveQuadAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/Airsim/quadPos", 1, &moveQuadAction::positionCB, this);
    PVAControl = nh_.advertise<px4_control::PVA>("/px4_control/PVA_Ref", 10);
    as_.start();
  }

  ~moveQuadAction(void)
  {
  }

  void goalCB()
  {

    targetNodes = as_.acceptNewGoal()->target;
    curTarget = targetNodes.data.front();
    PVAControl.publish(curTarget);
    ROS_INFO("Publishhing first node!");
    std::cout<<"Publishing first node "<< targetNodes.data.front().Pos.x;
    targetNodes.data.erase(targetNodes.data.begin());

  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void positionCB(const geometry_msgs::Pose& curPos)
  {
    // make sure that the action hasn't been canceled

    if (!as_.isActive())
      return;

    maxDev = 1;
    feedback_.currentPosition = curPos;
    as_.publishFeedback(feedback_);
    
    if(finished){
      result_.finalPosition.position.x = curPos.position.x;
      result_.finalPosition.position.y = curPos.position.y;
      result_.finalPosition.position.z = curPos.position.z;
      as_.setSucceeded(result_);
      finished = false;
      spin = false;
    }
    else if(!spin){
      if(curPos.position.x > curTarget.Pos.x-maxDev && curPos.position.x < curTarget.Pos.x+maxDev &&
         curPos.position.y > curTarget.Pos.y-maxDev && curPos.position.y < curTarget.Pos.y+maxDev ){

         if(targetNodes.data.size()==0){
              spin = true;
              

         }
         else{
             curTarget = targetNodes.data.front();
             yaw+=yawIncrement;
             if(yaw==2*PI_F)yaw = 0;
             curTarget.yaw=yaw;
             PVAControl.publish(curTarget);
             targetNodes.data.erase(targetNodes.data.begin());
             ROS_INFO("Publishing next node");
             
         }
      }
    }
    else{
      ROS_INFO("Spinning: %f", curTarget.yaw);
      ros::Duration(0.5).sleep();
      yaw+=yawIncrement;
      if(yaw==2*PI_F)yaw = 0;
      curTarget.yaw=yaw;
      PVAControl.publish(curTarget);
      spinCounter++;
      if(spinCounter == spinMax)finished = true;
    }
  }

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<prm::moveQuadAction> as_;
  std::string action_name_;
  px4_control::PVAarray targetNodes;
  px4_control::PVA curTarget;
  prm::moveQuadFeedback feedback_;
  prm::moveQuadResult result_;
  ros::Subscriber sub_;
  ros::Publisher PVAControl;
  const float  PI_F=3.14159265358979f;
  float maxDev = .15;
  float yaw = 0, yawIncrement = PI_F/8;
  bool spin = false, finished = false;
  int numSpins = 2, spinMax = 1/(yawIncrement/PI_F)*numSpins, spinCounter = 0;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movingQuad");
  std::cout<<"Ready to move quad (PRM)"<<std::endl;
  moveQuadAction mover(ros::this_node::getName());
  ros::spin();

  return 0;
}
