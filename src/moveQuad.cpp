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
    /*
    data_count_++;
    feedback_.sample = data_count_;
    feedback_.data = msg->data;
    //compute the std_dev and mean of the data
    sum_ += msg->data;
    feedback_.mean = sum_ / data_count_;
    sum_sq_ += pow(msg->data, 2);
    feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
    */
    ROS_INFO("CB is being called");
    maxDev = 1;
    feedback_.currentPosition = curPos;
    as_.publishFeedback(feedback_);
    ROS_INFO("Current x: %f", curPos.position.x);
    ROS_INFO("Current xmin: %f", curTarget.Pos.x-maxDev);
    ROS_INFO("Current xmax: %f", curTarget.Pos.x+maxDev);
    ROS_INFO("Current y: %f", curPos.position.y);
    ROS_INFO("Current ymin: %f", curTarget.Pos.y-maxDev);
    ROS_INFO("Current ymax: %f", curTarget.Pos.y+maxDev);

    if(curPos.position.x > curTarget.Pos.x-maxDev && curPos.position.x < curTarget.Pos.x+maxDev &&
       curPos.position.y > curTarget.Pos.y-maxDev && curPos.position.y < curTarget.Pos.y+maxDev ){

       if(targetNodes.data.size()==0){
            result_.finalPosition.position.x = curPos.position.x;
            result_.finalPosition.position.y = curPos.position.y;
            result_.finalPosition.position.z = curPos.position.z;
            as_.setSucceeded(result_);

       }
       else{
           curTarget = targetNodes.data.front();
           PVAControl.publish(curTarget);
           targetNodes.data.erase(targetNodes.data.begin());
           ROS_INFO("Publishing next node");
       }

    }
    /*
    if(data_count_ > goal_)
    {
      result_.mean = feedback_.mean;
      result_.std_dev = feedback_.std_dev;

      if(result_.mean < 5.0)
      {
        ROS_INFO("%s: Aborted", action_name_.c_str());
        //set the action state to aborted
        as_.setAborted(result_);
      }
      else
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
    }
    */
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
  float maxDev = .15;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movingQuad");
  std::cout<<"Initialized node"<<std::endl;
  moveQuadAction mover(ros::this_node::getName());
  ros::spin();

  return 0;
}
