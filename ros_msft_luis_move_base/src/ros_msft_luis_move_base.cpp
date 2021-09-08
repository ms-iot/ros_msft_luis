#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ros_msft_luis_msgs/Entity.h>
#include <ros_msft_luis_msgs/Intent.h>
#include <ros_msft_luis_msgs/TopIntent.h>

// TODO: Update these based on intent definitions
const std::string FORWARD = "move-forward";
const std::string BACKWARD = "move-backward";
const std::string RIGHT = "move-right";
const std::string LEFT = "move-left";

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

void intentCallback(const ros_msft_luis_msgs::TopIntent::ConstPtr& msg)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    // TODO: Update parsing of top intent based on grammer format and populate goal
    if(msg->topIntent == FORWARD)
    {
        // Populate goal
    }
    else if(msg->topIntent == BACKWARD)
    {
        // Populate goal
    }
    else if(msg->topIntent == BACKWARD)
    {
        // Populate goal
    }
    else if(msg->topIntent == BACKWARD)
    {
        // Populate goal
    }
    else
    {
        ROS_WARN("Unknown intent, no move base goal will be sent.");
        return;
    }

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Move base goal has been completed.");
    else
        ROS_WARN("Move base goal has failed.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_msft_luis_move_base");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // Subscribe to intent messages
    ros::Subscriber g_intent_sub = nh.subscribe("intent", 1000, intentCallback);

    // Move base action client
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while(ros::ok())
    {
        ros::spin();
    }

    nh.shutdown();
    return 0;
}