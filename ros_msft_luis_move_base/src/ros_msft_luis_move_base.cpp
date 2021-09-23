#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ros_msft_luis_msgs/Entity.h>
#include <ros_msft_luis_msgs/Intent.h>
#include <ros_msft_luis_msgs/TopIntent.h>

// TODO: Update these based on intent definitions
const std::string FORWARD = "move forward";
const std::string BACKWARD = "move backward";
const std::string RIGHT = "turn right";
const std::string LEFT = "turn left";

std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;

std::string str_tolower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), 
                   [](unsigned char c){ return std::tolower(c); }
                  );
    return s;
}

void intentCallback(const ros_msft_luis_msgs::TopIntent::ConstPtr& msg)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    auto intent = str_tolower(msg->topIntent);

    if (intent == FORWARD && msg->dimension.value != 0.0)
    {
        // TODO: handle feet vs meters
        goal.target_pose.pose.position.x = msg->dimension.value;
        goal.target_pose.pose.orientation.w = 1.0;
    }
    else if (intent == BACKWARD)
    {
        goal.target_pose.pose.position.x = -msg->dimension.value;
        goal.target_pose.pose.orientation.w = 1.0;
    }
    else if (intent == RIGHT)
    {
        // TODO: extract angle from intent
        // TODO: compute quaternion properly...

        // By default, turn 90 degrees
        goal.target_pose.pose.orientation.z = -0.7071068;
        goal.target_pose.pose.orientation.w = 0.7071068;
    }
    else if (intent == LEFT)
    {
        // TODO: extract angle from intent
        // TODO: compute quaternion properly...

        // By default, turn 90 degrees
        goal.target_pose.pose.orientation.z = 0.7071068;
        goal.target_pose.pose.orientation.w = 0.7071068;
    }
    else
    {
        ROS_WARN("Unknown intent, no move base goal will be sent.");
        return;
    }

    ROS_INFO("Sending goal");
    ac->sendGoal(goal);
    ac->waitForResult();

    if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
    ac = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);

    while (!ac->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while (ros::ok())
    {
        ros::spin();
    }

    nh.shutdown();
    return 0;
}
