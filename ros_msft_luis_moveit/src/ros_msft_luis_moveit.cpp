#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <ros_msft_luis_msgs/Entity.h>
#include <ros_msft_luis_msgs/Intent.h>
#include <ros_msft_luis_msgs/TopIntent.h>

// TODO: move to parameter
static const std::string PLANNING_GROUP = "xarm7";
static const std::string PLANNING_GROUP_GRIPPER = "xarm_gripper";

// TODO: Update these based on intent definitions
const std::string FORWARD = "move arm forward";
const std::string BACKWARD = "move arm backward";
const std::string UP = "move arm up";
const std::string DOWN = "move arm down";
const std::string OPEN = "open hand";
const std::string CLOSE = "close hand";
const std::string STOP = "stop";

std::string str_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), 
                   [](unsigned char c){ return std::tolower(c); }
                  );
    return s;
}

// Convert length to meters; unit is a singular form string, such as "meter", "foot".
float to_meters(float value, std::string unit)
{
    unit = str_tolower(unit);

    if (unit == "meter")
    {
        return value;
    }
    else if (unit == "centimeter")
    {
        return value * 0.01;
    }
    else if (unit == "inch")
    {
        return value * 0.0254;
    }
    else if (unit == "foot")
    {
        return value * 0.3048;
    }

    // Default value if unit unknown
    return 0.0;
}

// Intent callback -- called when an intent message is received
void intentCallback(const ros_msft_luis_msgs::TopIntent::ConstPtr& msg)
{
    auto intent = str_tolower(msg->topIntent);

    float value_meters;
    
    if (msg->dimension.value != 0.0)
    {
        value_meters = to_meters(msg->dimension.value, msg->dimension.unit);
        if (value_meters == 0.0)
        {
            ROS_WARN("Could not convert units to meters, no goal will be sent");
            return;
        }
    }
    else
    {
        // Default of 10 centimeters
        value_meters = 0.1;
    }

    // Target planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Display some information

    ROS_INFO("Available planning groups:");
    std::for_each(move_group.getJointModelGroupNames().begin(),
                  move_group.getJointModelGroupNames().end(),
                  [](const std::string s) { ROS_INFO("- %s", s.c_str()); }
    );

    ROS_INFO("Available named targets for current planning group:");
    std::for_each(move_group.getNamedTargets().begin(),
                  move_group.getNamedTargets().end(),
                  [](const std::string s) { ROS_INFO("- %s", s.c_str()); }
    );

    // Create new pose based on current one
    auto current_pose = move_group.getCurrentPose().pose;
    geometry_msgs::Pose target_pose = current_pose;

    if (intent == FORWARD)
    {
        target_pose.position.x += value_meters;
    }
    else if (intent == BACKWARD)
    {
        target_pose.position.x -= value_meters;
    }
    else if (intent == UP)
    {
        target_pose.position.z += value_meters;
    }
    else if (intent == DOWN)
    {
        target_pose.position.z -= value_meters;
    }
    else
    {
        ROS_WARN("Unknown intent, no MoveIt target will be set");
        return;
    }

    move_group.setPoseTarget(target_pose);

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan move_plan;
    bool success = (move_group.plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Plan result: %s", success ? "SUCCESSFUL" : "FAILED");

    // Execute
    if (success)
        move_group.move();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_msft_luis_moveit");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // Subscribe to intent messages
    ros::Subscriber g_intent_sub = nh.subscribe("intent", 1000, intentCallback);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    nh.shutdown();
    return 0;
}