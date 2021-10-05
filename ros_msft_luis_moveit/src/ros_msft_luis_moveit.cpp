#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <ros_msft_luis_msgs/Entity.h>
#include <ros_msft_luis_msgs/Intent.h>
#include <ros_msft_luis_msgs/TopIntent.h>

// TODO: move to parameter
static const std::string PLANNING_GROUP = "xarm7";

// TODO: Update these based on intent definitions
const std::string FORWARD = "move forward";
const std::string BACKWARD = "move backward";
const std::string RIGHT = "turn right";
const std::string LEFT = "turn left";
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
    else if (unit == "foot")
    {
        return value * 0.3048;
    }
    else if (unit == "yard")
    {
        return value * 0.9144;
    }

    // Default value if unit unknown
    return 0.0;
}

// Intent callback -- called when an intent message is received
void intentCallback(const ros_msft_luis_msgs::TopIntent::ConstPtr& msg)
{
    // TODO: implement stuff
    
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_msft_luis_moveit");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // Subscribe to intent messages
    ros::Subscriber g_intent_sub = nh.subscribe("intent", 1000, intentCallback);

    while (ros::ok())
    {
        ros::spin();
    }

    nh.shutdown();
    return 0;
}
