#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

#include <ros_msft_luis_msgs/Entity.h>
#include <ros_msft_luis_msgs/Intent.h>
#include <ros_msft_luis_msgs/TopIntent.h>

// TODO: Update these based on intent definitions
const std::string FORWARD = "move forward";
const std::string BACKWARD = "move backward";
const std::string RIGHT = "turn right";
const std::string LEFT = "turn left";
const std::string STOP = "stop";

std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;

std::string str_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), 
                   [](unsigned char c){ return std::tolower(c); }
                  );
    return s;
}

// Goal callback -- called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

// Goal callback -- called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// Goal callback -- called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    //ROS_INFO("Feedback received");
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
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    auto intent = str_tolower(msg->topIntent);

    if ((intent == FORWARD || intent == BACKWARD) && msg->dimension.value != 0.0)
    {
        float value_meters = to_meters(msg->dimension.value, msg->dimension.unit);

        if (value_meters == 0.0)
        {
            ROS_WARN("Could not convert units to meters, no goal will be sent");
            return;
        }

        if (intent == BACKWARD)
            value_meters = -value_meters;

        goal.target_pose.pose.position.x = value_meters;
        goal.target_pose.pose.orientation.w = 1.0;
    }
    else if (intent == RIGHT || intent == LEFT)
    {
        float angle;

        if (msg->entities.empty())
        {
            // By default, turn 90 degrees
            angle = 90.0;
        }
        else
        {
            auto entity = msg->entities.at(0);
            try
            {
                angle = stof(entity.value);
            }
            catch (std::exception& ex)
            {
                ROS_WARN("Could not convert angle to float, no goal will be sent");
                return;
            }
        }

        if (intent == RIGHT)
            angle = -angle;

        auto q = tf::createQuaternionMsgFromYaw(angles::from_degrees(angle));
        goal.target_pose.pose.orientation.x = q.x;
        goal.target_pose.pose.orientation.y = q.y;
        goal.target_pose.pose.orientation.z = q.z;
        goal.target_pose.pose.orientation.w = q.w;
    }
    else if (intent == STOP)
    {
        ROS_INFO("Canceling all goals");
        ac->cancelAllGoals();
    }
    else
    {
        ROS_WARN("Unknown intent, no move base goal will be sent.");
        return;
    }

    if (intent != STOP)
    {
        // Cancel any pre-existing goal
        if (ac->getState() == actionlib::SimpleClientGoalState::PENDING ||
            ac->getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            ROS_INFO("Canceling current goal");
            ac->cancelGoal();
            ac->waitForResult();
        }

        ROS_INFO("Sending goal");
        ac->sendGoal(goal, doneCb, activeCb, feedbackCb);
    }
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
