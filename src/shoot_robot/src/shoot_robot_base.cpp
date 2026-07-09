#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// Function declarations
void Move2goal(MoveBaseClient &ac, double x, double y, double yaw, string tag_name);
void Move1goal(MoveBaseClient &ac, double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, double x, double y, double yaw, const std::string &tag_name);
void sleep(double second)
{
    ros::Duration(second).sleep();
}

// Retry logic function
void performRetryLogic(MoveBaseClient &ac, double x, double y, double yaw, const std::string &tag_name)
{
    ros::NodeHandle nh;
    geometry_msgs::Twist vel_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    int count = 0;
    ros::Rate loop_rate(10);

    ROS_INFO("Executing backward retry logic...");
    vel_msg.linear.x = -0.05;
    count = 0;
    while (ros::ok() && count < 10)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    ROS_INFO("Retrying to move to target point (%.3f, %.3f, %.3f)", x, y, yaw);
    Move2goal(ac, x, y, yaw, tag_name);
}

void Move2goal(MoveBaseClient &ac, double x, double y, double yaw, string tag_name)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();

    actionlib::SimpleClientGoalState state = ac.getState();

    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO("Target point %s (%.3f, %.3f, %.3f) reached successfully!", tag_name.c_str(), x, y, yaw);
        system(("roslaunch shoot_robot shoot_tag_" + tag_name + ".launch").c_str());
        break;

    case actionlib::SimpleClientGoalState::ABORTED:
        ROS_WARN("Navigation aborted - possibly due to obstacles or path planning failure");
        performRetryLogic(ac, x, y, yaw, tag_name);
        break;
    }
    // sleep(0.5);
}

void Move1goal(MoveBaseClient &ac, double x, double y, double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();
    // sleep(0.5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shoot_robot_base");
    ros::NodeHandle nh;

    geometry_msgs::Twist vel_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceClient shoot_close_client;
    std_srvs::Empty empty_srv;

    shoot_close_client = nh.serviceClient<std_srvs::Empty>("/close");
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    int count = 0;
    ros::Rate loop_rate(10);
    shoot_close_client.call(empty_srv);
    Move1goal(ac, 1.5, 1.1,0);

    // First target point
    Move2goal(ac, 2.44, 0.76, 0.785, "1");
    shoot_close_client.call(empty_srv);

    //Move1goal(ac, 0.877, 0.3, 1.57);

    // //Second target point
    Move2goal(ac, 2.44, -0.01, -0.785, "1");
    shoot_close_client.call(empty_srv);

    // vel_msg.linear.x = -0.05;
    // count = 0;
    // while (ros::ok() && count < 20)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // // Stop
    // vel_msg.linear.x = 0.0;
    // pub.publish(vel_msg);

    // //Third target point
    Move2goal(ac, 1.63, 0.01, -2.355, "1");
    shoot_close_client.call(empty_srv);

    // Fourth target point
    Move2goal(ac, 1.67, 2.39, 2.355, "1");
    shoot_close_client.call(empty_srv);

    // vel_msg.linear.x = -0.05;
    // count = 0;
    // while (ros::ok() && count < 10)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // // Stop
    // vel_msg.linear.x = 0.0;
    // pub.publish(vel_msg);

    // Move1goal(ac, 1.100, 0.400, 0);

    // Fifth target point
    Move2goal(ac, 2.48, 2.36, 0.785, "1");
    shoot_close_client.call(empty_srv);

    // vel_msg.linear.x = -0.05;
    // count = 0;
    // while (ros::ok() && count < 20)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // // Stop
    // vel_msg.linear.x = 0.0;
    // pub.publish(vel_msg);

    // Sixth target point
    Move2goal(ac, 2.46, 1.45, -0.785, "1");
    shoot_close_client.call(empty_srv);

    // Seventh target point
    Move2goal(ac, 0.12, 1.58, -2.355, "1");
    shoot_close_client.call(empty_srv);

    // vel_msg.linear.x = -0.05;
    // count = 0;
    // while (ros::ok() && count < 30)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // // Stop
    // vel_msg.linear.x = 0.0;
    // pub.publish(vel_msg);

    // Eighth target point
    Move2goal(ac, 0.14, 2.47, 2.355, "1");
    shoot_close_client.call(empty_srv);

    // nineth target point
    Move2goal(ac, 0.94, 2.39, 0.785, "3");
    shoot_close_client.call(empty_srv);

    Move2goal(ac, 0, 0, 0,"1");

    return 0;
}
