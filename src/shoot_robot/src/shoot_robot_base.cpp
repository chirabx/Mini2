#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::ServiceClient shoot_client;
std_srvs::Empty empty_srv;


void Move2goal(MoveBaseClient& ac, double x, double y, double yaw)
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

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 1 Reached Successfully!!!");
        system("roslaunch shoot_robot shoot_tag_1.launch");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shoot_robot_base");
    ros::NodeHandle nh;

    ros::ServiceClient shoot_close_client;
    std_srvs::Empty empty_srv;
    shoot_client = nh.serviceClient<std_srvs::Empty>("close");
    
    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    Move2goal(ac, 2.44, 0.76, 0.7);//G
    shoot_client.call(empty_srv);
    Move2goal(ac, 2.42, -0.006, -0.86);//H
    shoot_client.call(empty_srv);
    Move2goal(ac, 1.63, 0.017, -2.36);//I
    shoot_client.call(empty_srv);
    Move2goal(ac, 1.67, 2.39, 2.31);//D
    shoot_client.call(empty_srv);
    Move2goal(ac, 2.48, 2.33, 0.84);//E
    shoot_client.call(empty_srv);
    Move2goal(ac, 2.41, 1.48, -0.498);//F
    shoot_client.call(empty_srv);
    Move2goal(ac, 0.14, 1.58, -2.31);//A
    shoot_client.call(empty_srv);
    Move2goal(ac, 0.19, 2.47, 2.64);//B
    shoot_client.call(empty_srv);
    Move2goal(ac, 1.00, 2.39, 0.94);//C
    shoot_client.call(empty_srv);
    
    move_base_msgs::MoveBaseGoal goal3;
    goal3.target_pose.pose.position.x = 0.0;
    goal3.target_pose.pose.position.y = 0.0;
    goal3.target_pose.pose.orientation.z = 0.0;
    goal3.target_pose.pose.orientation.w = 1.0;
    goal3.target_pose.header.frame_id = "map";
    goal3.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal3);
    ROS_INFO("Send Goal Home !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Back !!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    return 0;

}