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
void Move2goal(MoveBaseClient &ac, ros::Publisher &pub,double x, double y, double yaw, string tag_name);
void Move1goal(MoveBaseClient &ac, double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw, const std::string &tag_name);
void sleep(double second)
{
    ros::Duration(second).sleep();
}

// Retry logic function
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw, const std::string &tag_name)
{
    ros::NodeHandle nh;
    geometry_msgs::Twist vel_msg;
    int count = 0;
    ros::Rate loop_rate(10);

    ROS_INFO("Executing backward retry logic...");
    vel_msg.linear.x = -0.1;
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
    Move2goal(ac, pub, x, y, yaw, tag_name);
}

void Move_safe(ros::Publisher &pub, double linear_x, double linear_y, double distance)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = linear_x;
    vel_msg.linear.y = linear_y;
    int count = 0;
    ros::Rate loop_rate(10);
    while (ros::ok() && count < distance)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);
}

void SwingAndShoot(ros ::Publisher &pub)
{
    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate(10);
    ROS_INFO("Laser ON, starting swing...");
    // 参数
    const double swing_speed = 0.20;      // 角速度 rad/s
    const double swing_angle = 0.262;   // 20度 = π/6 弧度
    const int one_way_steps = (int)(swing_angle / swing_speed / 0.1);  // 约10步
    // 左摆20度
    vel_msg.angular.z = swing_speed;
    for (int i = 0; i < one_way_steps && ros::ok(); i++)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    // 右摆40度（从左20度 → 右20度）
    // vel_msg.angular.z = -swing_speed;
    // for (int i = 0; i < one_way_steps * 2 && ros::ok(); i++)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //停止
    vel_msg.angular.z = 0;
    pub.publish(vel_msg);
}

void Move2goal(MoveBaseClient &ac, ros ::Publisher &pub,double x, double y, double yaw, string tag_name)
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
        SwingAndShoot(pub);
        break;

    case actionlib::SimpleClientGoalState::ABORTED:
        ROS_WARN("Navigation aborted - possibly due to obstacles or path planning failure");
        performRetryLogic(ac, pub, x, y, yaw, tag_name);
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
    // 【修改】同时声明开启和关闭激光的服务客户端
    ros::ServiceClient shoot_close_client = nh.serviceClient<std_srvs::Empty>("/close");
    ros::ServiceClient shoot_open_client = nh.serviceClient<std_srvs::Empty>("/shoot");
    std_srvs::Empty empty_srv;
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    int count = 0;
    ros::Rate loop_rate(10);
    // 【修改】程序开始时，常开激光
    ros::service::waitForService("/shoot");
    shoot_open_client.call(empty_srv);
    ROS_INFO("Laser ON (Always on until return)");
    // sleep(0.5);

    // First target point
    Move2goal(ac, pub, 0.949, -0.950, -0.96, "1");//2.56, 0.84, 0.785

    Move2goal(ac, pub, 1.020, 1.403, 0.510, "1");//2.36, -0.014, -0.685

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

    // Third target point
    Move2goal(ac, pub, 0.151, 1.540, 2.180, "1");//1.581, 0.116, -2.355

    // Fourth target point
    Move2goal(ac, pub, 0.120, 0.718, -2.429, "1");//1.73, 2.47, 2.375

    vel_msg.linear.x = -0.10;
    count = 0;
    while (ros::ok() && count < 40)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    Move2goal(ac, pub, 1.500, 1.000, 0, "1");

    // Fifth target point
    Move2goal(ac, pub, 1.681, -0.860, -2.429, "1");
    // Sixth target point
    Move2goal(ac, pub, 2.451, -0.783, -0.96, "1");//2.39, 1.49, -0.685
    //Move2goal(ac, pub, 2.38, 1.51, -0.685, "1");//kongdi

    // Seventh target point
    Move2goal(ac, pub, 2.306, -0.153, 0.510, "1");

    vel_msg.linear.x = -0.10;
    count = 0;
    while (ros::ok() && count < 40)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    // Eighth target point
    Move2goal(ac, pub, 1.634, 1.514, 2.180, "1");//0.14, 2.45, 2.355
    // Move2goal(ac, pub, 0.14, 2.45, 2.355, "1");//kongdi

    // Ninth target point
    Move2goal(ac, pub, 2.550, 1.353, 0.710, "1");//0.93, 2.37, 0.785
    // Move2goal(ac, pub, 0.93, 2.33, 0.785, "1");//kongdi
    // 【修改】完成所有动作，返回起始点后，关闭激光
    ros::service::waitForService("/close");
    shoot_close_client.call(empty_srv);
    ROS_INFO("Returned to start. Laser OFF.");
    return 0;
}