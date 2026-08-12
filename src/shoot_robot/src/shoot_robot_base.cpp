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
void Move2goal(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw, string tag_name);
void Move1goal(MoveBaseClient &ac, double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw, const std::string &tag_name);
void Turn_safe_1(ros::Publisher &pub, double angular_z, double distance);
void StepShoot(ros::Publisher &pub);

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
    vel_msg.linear.x = -0.2;//0.05
    count = 0;
    while (ros::ok() && count < 15)
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

// 原地旋转一小段角度（你提供的函数）
void Turn_safe_1(ros::Publisher &pub, double angular_z, double distance)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = angular_z;
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
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);
}

// 分步射击：旋转一小角度 -> 停顿（射击） -> 再旋转 -> 再停顿 ...
// 分步射击：左转小角度 -> 停顿 -> 再左转 -> 停顿 ... 然后右转对应角度 -> 停顿 -> 再右转 -> 停顿 ...（回到起始朝向）
void StepShoot(ros::Publisher &pub)
{
    ROS_INFO("Step shoot: rotate left, pause, repeat; then rotate right, pause, repeat...");
    // ===== 参数可调 =====
    const double turn_speed   = 0.18;    // 旋转角速度 rad/s（正值=左转，负值=右转）
    const double step_angle   = 0.0875;  // 每次旋转角度（弧度），0.0875 rad ≈ 5°
    const int    repeat_times = 5;       // 左、右各旋转-停顿循环次数
    const double pause_sec    = 0.5;     // 每次停顿（射击）时间（秒）
    // =====================
    const int turn_steps = (int)(step_angle / turn_speed / 0.1);  // 每次旋转的控制步数（10Hz，每步0.1s）

    // 向左分步旋转射击
    ROS_INFO("Rotating LEFT...");
    for (int i = 0; i < repeat_times && ros::ok(); i++)
    {
        Turn_safe_1(pub, turn_speed, turn_steps);   // 左转一小角度（正角速度 = 逆时针 = 左）
        ros::Duration(pause_sec).sleep();           // 停顿，激光常开完成射击
    }

    // 向右分步旋转射击（转回起始朝向）
    ROS_INFO("Rotating RIGHT...");
    for (int i = 0; i < repeat_times && ros::ok(); i++)
    {
        Turn_safe_1(pub, -turn_speed, turn_steps);  // 右转一小角度（负角速度 = 顺时针 = 右）
        ros::Duration(pause_sec).sleep();           // 停顿，激光常开完成射击
    }
}


void Move2goal(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw, string tag_name)
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
        StepShoot(pub);   // 到达导航点后：分步旋转射击（转小角度→停顿→再转→再停顿）
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
    // 同时声明开启和关闭激光的服务客户端
    ros::ServiceClient shoot_close_client = nh.serviceClient<std_srvs::Empty>("/close");
    ros::ServiceClient shoot_open_client = nh.serviceClient<std_srvs::Empty>("/shoot");
    std_srvs::Empty empty_srv;
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    int count = 0;
    ros::Rate loop_rate(10);
    // 程序开始时，常开激光
    ros::service::waitForService("/shoot");
    shoot_open_client.call(empty_srv);
    ROS_INFO("Laser ON (Always on until return)");

    Move_safe(pub, 0.0, 0.45, 35);
    Move_safe(pub, 0.4, 0.0, 25);
    // Move_safe(pub,0.0,0.4,20);
    // sleep(0.5);

    Move1goal(ac, 1.4, 1.2, 0);

  // First target point
    Move2goal(ac, pub, 2.50, 0.80, 0.785, "1");

    // Second target point
    Move2goal(ac, pub, 2.35, -0.004, -0.785, "1");

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
    Move2goal(ac, pub, 1.584, 0.114, -2.355, "1");

    // Fourth target point
    Move2goal(ac, pub, 1.71, 2.47, 2.355, "1");

    // Fifth target point
    Move2goal(ac, pub, 2.51, 2.33, 0.785, "1");

    // Sixth target point
    Move2goal(ac, pub, 2.38, 1.51, -0.785, "1");

    Move1goal(ac, 1.40, 1.40, -3.14);
    sleep(0.5);

    // Seventh target point
    Move2goal(ac, pub, 0.10, 1.60, -2.355, "1");

    // Eighth target point
    Move2goal(ac, pub, 0.17, 2.45, 2.355, "1");

    // Ninth target point
    Move2goal(ac, pub, 0.93, 2.33, 0.785, "1");

    Move1goal(ac, 1.1, 1.0, -1.57);
    sleep(0.5);

    Move1goal(ac, 0.3, 0.3, 0);//(0.05,0.05,0)
    // Move_safe(pub,0.0,-0.4,15);
    // Move_safe(pub,-0.4,0.0,15);

    // 完成所有动作，返回起始点后，关闭激光
    ros::service::waitForService("/close");
    shoot_close_client.call(empty_srv);
    ROS_INFO("Returned to start. Laser OFF.");
    return 0;
}
