#include <ros/ros.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <cmath>

class AprilTagController
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber tag_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient shoot_client;

    // 沿用 202510 参考代码的控制参数
    const double Kp = 5.0;                     // 距离比例系数
    const double target_x_tolerance = 0.03;     // X轴位置容忍误差 (3 cm)
    const double z_target_distance = 0.145;      // 目标打靶距离 (14.5 cm)
    const double target_z_tolerance = 0.04;      // Z轴距离容忍误差 (4 cm)

    bool should_exit_ = false;

    // 未找到Tag时的后退搜寻保护机制
    bool is_backing_up_ = false;
    ros::Time backup_start_time_;
    const double backup_duration_ = 2.0;        // 后退搜寻持续时间（秒）

    std_srvs::Empty empty_srv;
    int tag_id;

public:
    AprilTagController() : private_nh_("~")
    {
        tag_sub_ = nh_.subscribe("tag_detections", 1, &AprilTagController::tagCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        shoot_client = nh_.serviceClient<std_srvs::Empty>("/shoot");

        private_nh_.getParam("tag", tag_id);
    }

    void tagCallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        geometry_msgs::Twist cmd_vel;
        bool target_found = false;

        for (const auto &detection : msg->detections)
        {
            if (detection.id[0] == tag_id)
            {
                target_found = true;
                double current_x = detection.pose.pose.pose.position.x;
                double current_z = detection.pose.pose.pose.position.z;

                double err_x = current_x;
                double err_z = current_z - z_target_distance;

                ROS_INFO("Tag %d detected | err_x: %.3f m, err_z: %.3f m", tag_id, err_x, err_z);

                // =========================================================================
                // 动态闭环微调逻辑：角度没对准优先旋角 -> 对准后前后移动 -> 偏了随时重新旋角校准
                // =========================================================================

                // 条件 1：优先旋转调整 X 轴方向（无论处于何种阶段，只要角度偏离就立刻暂停前后移动，重新旋转对准）
                if (std::fabs(err_x) >= target_x_tolerance)
                {
                    ROS_INFO("Aligning X (rotation): err_x = %f", err_x);
                    cmd_vel.linear.x = 0.0; // 暂停前后移动
                    cmd_vel.angular.z = 0.12 * (err_x / std::fabs(err_x)); // 旋转对准
                }
                // 条件 2：角度已经正对，但 Z 轴前后距离还没到，进行前后微调
                else if (std::fabs(err_z) >= target_z_tolerance)
                {
                    ROS_INFO("Aligning Z (distance): err_z = %f", err_z);
                    cmd_vel.angular.z = 0.0; // 暂停旋转
                    cmd_vel.linear.x = Kp * 0.2 * err_z; // 前后移动
                }
                // 条件 3：角度与前后距离【同时】满足对准要求 -> 停稳、开激光打靶 2 秒并退出节点
                else
                {
                    // 1. 完全停稳小车
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_pub_.publish(cmd_vel);

                    // 2. 打开激光
                    if (shoot_client.exists())
                    {
                        shoot_client.call(empty_srv);
                    }
                    ROS_INFO("Target fully aligned in both Angle & Distance! Laser ON, holding for 2 seconds...");

                    // 3. 原地停靠 2 秒 (20次 * 100ms = 2秒)
                    ros::Rate hold_rate(10);
                    int hold_count = 0;
                    while (ros::ok() && hold_count < 20)
                    {
                        cmd_vel_pub_.publish(cmd_vel); // 持续发送零速度指令
                        hold_rate.sleep();
                        hold_count++;
                    }

                    // 4. 设置正常退出状态并关闭节点
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_pub_.publish(cmd_vel);

                    should_exit_ = true;
                    ros::param::set("/apriltag_exit_status", "normal_exit");
                    ROS_INFO("Shooting completed. Exiting apriltag node...");

                    ros::shutdown(); // 终止 ROS 通信，退出节点
                    return;
                }

                break; // 找到对应 Tag 结束循环
            }
        }

        // Tag 未检测到时的后退搜寻逻辑
        if (!target_found)
        {
            if (!is_backing_up_)
            {
                is_backing_up_ = true;
                backup_start_time_ = ros::Time::now();
                ROS_INFO("Target tag not detected, starting backup...");
            }

            ros::Duration backup_elapsed = ros::Time::now() - backup_start_time_;
            if (backup_elapsed.toSec() >= backup_duration_)
            {
                ROS_INFO("Backup timeout reached (2.0s). Exiting task...");
                executeNextTask();
                ros::param::set("/apriltag_exit_status", "unnormal_exit");
                return;
            }

            ROS_INFO("Backing up to search tag... %.1fs elapsed", backup_elapsed.toSec());
            cmd_vel.linear.x = -0.05;
            cmd_vel.angular.z = 0.0;
        }
        else
        {
            if (is_backing_up_)
            {
                is_backing_up_ = false;
                ROS_INFO("Target re-detected, stopping backup.");
            }
        }

        cmd_vel_pub_.publish(cmd_vel);
    }

    void executeNextTask()
    {
        ROS_INFO("Executing next task...");
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(stop_cmd);

        ros::shutdown();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_controller");
    AprilTagController controller;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Node shutdown gracefully");
    return 0;
}