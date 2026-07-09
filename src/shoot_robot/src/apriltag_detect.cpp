#include <ros/ros.h>
// 严格保留初始代码的头文件
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

class AprilTagController
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber tag_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient shoot_client;

    // 保留初始代码特有的控制参数
    const double Kp = 5;                    // 比例系数
    const double target_x_tolerance = 0.03;    // X轴位置容忍误差
    const double z_target_distance = 0.145;
    const double target_z_tolerance = 0.04;

    bool should_exit_ = false;

    // 引入参考代码中的后退保护机制变量
    bool is_backing_up_ = false;
    ros::Time backup_start_time_;
    const double backup_duration_ = 2.0; // 后退持续时间（秒）

    std_srvs::Empty empty_srv;

    // 在参数中加载要射击的tag目标
    int tag_id;

public:
    AprilTagController() : private_nh_("~")
    {
        // 初始化订阅者和发布者
        tag_sub_ = nh_.subscribe("tag_detections", 1, &AprilTagController::tagCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        shoot_client = nh_.serviceClient<std_srvs::Empty>("/shoot");

        private_nh_.getParam("tag", tag_id);
    }

    // 维持初始代码的 apriltags2_ros 消息类型
    void tagCallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        geometry_msgs::Twist cmd_vel;
        bool target_found = false;

        for (const auto &detection : msg->detections)
        {
            if (detection.id[0] == tag_id)
            {
                ROS_INFO(" The value of tag is %d .", tag_id);
                double current_x = detection.pose.pose.pose.position.x;
                double current_z = detection.pose.pose.pose.position.z;
        
                if ((fabs(current_x) < target_x_tolerance) && (fabs(current_z - z_target_distance) < target_z_tolerance))
                {
                    // 对齐成功，先停车并触发射击
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel);
                    shoot_client.call(empty_srv);

                    ros::Rate loop_rate(10);
                    int count = 0;

                    // 使用参考代码的循环机制替换原有的 sleep 阻塞
                    // 动作1：转向 -0.3 持续 1秒 (10 * 100ms)
                    cmd_vel.angular.z = -0.3;
                    count = 0;
                    while (ros::ok() && count < 10)
                    {
                        cmd_vel_pub_.publish(cmd_vel);
                        loop_rate.sleep();
                        count++;
                    }

                    // 动作2：转向 0.4 持续 1秒
                    cmd_vel.angular.z = 0.4;
                    count = 0;
                    while (ros::ok() && count < 10)
                    {
                        cmd_vel_pub_.publish(cmd_vel);
                        loop_rate.sleep();
                        count++;
                    }

                    // 动作3：停车退出
                    cmd_vel.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel);
                    
                    should_exit_ = true;
                    // 加入参考代码中的退出状态参数
                    ros::param::set("/apriltag_exit_status", "normal_exit");
                    ros::shutdown(); // 终止ROS通信
                    return;          // 直接退出回调函数
                }
                else if (fabs(current_x) > target_x_tolerance)
                {
                    ROS_INFO(" turn current_x = %f", current_x);
                    // 维持初始代码的角速度运算逻辑
                    cmd_vel.angular.z = 0.12 * (current_x / fabs(current_x));
                }
                else if (fabs(current_z - z_target_distance) > target_z_tolerance)
                {
                    // 维持初始代码的线速度运算逻辑
                    cmd_vel.linear.x = Kp * 0.2 * (current_z - z_target_distance);
                }
                target_found = true;
                break;
            }
        }
    
        if (!target_found)
        {
            // 如果还没有开始后退，记录开始时间
            if (!is_backing_up_)
            {
                is_backing_up_ = true;
                backup_start_time_ = ros::Time::now();
                ROS_INFO("Starting backup, target tag not detected");
            }

            // 检查是否已经后退足够时间
            ros::Duration backup_elapsed = ros::Time::now() - backup_start_time_;
            if (backup_elapsed.toSec() >= backup_duration_)
            {
                ROS_INFO("Backup time reached, executing next task");
                executeNextTask();
                ros::param::set("/apriltag_exit_status", "unnormal_exit");
                return;
            }

            // 继续后退
            ROS_INFO("Backing up... %.1f seconds elapsed", backup_elapsed.toSec());
            cmd_vel.linear.x = -0.05;
            cmd_vel.angular.z = 0;
        }
        else
        {
            // 如果检测到目标，重置后退状态
            if (is_backing_up_)
            {
                is_backing_up_ = false;
                ROS_INFO("Target detected, stopping backup");
            }
        }

        // 统一发布控制指令
        cmd_vel_pub_.publish(cmd_vel);
    }

    // 补充参考代码中的后续异常处理函数
    void executeNextTask()
    {
        ROS_INFO("Executing next task...");

        // 停止机器人运动
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0;
        stop_cmd.angular.z = 0;
        cmd_vel_pub_.publish(stop_cmd);

        should_exit_ = true;
        ros::shutdown();

        ROS_INFO("Next task execution completed");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_controller");
    AprilTagController controller;
    ros::Rate loop_rate(10); // 控制循环频率（10Hz）
    while (ros::ok())
    {
        ros::spinOnce(); // 处理回调队列
        loop_rate.sleep();
    }
    // 退出前的清理工作（可选）
    ROS_INFO("Node shutdown gracefully");
    return 0;
}