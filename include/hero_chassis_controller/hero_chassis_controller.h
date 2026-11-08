#pragma once

#include <ros/ros.h>
#include "hero_chassis_controller/wheel.h"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace hero_chassis_controller {

    class HeroChassisController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        HeroChassisController() = default;
        ~HeroChassisController() override = default;

        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
        void starting(const ros::Time& time) override;
        void update(const ros::Time& time, const ros::Duration& period) override;
        void stopping(const ros::Time& time) override;

    private:
        ros::NodeHandle controller_nh_;
        // 硬件接口 (被替换为 Wheel 对象)
        std::vector<Wheel> wheels_;

        // 订阅者与速度指令
        ros::Subscriber cmd_vel_sub_;
        geometry_msgs::Twist cmd_vel_;

        // 运动学参数
        double wheel_radius_;
        double wheel_separation_x_;
        double wheel_separation_y_;

        // 里程计相关
        ros::Publisher odom_pub_;
        tf::TransformBroadcaster tf_broadcaster_;
        double x_, y_, yaw_;

        // TF 与 模式切换
        tf::TransformListener tf_listener_;
        bool use_world_frame_;

        void cmdVelCallback(const geometry_msgs::Twist& msg);
    };

} // namespace hero_chassis_controller