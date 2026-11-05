#pragma once
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.hpp>
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
        // 硬件接口
        std::vector<hardware_interface::JointHandle> joints_;

        // PID 控制器
        std::vector<control_toolbox::Pid> pids_;
        std::vector<double> wheel_velocities_;

        // 会用的参数
        ros::Subscriber cmd_vel_sub_;
        geometry_msgs::Twist cmd_vel_;

        // 运动学参数 (在 yaml 配置文件里读取)
        double wheel_radius_;
        double wheel_separation_x_; // 前后轮距的一半
        double wheel_separation_y_; // 左右轮距的一半

        // 回调函数声明
        void cmdVelCallback(const geometry_msgs::Twist& msg);
    };

} // namespace hero_chassis_controller