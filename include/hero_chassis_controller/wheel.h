#pragma once

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

namespace hero_chassis_controller {

    class Wheel {
    public:
        Wheel() = default;

        // 初始化：绑定硬件接口，读取PID参数
        bool init(hardware_interface::JointHandle joint_handle,
                  const ros::NodeHandle& controller_nh,
                  const std::string& wheel_name) {
            joint_handle_ = joint_handle;

            // 初始化该轮子的 PID (从 yaml 文件读取该轮子的独立参数)
            if (!pid_.init(ros::NodeHandle(controller_nh, wheel_name))) {
                ROS_ERROR("Failed to initialize PID for %s", wheel_name.c_str());
                return false;
            }
            target_velocity_ = 0.0;
            return true;
        }

        // 设置期望速度
        void setTargetVelocity(double vel) {
            target_velocity_ = vel;
        }

        // 核心执行函数
        void update(const ros::Duration& period) {
            double current_vel = joint_handle_.getVelocity(); // 获取 Gazebo 反馈的实际速度
            // PID 计算补偿
            double pid_out = pid_.computeCommand(target_velocity_ - current_vel, period);
            // 下发给底层
            joint_handle_.setCommand(current_vel + pid_out);
        }

        void stop() {
            target_velocity_ = 0.0;
            joint_handle_.setCommand(0.0);
        }
        double getVelocity() const {
            return joint_handle_.getVelocity();
        }
        // 新增：用于动态更新 PID 参数
        // 新增：用于动态更新 PID 参数
        void setPID(double p, double i, double d) {
            // 强制重置 PID 参数（p, i, d, i_max, i_min）
            // 这里把 i_max 和 i_min 设为 1.0 和 -1.0，或者根据你需要调整
            pid_.setGains(p, i, d, 1.0, -1.0); 
        }
    private:
        hardware_interface::JointHandle joint_handle_;
        control_toolbox::Pid pid_;
        double target_velocity_ = 0.0;
    };

} // namespace hero_chassis_controller
