#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

    bool HeroChassisController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
        // 1. 获取关节名称列表 (对应 URDF 里的 4 个轮子关节名)
        std::vector<std::string> joint_names;
        if (!controller_nh.getParam("joints", joint_names)) {
            ROS_ERROR("Could not find joint names parameter");
            return false;
        }

        // 2. 初始化关节接口和 PID
        for (const auto& name : joint_names) {
            joints_.push_back(hw->getHandle(name));
            control_toolbox::Pid pid;
            pid.init(ros::NodeHandle(controller_nh, name)); // 每个轮子独立读取 PID 参数
            pids_.push_back(pid);
        }
        wheel_velocities_.resize(joints_.size(), 0.0);

        ROS_INFO("HeroChassisController initialized successfully.");
        return true;
    }

    void HeroChassisController::starting(const ros::Time& time) {
        // 复位 PID 和 初始速度
        for (auto& pid : pids_) pid.reset();
        std::fill(wheel_velocities_.begin(), wheel_velocities_.end(), 0.0);
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {

        for (size_t i = 0; i < joints_.size(); ++i) {
            double current_vel = joints_[i].getVelocity(); // 实际速度
            double target_vel = wheel_velocities_[i];      // 期望速度

            // 使用 PID 计算期望输出
            double effort = pids_[i].computeCommand(target_vel - current_vel, period);
            joints_[i].setCommand(effort);
        }
    }

    void HeroChassisController::stopping(const ros::Time& time) {
        for (auto& joint : joints_) joint.setCommand(0.0);
    }

} // namespace hero_chassis_controller

// 注册插件
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
// Created by ajx on 2026/11/3.
//
