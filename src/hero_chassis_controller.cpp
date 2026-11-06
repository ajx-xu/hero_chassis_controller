#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace hero_chassis_controller {

    bool HeroChassisController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
        std::vector<std::string> joint_names;
        if (!controller_nh.getParam("joints", joint_names)) {
            ROS_ERROR("Could not find joint names parameter");
            return false;
        }

        controller_nh.param("wheel_radius", wheel_radius_, 0.076);
        controller_nh.param("wheel_separation_x", wheel_separation_x_, 0.25);
        controller_nh.param("wheel_separation_y", wheel_separation_y_, 0.25);

        // 初始化轮子对象
        for (const auto& name : joint_names) {
            Wheel wheel;
            // 这里直接把 getHandle 传进去
            if (!wheel.init(hw->getHandle(name), controller_nh, name)) {
                return false;
            }
            wheels_.push_back(wheel);
        }

        cmd_vel_sub_ = controller_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
        ROS_INFO("HeroChassisController initialized with Wheel objects.");
        return true;
    }

    void HeroChassisController::starting(const ros::Time& time) {
        for (auto& wheel : wheels_) {
            wheel.stop();
        }
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.angular.z = 0.0;
    }

    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
        cmd_vel_ = msg;
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
        double vx = cmd_vel_.linear.x;
        double vy = cmd_vel_.linear.y;
        double wz = cmd_vel_.angular.z;

        double L = wheel_separation_x_ + wheel_separation_y_;

        // 计算四个轮子的期望速度
        double target_vel_0 = (vx - vy - wz * L) / wheel_radius_; // 左前
        double target_vel_1 = (vx + vy + wz * L) / wheel_radius_; // 右前
        double target_vel_2 = (vx + vy - wz * L) / wheel_radius_; // 左后
        double target_vel_3 = (vx - vy + wz * L) / wheel_radius_; // 右后

        // 分发给各个轮子对象，并让它们各自执行 PID
        if (wheels_.size() == 4) {
            wheels_[0].setTargetVelocity(target_vel_0);
            wheels_[1].setTargetVelocity(target_vel_1);
            wheels_[2].setTargetVelocity(target_vel_2);
            wheels_[3].setTargetVelocity(target_vel_3);
        }

        for (auto& wheel : wheels_) {
            wheel.update(period);
        }
    }

    void HeroChassisController::stopping(const ros::Time& time) {
        for (auto& wheel : wheels_) {
            wheel.stop();
        }
    }

} // namespace hero_chassis_controller

// 注册插件，让 ROS 能加载它
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)