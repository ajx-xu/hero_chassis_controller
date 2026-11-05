#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    // 1. 获取关节名称列表 (从 yaml 文件读取)
    std::vector<std::string> joint_names;
    if (!controller_nh.getParam("joints", joint_names)) {
        ROS_ERROR("Could not find joint names parameter");
        return false;
    }

    // 2. 读取底盘运动学参数
    controller_nh.param("wheel_radius", wheel_radius_, 0.076);     // 默认值视实际模型而定
    controller_nh.param("wheel_separation_x", wheel_separation_x_, 0.25);
    controller_nh.param("wheel_separation_y", wheel_separation_y_, 0.25);

    // 3. 初始化关节接口和 PID
    for (const auto& name : joint_names) {
        joints_.push_back(hw->getHandle(name));
        control_toolbox::Pid pid;
        pid.init(ros::NodeHandle(controller_nh, name)); // 为每个轮子独立读取 PID 参数
        pids_.push_back(pid);
    }

    // 初始化目标速度数组
    wheel_velocities_.resize(joints_.size(), 0.0);

    // 4. 订阅速度指令话题
    cmd_vel_sub_ = controller_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

    ROS_INFO("HeroChassisController initialized successfully.");
    return true;
}

void HeroChassisController::starting(const ros::Time& time) {
    for (auto& pid : pids_) pid.reset();
    std::fill(wheel_velocities_.begin(), wheel_velocities_.end(), 0.0);
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
    cmd_vel_ = msg;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
    // 1. 获取底盘期望速度
    double vx = cmd_vel_.linear.x;
    double vy = cmd_vel_.linear.y;
    double wz = cmd_vel_.angular.z;

    // 2. 麦克纳姆轮逆运动学计算 (计算四个轮子的期望角速度)
    // 注意：这里的公式和正负号必须与你的 URDF 模型中轮子的安装方向完全匹配
    double L = wheel_separation_x_ + wheel_separation_y_;

    double target_vel_0 = (vx - vy - wz * L) / wheel_radius_; // 左前
    double target_vel_1 = (vx + vy + wz * L) / wheel_radius_; // 右前
    double target_vel_2 = (vx + vy - wz * L) / wheel_radius_; // 左后
    double target_vel_3 = (vx - vy + wz * L) / wheel_radius_; // 右后

    // 将计算结果存入目标速度数组
    if (joints_.size() == 4) {
        wheel_velocities_[0] = target_vel_0;
        wheel_velocities_[1] = target_vel_1;
        wheel_velocities_[2] = target_vel_2;
        wheel_velocities_[3] = target_vel_3;
    }

    // 3. 使用 PID 计算实际输出
    for (size_t i = 0; i < joints_.size(); ++i) {
        double current_vel = joints_[i].getVelocity(); // 当前实际速度
        double target_vel = wheel_velocities_[i];      // 期望速度

        // 计算误差并通过 PID 得到补偿量
        double pid_output = pids_[i].computeCommand(target_vel - current_vel, period);

        // 下发给硬件接口 (期望速度 + PID补偿)
        joints_[i].setCommand(current_vel + pid_output);
    }
}

void HeroChassisController::stopping(const ros::Time& time) {
    for (auto& joint : joints_) joint.setCommand(0.0);
}

} // namespace hero_chassis_controller

// 注册插件，让 ROS 能加载它
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)