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
            if (!wheel.init(hw->getHandle(name), controller_nh, name)) {
                return false;
            }
            wheels_.push_back(wheel);
        }

        cmd_vel_sub_ = controller_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

        // 初始化里程计 (移到 return true 之前)
        x_ = 0.0; y_ = 0.0; yaw_ = 0.0;
        odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 50);

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

        // =========== 选做题 6：正运动学里程计 ===========
        if (wheels_.size() == 4) {
            // 1. 获取四个轮子的实际速度
            double actual_v0 = wheels_[0].getVelocity(); // 左前
            double actual_v1 = wheels_[1].getVelocity(); // 右前
            double actual_v2 = wheels_[2].getVelocity(); // 左后
            double actual_v3 = wheels_[3].getVelocity(); // 右后

            // 2. 正运动学公式：根据轮速反推底盘速度
            // 注意：这里的 L 直接使用上面算好的，不要再加 double L = ... 了
            double vx_act = (actual_v0 + actual_v1 + actual_v2 + actual_v3) * wheel_radius_ / 4.0;
            double vy_act = (-actual_v0 + actual_v1 + actual_v2 - actual_v3) * wheel_radius_ / 4.0;
            double wz_act = (-actual_v0 + actual_v1 - actual_v2 + actual_v3) * wheel_radius_ / (4.0 * L);

            // 3. 积分累加得到坐标
            double dt = period.toSec();
            x_ += (vx_act * cos(yaw_) - vy_act * sin(yaw_)) * dt;
            y_ += (vx_act * sin(yaw_) + vy_act * cos(yaw_)) * dt;
            yaw_ += wz_act * dt;

            // 4. 组装并发布 Odometry 消息
            nav_msgs::Odometry odom;
            odom.header.stamp = time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.position.z = 0.0;
            tf::Quaternion q;
            q.setRPY(0, 0, yaw_);
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();

            odom.twist.twist.linear.x = vx_act;
            odom.twist.twist.linear.y = vy_act;
            odom.twist.twist.angular.z = wz_act;

            odom_pub_.publish(odom);

            // 5. 广播 TF 变换 (odom -> base_link)
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(x_, y_, 0.0));
            transform.setRotation(q);
            tf_broadcaster_.sendTransform(tf::StampedTransform(transform, time, "odom", "base_link"));
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