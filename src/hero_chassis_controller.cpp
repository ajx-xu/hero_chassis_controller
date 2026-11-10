#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace hero_chassis_controller {

    bool HeroChassisController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
        controller_nh_ = controller_nh;
        std::vector<std::string> joint_names;
        if (!controller_nh.getParam("joints", joint_names)) {
            ROS_ERROR("Could not find joint names parameter");
            return false;
        }

        controller_nh.param("wheel_radius", wheel_radius_, 0.076);
        controller_nh.param("wheel_separation_x", wheel_separation_x_, 0.25);
        controller_nh.param("wheel_separation_y", wheel_separation_y_, 0.25);

        for (const auto& name : joint_names) {
            Wheel wheel;
            if (!wheel.init(hw->getHandle(name), controller_nh, name)) {
                return false;
            }
            wheels_.push_back(wheel);
        }

        cmd_vel_sub_ = controller_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

        x_ = 0.0; y_ = 0.0; yaw_ = 0.0;
        odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 50);

        // 读取初始模式
        controller_nh.param("use_world_frame", use_world_frame_, false);
        // 初始化动态调参服务器
        dynamic_server_.reset(new dynamic_reconfigure::Server<hero_chassis_controller::PIDConfig>(controller_nh));
        dynamic_cb_ = boost::bind(&HeroChassisController::dynamicParamCallback, this, _1, _2);
        dynamic_server_->setCallback(dynamic_cb_);
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
        // 获取底盘目标速度
        double vx = cmd_vel_.linear.x;
        double vy = cmd_vel_.linear.y;
        double wz = cmd_vel_.angular.z;

        // ========== 选做题 7：世界坐标速度转换 ==========
        // 这里每次循环都会判断 use_world_frame_ 的值
        controller_nh_.param("use_world_frame", use_world_frame_, false);
        if (use_world_frame_) {
            try {
                tf::StampedTransform transform;
                tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), transform);

                double roll, pitch, yaw;
                tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

                double vx_body = vx * cos(yaw) + vy * sin(yaw);
                double vy_body = -vx * sin(yaw) + vy * cos(yaw);

                vx = vx_body;
                vy = vy_body;
            } catch (tf::TransformException &ex) {
                ROS_WARN("TF Exception: %s", ex.what());
            }
        }

        double L = wheel_separation_x_ + wheel_separation_y_;

        double target_vel_0 = (vx - vy - wz * L) / wheel_radius_;
        double target_vel_1 = (vx + vy + wz * L) / wheel_radius_;
        double target_vel_2 = (vx + vy - wz * L) / wheel_radius_;
        double target_vel_3 = (vx - vy + wz * L) / wheel_radius_;

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
            double actual_v0 = wheels_[0].getVelocity();
            double actual_v1 = wheels_[1].getVelocity();
            double actual_v2 = wheels_[2].getVelocity();
            double actual_v3 = wheels_[3].getVelocity();

            double vx_act = (actual_v0 + actual_v1 + actual_v2 + actual_v3) * wheel_radius_ / 4.0;
            double vy_act = (-actual_v0 + actual_v1 + actual_v2 - actual_v3) * wheel_radius_ / 4.0;
            double wz_act = (-actual_v0 + actual_v1 - actual_v2 + actual_v3) * wheel_radius_ / (4.0 * L);

            double dt = period.toSec();
            x_ += (vx_act * cos(yaw_) - vy_act * sin(yaw_)) * dt;
            y_ += (vx_act * sin(yaw_) + vy_act * cos(yaw_)) * dt;
            yaw_ += wz_act * dt;

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
    void HeroChassisController::dynamicParamCallback(hero_chassis_controller::PIDConfig &config, uint32_t level) {
        if (wheels_.size() == 4) {
            // 这里需要你在 wheel.h 里加一个更新参数的接口
            wheels_[0].setPID(config.lf_p, config.lf_i, config.lf_d);
            wheels_[1].setPID(config.rf_p, config.rf_i, config.rf_d);
            ROS_INFO("PID Params Updated via rqt_reconfigure!");
        }
    }
} // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)