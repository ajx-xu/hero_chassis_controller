#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point

def jump():
    rospy.init_node('jump_node')
    rospy.loginfo("等待 Gazebo 服务...")
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

    # 注意：如果没反应，去 Gazebo 左侧看看底盘真正的名字叫什么
    body_name = "hero_robot::base_link"

    wrench = Wrench()
    wrench.force.z = 2000.0  # 向上的巨大力量 (2000 牛顿)
    wrench.torque.x = 0.0
    wrench.torque.y = 0.0
    wrench.torque.z = 0.0

    # 持续时间 0.1 秒，模拟瞬间的冲量
    duration = rospy.Duration(0.1)
    start_time = rospy.Time.now()

    try:
        rospy.loginfo("起跳！")
        # 这里加上 Point(0, 0, 0) 作为 reference_point
        apply_wrench(body_name, "world", Point(0, 0, 0), wrench, start_time, duration)
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s" % e)

if __name__ == '__main__':
    try:
        jump()
    except rospy.ROSInterruptException:
        pass