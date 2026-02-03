#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def odom_callback(msg):
    # 1. 从里程计消息中提取四元数姿态
    orientation_q = msg.pose.pose.orientation
    quaternion = [
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    ]
    
    # 2. 将四元数转换为欧拉角（弧度）
    # roll, pitch, yaw 分别对应翻滚、俯仰、偏航（朝向角）
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    
    # 3. 将弧度转换为角度，方便人类阅读
    yaw_degrees = math.degrees(yaw)
    
    # 4. 打印当前朝向信息
    # 0度通常代表 X 轴正方向（正前方）
    rospy.loginfo("当前机器人朝向 (Yaw): {:.2f} 度".format(yaw_degrees))

def monitor_heading():
    rospy.init_node('heading_monitor_node', anonymous=True)
    
    # 订阅你的 ZED 2i 里程计话题
    rospy.Subscriber("/zed2i/zed_node/odom", Odometry, odom_callback)
    
    rospy.loginfo("朝向监测已启动，请观察输出...")
    rospy.spin()

if __name__ == '__main__':
    try:
        monitor_heading()
    except rospy.ROSInterruptException:
        pass