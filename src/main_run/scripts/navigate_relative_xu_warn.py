#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相对位置导航脚本（增强版）
功能：
1. 输入相对坐标进行导航
2. 实时监测局部代价地图，计算最近障碍物距离
"""

import rospy
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid

class RelativeNavigator:
    def __init__(self):
        rospy.init_node('navigate_relative_xu_warn', anonymous=True)
        
        # 获取参数
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        
        # 障碍物相关变量
        self.min_dist_to_obstacle = float('inf')
        self.obstacle_threshold = 60  # 代价地图阈值，超过此值视为障碍物 
        
        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 订阅器：里程计与代价地图
        self.odom_sub = rospy.Subscriber('/zed2i/zed_node/odom', Odometry, self.odom_callback)
        self.costmap_sub = rospy.Subscriber(
            "move_base/local_costmap/costmap", 
            OccupancyGrid, 
            self.costmap_callback
        )
        
        # 目标发布器
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        
        rospy.loginfo("相对位置导航增强版已启动")
        rospy.loginfo("正在监听 ZED 2i 深度点云与代价地图...")
        rospy.sleep(1.0)
        
    def costmap_callback(self, msg):
        """核心原理：将地图索引转换为物理距离 """
        res = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        data = np.array(msg.data).reshape((height, width))
        
        # 找到所有高于阈值的像素点索引
        occ_indices = np.where(data > self.obstacle_threshold)
        
        if len(occ_indices[0]) > 0:
            # 转换为物理坐标（相对于地图原点）
            # 由于是 rolling_window 模式，原点通常是地图左下角 
            x_coords = occ_indices[1] * res + origin_x
            y_coords = occ_indices[0] * res + origin_y
            
            # 计算到机器人中心（base_link 在局部地图中通常接近中心）的距离
            distances = np.sqrt(x_coords**2 + y_coords**2)
            self.min_dist_to_obstacle = np.min(distances)
        else:
            self.min_dist_to_obstacle = float('inf')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5))
            pose = PoseStamped()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation
            return pose.pose
        except Exception:
            return None
    
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def send_goal(self, rel_x, rel_y, rel_yaw):
        current_pose = self.get_current_pose()
        if current_pose is None: 
            rospy.logerr("获取位置失败")
            return False
        
        curr_yaw = self.quaternion_to_yaw(current_pose.orientation)
        
        # 坐标变换逻辑
        goal_x = current_pose.position.x + rel_x * math.cos(curr_yaw) - rel_y * math.sin(curr_yaw)
        goal_y = current_pose.position.y + rel_x * math.sin(curr_yaw) + rel_y * math.cos(curr_yaw)
        goal_yaw = curr_yaw + rel_yaw
        
        goal = PoseStamped()
        goal.header.frame_id = self.odom_frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        
        # 四元数转换
        goal.pose.orientation.w = math.cos(goal_yaw / 2.0)
        goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        
        self.goal_pub.publish(goal)
        return True
    
    def run_interactive(self):
        while not rospy.is_shutdown():
            # 实时显示距离信息
            dist_str = "安全" if self.min_dist_to_obstacle == float('inf') else "%.2fm" % self.min_dist_to_obstacle
            print("\r[实时感知] 最近障碍物距离: %s " % dist_str, end="")
            
            try:
                # 使用 input 可能会阻塞显示，但这里保留交互性
                user_input = input("\n输入目标 (x y yaw) 或 'q' 退出: ").strip()
                if user_input.lower() in ['q', 'quit']: break
                
                parts = user_input.split()
                if len(parts) == 3:
                    self.send_goal(float(parts[0]), float(parts[1]), float(parts[2]))
                    rospy.loginfo("目标已发送")
            except Exception as e:
                pass

if __name__ == '__main__':
    try:
        nav = RelativeNavigator()
        nav.run_interactive()
    except rospy.ROSInterruptException:
        pass