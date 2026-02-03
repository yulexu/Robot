#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相对位置导航脚本（_xu 版本）
在终端输入相对于机器人当前位置的目标位置，机器人将导航到该位置
"""

import rospy
import tf2_ros
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid

class RelativeNavigator:
    def __init__(self):
        rospy.init_node('navigate_relative_xu', anonymous=True)
        
        # 获取参数
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
      
        # TF 监听器（用于获取机器人当前位置）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 或者使用 odom 话题（备用方案）
        self.current_pose = None
        self.odom_sub = rospy.Subscriber('/zed2i/zed_node/odom', Odometry, self.odom_callback)
        
        # 目标发布器
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        
        rospy.loginfo("相对位置导航节点已启动")
        rospy.loginfo("坐标系: %s -> %s" % (self.base_frame, self.odom_frame))
        rospy.loginfo("等待 TF 或 odom 数据...")
        
        # 等待 TF 或 odom 可用
        rospy.sleep(1.0)
        
    def odom_callback(self, msg):
        """从 odom 获取当前位置"""
        self.current_pose = msg.pose.pose
        
    def get_current_pose(self):
        """获取机器人当前位置（优先使用 TF，备用 odom）"""
        try:
            # 尝试从 TF 获取
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5))
            
            pose = PoseStamped()
            pose.header.frame_id = self.odom_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF 获取失败，尝试使用 odom: %s" % str(e))
            
            # 备用：使用 odom
            if self.current_pose is not None:
                return self.current_pose
            else:
                rospy.logerr("无法获取机器人当前位置！")
                return None
    
    def euler_to_quaternion(self, yaw):
        """将欧拉角（yaw）转换为四元数"""
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q
    
    def quaternion_to_yaw(self, q):
        """从四元数提取 yaw 角"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def send_goal(self, rel_x, rel_y, rel_yaw):
        """发送相对目标位置"""
        # 获取当前位置
        current_pose = self.get_current_pose()
        if current_pose is None:
            rospy.logerr("无法获取当前位置，取消发送目标")
            return False
        
        # 提取当前位置和朝向
        curr_x = current_pose.position.x
        curr_y = current_pose.position.y
        curr_yaw = self.quaternion_to_yaw(current_pose.orientation)
        
        rospy.loginfo("当前位置: x=%.2f, y=%.2f, yaw=%.2f°" % 
                     (curr_x, curr_y, math.degrees(curr_yaw)))
        
        # 计算目标位置（相对坐标转绝对坐标）
        # 相对坐标是在机器人坐标系中：x=前方，y=左侧，yaw=逆时针
        cos_yaw = math.cos(curr_yaw)
        sin_yaw = math.sin(curr_yaw)
        
        goal_x = curr_x + rel_x * cos_yaw - rel_y * sin_yaw
        goal_y = curr_y + rel_x * sin_yaw + rel_y * cos_yaw
        goal_yaw = curr_yaw + rel_yaw
        
        rospy.loginfo("目标位置: x=%.2f, y=%.2f, yaw=%.2f°" % 
                     (goal_x, goal_y, math.degrees(goal_yaw)))
        
        # 创建目标消息
        goal = PoseStamped()
        goal.header.frame_id = self.odom_frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation = self.euler_to_quaternion(goal_yaw)
        
        # 发布目标
        self.goal_pub.publish(goal)
        rospy.loginfo("已发送导航目标！")
        
        return True
    
    def run_interactive(self):
        """交互式运行"""
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("相对位置导航 - 使用说明")
        rospy.loginfo("="*50)
        rospy.loginfo("输入格式: x y yaw")
        rospy.loginfo("  x: 前方距离（米，正数=前进，负数=后退）")
        rospy.loginfo("  y: 左侧距离（米，正数=左移，负数=右移）")
        rospy.loginfo("  yaw: 目标朝向（弧度，正数=逆时针，负数=顺时针）")
        rospy.loginfo("示例: 1.0 0.5 0.0  (前进1米，左移0.5米，保持朝向)")
        rospy.loginfo("示例: 2.0 0.0 1.57  (前进2米，逆时针转90度)")
        rospy.loginfo("输入 'q' 或 'quit' 退出")
        rospy.loginfo("="*50 + "\n")
        
        while not rospy.is_shutdown():

            
                
            try:
        
                # Python 2/3 兼容
                try:
                    user_input = input("\n请输入目标位置 (x y yaw) 或 'q' 退出: ").strip()
                except NameError:
                    user_input = input("\n请输入目标位置 (x y yaw) 或 'q' 退出: ").strip()
                
                if user_input.lower() in ['q', 'quit', 'exit']:
                    rospy.loginfo("退出导航程序")
                    break
                
                if not user_input:
                    continue
                
           
                # 解析输入
                parts = user_input.split()
                if len(parts) != 3:
                    rospy.logwarn("输入格式错误！需要3个数字: x y yaw")
                    continue
                
                rel_x = float(parts[0])
                rel_y = float(parts[1])
                rel_yaw = float(parts[2])
                
                # 发送目标
                if self.send_goal(rel_x, rel_y, rel_yaw):
                    rospy.loginfo("等待机器人到达目标...")
                    # 可以在这里等待结果，但为了简单起见，让用户手动输入下一个目标
                else:
                    rospy.logwarn("发送目标失败，请重试")
                    
            except ValueError:
                rospy.logwarn("输入格式错误！请输入数字")
            except KeyboardInterrupt:
                rospy.loginfo("\n收到中断信号，退出...")
                break
            except Exception as e:
                rospy.logerr("发生错误: %s" % str(e))

def main():
    try:
        navigator = RelativeNavigator()
        navigator.run_interactive()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
