#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rostopic
import tf
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
import time

class NavDiagnose:
    def __init__(self):
        rospy.init_node('nav_diagnose', anonymous=True)
        
        self.cmd_topic = "/dynamixel_workbench/cmd_vel"
        self.tf_listener = tf.TransformListener()
        
        # Status tracking
        self.last_cmd_time = 0
        self.cmd_msg = None
        self.move_base_status = "UNKNOWN"
        
        # Subscribers
        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        
        print("=== Navigation Diagnostic Tool Started ===")
        print(f"Monitoring topic: {self.cmd_topic}")
        print("Waiting for messages and TF...")

    def cmd_callback(self, msg):
        self.last_cmd_time = time.time()
        self.cmd_msg = msg

    def status_callback(self, msg):
        if msg.status_list:
            # Get the latest status
            latest = msg.status_list[-1]
            status_map = {
                0: "PENDING",
                1: "ACTIVE",
                2: "PREEMPTED",
                3: "SUCCEEDED",
                4: "ABORTED",
                5: "REJECTED",
                6: "PREEMPTING",
                7: "RECALLING",
                8: "RECALLED",
                9: "LOST"
            }
            self.move_base_status = f"{status_map.get(latest.status, 'UNKNOWN')} (ID: {latest.goal_id.id})"
        else:
            self.move_base_status = "IDLE (No Goal)"

    def check_tf(self, parent, child):
        try:
            self.tf_listener.waitForTransform(parent, child, rospy.Time(0), rospy.Duration(0.5))
            (trans, rot) = self.tf_listener.lookupTransform(parent, child, rospy.Time(0))
            return True, "OK"
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            return False, str(e)

    def run(self):
        rate = rospy.Rate(1) # 1Hz report
        while not rospy.is_shutdown():
            print("\n----------------------------------------")
            cur_time = time.time()
            
            # 1. Check Move Base Status
            print(f"[MoveBase Status]: {self.move_base_status}")
            
            # 2. Check CMD_VEL
            time_since_cmd = cur_time - self.last_cmd_time
            if self.cmd_msg is None:
                print(f"[CMD_VEL]: NO MESSAGES RECEIVED yet on {self.cmd_topic}")
            elif time_since_cmd > 1.0:
                print(f"[CMD_VEL]: SILENT (Last msg {time_since_cmd:.1f}s ago)")
            else:
                lx = self.cmd_msg.linear.x
                az = self.cmd_msg.angular.z
                print(f"[CMD_VEL]: ACTIVE | Speed: Lin={lx:.2f}, Ang={az:.2f}")
                if abs(lx) < 0.001 and abs(az) < 0.001:
                    print("           WARNING: Command matches ZERO velocity. Robot is being told to stop.")

            # 3. TF Checks
            # Check Odom -> Base
            ok_odom, msg_odom = self.check_tf("odom", "base_link")
            print(f"[TF odom->base]: {'PASS' if ok_odom else 'FAIL'} | {msg_odom}")
            
            # Check Costmap Sensors
            # Based on params, we likely need this one:
            ok_cam, msg_cam = self.check_tf("base_link", "zed2i_left_camera_frame")
            print(f"[TF base->camera]: {'PASS' if ok_cam else 'FAIL'} | {msg_cam}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = NavDiagnose()
        node.run()
    except rospy.ROSInterruptException:
        pass
