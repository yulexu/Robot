#!/usr/bin/python3

import cv2
import rospy
import pyrealsense2 as rs
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from gesture_recognition import *
from cvfpscalc import CvFpsCalc
from cv_bridge import CvBridge, CvBridgeError
from ros_hand_gesture_recognition.msg import HandGesture  # 导入自定义消息类型

class HandSignRecognition:

    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('hand_sign_recognition', anonymous=True)

        # 获取相机内参（可以通过RealSense SDK获取）
        self.fx = rospy.get_param("camera_fx", 615.0)
        self.fy = rospy.get_param("camera_fy", 615.0)
        self.cx = rospy.get_param("camera_cx", 320.0)
        self.cy = rospy.get_param("camera_cy", 240.0)

        self.camera_matrix = None  # 初始化相机矩阵

        # 订阅图像话题
        self.image_subscriber = rospy.Subscriber(rospy.get_param("hand_sign_recognition/subscribe_image_topic"), 
                                                 Image, self.callback)
        self.camera_info_subscriber = rospy.Subscriber(rospy.get_param("hand_sign_recognition/subscribe_camera_info_topic"),
                                                       CameraInfo, self.callback_camera_info)

        # 发布器
        self.gesture_publisher = rospy.Publisher(rospy.get_param("hand_sign_recognition/publish_gesture_topic"), 
                                                 HandGesture, queue_size=10)  # 更改为HandGesture类型

        # 创建手势识别对象，加载标签和训练模型
        self.gesture_detector = GestureRecognition(rospy.get_param("hand_sign_recognition/keypoint_classifier_label"),
                                                   rospy.get_param("hand_sign_recognition/keypoint_classifier_model"))
        self.bridge = CvBridge()
        self.cv_fps_calc = CvFpsCalc(buffer_len=10)

    def callback_camera_info(self, data):
        self.camera_matrix = np.array(data.K).reshape(3, 3)

    def pixel_to_world(self, u, v):
        if self.camera_matrix is None:
            rospy.logwarn("Camera matrix not received yet")
            return None, None

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        X = (u - cx) / fx
        Y = -(v - cy) / fy

        return X, Y

    def callback(self, image_msg):
        """图像订阅者的回调函数

        Args:
            image_msg (sensor_msgs.msg.Image): 图像消息
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv_image = cv2.flip(cv_image, 1) 
            debug_image, gesture, position = self.gesture_detector.recognize(cv_image)  # 获取手势和位置

            hand_gesture_msg = HandGesture()
            hand_gesture_msg.header.stamp = rospy.Time.now()
            hand_gesture_msg.header.frame_id = "hand_gesture"
            hand_gesture_msg.gesture_name = gesture

            x, y = self.pixel_to_world(position[0], position[1])  # 将像素坐标转换为真实世界坐标
            if x is not None and y is not None:
                hand_gesture_msg.pose.position.x = x
                hand_gesture_msg.pose.position.y = y
                hand_gesture_msg.pose.position.z = 0  # 设置Z轴为0
                hand_gesture_msg.pose.orientation.w = 1  # 根据需要设置四元数
                self.gesture_publisher.publish(hand_gesture_msg)

            if rospy.get_param("hand_sign_recognition/show_image"):
                fps = self.cv_fps_calc.get()
                debug_image = self.gesture_detector.draw_fps_info(debug_image, fps)
                cv2.imshow('ROS Gesture Recognition', debug_image)
                cv2.waitKey(1)  # 等待10毫秒
        except CvBridgeError as error:
            rospy.logerr(error)
        except Exception as e:
            rospy.logerr(e)

if __name__ == "__main__":
    try:
        hand_sign = HandSignRecognition()
        rospy.spin()
    # 如果按下Ctrl+C，节点将停止。
    except rospy.ROSInternalException:
        cv2.destroyAllWindows()
        pass
