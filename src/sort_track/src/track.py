#!/usr/bin/env python3

import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from sort.sort import Sort  
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image, CameraInfo
from sort_track.msg import TrackedObject, TrackedObjects
from darknet_ros_msgs.msg import DetectionPoseList, DetectionPose

# 用于存储深度图像和相机内参
depth_image = None
camera_matrix = None

detections = []
track = []
trackers = []

def get_parameters():
    camera_topic = rospy.get_param("~camera_topic")
    depth_camera_topic = rospy.get_param("~depth_camera_topic")
    camera_info_topic = rospy.get_param("~camera_info_topic")
    detection_topic = rospy.get_param("~detection_topic")
    tracker_topic = rospy.get_param('~tracker_topic')
    pose_topic = rospy.get_param('~pose_topic')
    cost_threshold = rospy.get_param('~cost_threshold')
    min_hits = rospy.get_param('~min_hits')
    max_age = rospy.get_param('~max_age')
    return (camera_topic, depth_camera_topic, camera_info_topic, detection_topic, tracker_topic, pose_topic, cost_threshold, max_age, min_hits)

def pixel_to_world(x, y, depth):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    depth_meters = depth / 1000.0 

    # X = (x - cx) * depth_meters / fx
    # Y = (y - cy) * depth_meters / fy
    # Z = depth_meters
    X = depth_meters
    Y = -((x - cx) * depth_meters / fx)
    Z = -((y - cy) * depth_meters / fy)

    #  X-axis extends outward along the direction the camera is facing, the Y-axis extends to the left, and the Z-axis extends vertically upward. This coordinate system adheres to the right-hand rule.
    return np.array([X, Y, Z])

def callback_det(data):
    global detections, trackers, track, tracker, msg, depth_image, camera_matrix
    if camera_matrix is None or depth_image is None:
        return

    detections = []
    for box in data.bounding_boxes:
        detections.append(np.array([box.xmin, box.ymin, box.xmax, box.ymax, round(box.probability, 2)]))
    detections = np.array(detections)
    # Call the tracker
    trackers = tracker.update(detections)
    trackers = np.array(trackers, dtype='int')
    track = trackers
    # track includes 2D info of each tracking obj
    tracked_objects_msg = TrackedObjects()
    tracked_objects_msg.header = data.header
    tracked_objects_msg.image_header = data.image_header
    # detection includes 3D info of tracking obj
    detection_poses_msg = DetectionPoseList()
    detection_poses_msg.header = data.header
    detection_poses_msg.image_header = data.image_header
    detection_poses_msg.detectionposelist = []

    for t in track:
        tracked_obj = TrackedObject()
        tracked_obj.xmin = t[0]
        tracked_obj.ymin = t[1]
        tracked_obj.xmax = t[2]
        tracked_obj.ymax = t[3]
        tracked_obj.id = t[4]
        tracked_obj.center_x = (t[0] + t[2]) // 2
        tracked_obj.center_y = (t[1] + t[3]) // 2
        tracked_objects_msg.objects.append(tracked_obj)

        # 获取深度信息并转换为真实世界坐标
        if depth_image is not None:
            depth = depth_image[tracked_obj.center_y, tracked_obj.center_x]
            world_coords = pixel_to_world(tracked_obj.center_x, tracked_obj.center_y, depth)
            detection_pose = DetectionPose()
            detection_pose.id = tracked_obj.id
            detection_pose.x = world_coords[0]
            detection_pose.y = world_coords[1]
            detection_pose.z = world_coords[2]
            detection_poses_msg.detectionposelist.append(detection_pose)

    pub_trackers.publish(tracked_objects_msg)
    pub_pose.publish(detection_poses_msg)

def callback_image(data):
    global detections, track
    bridge = CvBridge()
    cv_rgb = bridge.imgmsg_to_cv2(data, "bgr8")
    if len(detections) > 0:
        for detection in detections:
            cv2.rectangle(cv_rgb, (int(detection[0]), int(detection[1])), (int(detection[2]), int(detection[3])), (100, 255, 50), 1)
            cv2.putText(cv_rgb, "person", (int(detection[0]), int(detection[1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 255, 50), lineType=cv2.LINE_AA)

    if len(track) > 0:
        for trk in track:
            cv2.rectangle(cv_rgb, (trk[0], trk[1]), (trk[2], trk[3]), (255, 255, 255), 1)
            cv2.putText(cv_rgb, str(trk[4]), (trk[2], trk[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)

    cv2.imshow("YOLO+SORT", cv_rgb)
    cv2.waitKey(3)

def callback_depth_image(data):
    global depth_image
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")

def callback_camera_info(data):
    global camera_matrix
    camera_matrix = np.array(data.K).reshape(3, 3)

def main():
    global tracker, msg, pub_trackers, pub_pose
    rospy.init_node('sort_tracker', anonymous=False)
    rate = rospy.Rate(10)
    (camera_topic, depth_camera_topic, camera_info_topic, detection_topic, tracker_topic, pose_topic, cost_threshold, max_age, min_hits) = get_parameters()
    tracker = Sort(max_age=max_age, min_hits=min_hits)  # create instance of the SORT tracker
    # Subscribe to image topic
    rospy.Subscriber(camera_topic, Image, callback_image)
    # Subscribe to depth image topic
    rospy.Subscriber(depth_camera_topic, Image, callback_depth_image)
    # Subscribe to camera info topic
    rospy.Subscriber(camera_info_topic, CameraInfo, callback_camera_info)
    # Subscribe to darknet_ros to get BoundingBoxes from YOLOv3
    rospy.Subscriber(detection_topic, BoundingBoxes, callback_det)
    # Publish results of object tracking
    pub_trackers = rospy.Publisher(tracker_topic, TrackedObjects, queue_size=10)
    # Publish detection poses
    pub_pose = rospy.Publisher(pose_topic, DetectionPoseList, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
