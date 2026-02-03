#!/usr/bin/env python3

"""
ROS node to track objects using SORT TRACKER and YOLOv3 detector (darknet_ros)
Takes detected bounding boxes from darknet_ros and uses them to calculate tracked bounding boxes
Tracked objects and their ID are published to the sort_track node
No delay here
"""

import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from sort.sort import Sort  
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sort_track.msg import TrackedObject, TrackedObjects

detections = []
track = []
trackers = []

def get_parameters():
    # """
    # Gets the necessary parameters from .yaml file
    # Returns tuple
    # """
    camera_topic = rospy.get_param("~camera_topic")
    detection_topic = rospy.get_param("~detection_topic")
    tracker_topic = rospy.get_param('~tracker_topic')
    cost_threshold = rospy.get_param('~cost_threshold')
    min_hits = rospy.get_param('~min_hits')
    max_age = rospy.get_param('~max_age')
    return (camera_topic, detection_topic, tracker_topic, cost_threshold, max_age, min_hits)

def callback_det(data):
    global detections, trackers, track, tracker, msg
    detections = []
    for box in data.bounding_boxes:
        detections.append(np.array([box.xmin, box.ymin, box.xmax, box.ymax, round(box.probability, 2)]))
    detections = np.array(detections)
    # Call the tracker
    trackers = tracker.update(detections)
    trackers = np.array(trackers, dtype='int')
    track = trackers

    tracked_objects_msg = TrackedObjects()
    tracked_objects_msg.header = data.header
    tracked_objects_msg.image_header = data.image_header
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
    pub_trackers.publish(tracked_objects_msg)
	
def callback_image(data):
    global detections, track
    bridge = CvBridge()
    cv_rgb = bridge.imgmsg_to_cv2(data, "bgr8")
    if len(detections) > 0:
        for detection in detections:
            cv2.rectangle(cv_rgb, (int(detection[0]), int(detection[1])), (int(detection[2]), int(detection[3])), (100, 255, 50), 1)
            cv2.putText(cv_rgb, "person", (int(detection[0]), int(detection[1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 255, 50), lineType=cv2.LINE_AA)

        # cv2.rectangle(cv_rgb, (int(detections[0][0]), int(detections[0][1])), (int(detections[0][2]), int(detections[0][3])), (100, 255, 50), 1)
        # cv2.putText(cv_rgb, "person", (int(detections[0][0]), int(detections[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 255, 50), lineType=cv2.LINE_AA)
    if len(track) > 0:
        for trk in track:
            cv2.rectangle(cv_rgb, (trk[0], trk[1]), (trk[2], trk[3]), (255, 255, 255), 1)
            cv2.putText(cv_rgb, str(trk[4]), (trk[2], trk[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)

        # cv2.rectangle(cv_rgb, (track[0][0], track[0][1]), (track[0][2], track[0][3]), (255, 255, 255), 1)
        # cv2.putText(cv_rgb, str(track[0][4]), (track[0][2], track[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)
    cv2.imshow("YOLO+SORT", cv_rgb)
    cv2.waitKey(3)

def main():
    global tracker, msg, pub_trackers
    rospy.init_node('sort_tracker', anonymous=False)
    rate = rospy.Rate(10)
    (camera_topic, detection_topic, tracker_topic, cost_threshold, max_age, min_hits) = get_parameters()
    tracker = Sort(max_age=max_age, min_hits=min_hits)  # create instance of the SORT tracker
    # Subscribe to image topic
    rospy.Subscriber(camera_topic, Image, callback_image)
    # Subscribe to darknet_ros to get BoundingBoxes from YOLOv3
    rospy.Subscriber(detection_topic, BoundingBoxes, callback_det)
    # Publish results of object tracking
    pub_trackers = rospy.Publisher(tracker_topic, TrackedObjects, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
