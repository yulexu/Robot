#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from rfid.msg import rfid_msg
from std_msgs.msg import Header
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import DetectionPoseList

import math

before = 180
final = 0
a = 0

def track_callback(msg):
    global before, final, a
    rospy.loginfo("epc=%s, time=%010lu, idx=%02u, mode=%04u, ant=%01u, ph=%f, rssi=%f",
                  msg.epc, msg.time, msg.idx, msg.mode, msg.ant, msg.phase, msg.rssi)

    with open("rfid_data.txt", "a") as outfile:
        if msg.epc == "E280-1160-6000-0209-F811-5CE4" and msg.ant == 1:
            threshold = 180.0
            after = msg.phase
            diff = after - before

            if diff > threshold:
                a += 1
            elif diff < -threshold:
                a -= 1

            final = after - a * 360
            outfile.write(f"epc={msg.epc}, phase={final}, a={a}\n")

            before = msg.phase

def DetectionPoseList_callback(msg):
    with open("poselist.txt", "a") as detectionpose_outfile:
        # Write header information
        detectionpose_outfile.write(f"header_stamp={msg.header.stamp}, header_frame_id={msg.header.frame_id}, "
                                   f"image_header_stamp={msg.image_header.stamp}, image_header_frame_id={msg.image_header.frame_id}\n")

        # Write DetectionPoseList information
        for detection_pose in msg.detectionposelist:
            detectionpose_outfile.write(f"x={detection_pose.x}, y={detection_pose.y}, z={detection_pose.z}, id={detection_pose.id}\n")

def BoundingBoxes_callback(msg):
    with open("boundingboxes.txt", "a") as bbox_outfile:
        # Write header information
        bbox_outfile.write(f"header_stamp={msg.header.stamp}, header_frame_id={msg.header.frame_id}, "
                           f"image_header_stamp={msg.image_header.stamp}, image_header_frame_id={msg.image_header.frame_id}\n")

        # Write BoundingBox information
        for bbox in msg.bounding_boxes:
            bbox_outfile.write(f"probability={bbox.probability}, "
                               f"xmin={bbox.xmin}, ymin={bbox.ymin}, xmax={bbox.xmax}, ymax={bbox.ymax}, "
                               f"id={bbox.id}, Class={bbox.Class}\n")


def main():
    global before, final, a
    rospy.init_node("rfid_subscriber")    
    #rfid_subscriber = rospy.Subscriber("/rfid_message", rfid_msg, track_callback)
    #pose_subscriber = rospy.Subscriber("/darknet_ros/Detection_pose", DetectionPoseList, DetectionPoseList_callback)
    bounding_subscriber = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, BoundingBoxes_callback)

    rate = rospy.Rate(100)  # Set the rate to match the original C++ code

    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()

if __name__ == "__main__":
    main()
