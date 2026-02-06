;; Auto-generated. Do not edit!


(when (boundp 'ros_hand_gesture_recognition::HandGesture)
  (if (not (find-package "ROS_HAND_GESTURE_RECOGNITION"))
    (make-package "ROS_HAND_GESTURE_RECOGNITION"))
  (shadow 'HandGesture (find-package "ROS_HAND_GESTURE_RECOGNITION")))
(unless (find-package "ROS_HAND_GESTURE_RECOGNITION::HANDGESTURE")
  (make-package "ROS_HAND_GESTURE_RECOGNITION::HANDGESTURE"))

(in-package "ROS")
;;//! \htmlinclude HandGesture.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ros_hand_gesture_recognition::HandGesture
  :super ros::object
  :slots (_header _gesture_name _pose ))

(defmethod ros_hand_gesture_recognition::HandGesture
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:gesture_name __gesture_name) "")
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _gesture_name (string __gesture_name))
   (setq _pose __pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:gesture_name
   (&optional __gesture_name)
   (if __gesture_name (setq _gesture_name __gesture_name)) _gesture_name)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _gesture_name
    4 (length _gesture_name)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _gesture_name
       (write-long (length _gesture_name) s) (princ _gesture_name s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _gesture_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gesture_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get ros_hand_gesture_recognition::HandGesture :md5sum-) "e2a74601111d83781469252b75af91b3")
(setf (get ros_hand_gesture_recognition::HandGesture :datatype-) "ros_hand_gesture_recognition/HandGesture")
(setf (get ros_hand_gesture_recognition::HandGesture :definition-)
      "std_msgs/Header header
string gesture_name
geometry_msgs/Pose pose


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :ros_hand_gesture_recognition/HandGesture "e2a74601111d83781469252b75af91b3")


