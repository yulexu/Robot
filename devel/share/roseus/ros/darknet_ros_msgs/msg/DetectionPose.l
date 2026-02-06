;; Auto-generated. Do not edit!


(when (boundp 'darknet_ros_msgs::DetectionPose)
  (if (not (find-package "DARKNET_ROS_MSGS"))
    (make-package "DARKNET_ROS_MSGS"))
  (shadow 'DetectionPose (find-package "DARKNET_ROS_MSGS")))
(unless (find-package "DARKNET_ROS_MSGS::DETECTIONPOSE")
  (make-package "DARKNET_ROS_MSGS::DETECTIONPOSE"))

(in-package "ROS")
;;//! \htmlinclude DetectionPose.msg.html


(defclass darknet_ros_msgs::DetectionPose
  :super ros::object
  :slots (_x _y _z _id ))

(defmethod darknet_ros_msgs::DetectionPose
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _id (round __id))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; int16 _id
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int16 _id
       (write-word _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get darknet_ros_msgs::DetectionPose :md5sum-) "2e292010fca4af7fb1a5badef13ed2a5")
(setf (get darknet_ros_msgs::DetectionPose :datatype-) "darknet_ros_msgs/DetectionPose")
(setf (get darknet_ros_msgs::DetectionPose :definition-)
      "float64 x
float64 y
float64 z
int16 id

")



(provide :darknet_ros_msgs/DetectionPose "2e292010fca4af7fb1a5badef13ed2a5")


