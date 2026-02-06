; Auto-generated. Do not edit!


(cl:in-package darknet_ros_msgs-msg)


;//! \htmlinclude DetectionPoseList.msg.html

(cl:defclass <DetectionPoseList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image_header
    :reader image_header
    :initarg :image_header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detectionposelist
    :reader detectionposelist
    :initarg :detectionposelist
    :type (cl:vector darknet_ros_msgs-msg:DetectionPose)
   :initform (cl:make-array 0 :element-type 'darknet_ros_msgs-msg:DetectionPose :initial-element (cl:make-instance 'darknet_ros_msgs-msg:DetectionPose))))
)

(cl:defclass DetectionPoseList (<DetectionPoseList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectionPoseList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectionPoseList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name darknet_ros_msgs-msg:<DetectionPoseList> is deprecated: use darknet_ros_msgs-msg:DetectionPoseList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DetectionPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:header-val is deprecated.  Use darknet_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_header-val :lambda-list '(m))
(cl:defmethod image_header-val ((m <DetectionPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:image_header-val is deprecated.  Use darknet_ros_msgs-msg:image_header instead.")
  (image_header m))

(cl:ensure-generic-function 'detectionposelist-val :lambda-list '(m))
(cl:defmethod detectionposelist-val ((m <DetectionPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader darknet_ros_msgs-msg:detectionposelist-val is deprecated.  Use darknet_ros_msgs-msg:detectionposelist instead.")
  (detectionposelist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectionPoseList>) ostream)
  "Serializes a message object of type '<DetectionPoseList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image_header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detectionposelist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detectionposelist))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectionPoseList>) istream)
  "Deserializes a message object of type '<DetectionPoseList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image_header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detectionposelist) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detectionposelist)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'darknet_ros_msgs-msg:DetectionPose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectionPoseList>)))
  "Returns string type for a message object of type '<DetectionPoseList>"
  "darknet_ros_msgs/DetectionPoseList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectionPoseList)))
  "Returns string type for a message object of type 'DetectionPoseList"
  "darknet_ros_msgs/DetectionPoseList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectionPoseList>)))
  "Returns md5sum for a message object of type '<DetectionPoseList>"
  "7f6fc00ec917fb60a5053a8b4b47bca8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectionPoseList)))
  "Returns md5sum for a message object of type 'DetectionPoseList"
  "7f6fc00ec917fb60a5053a8b4b47bca8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectionPoseList>)))
  "Returns full string definition for message of type '<DetectionPoseList>"
  (cl:format cl:nil "Header header~%Header image_header~%DetectionPose[] detectionposelist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: darknet_ros_msgs/DetectionPose~%float64 x~%float64 y~%float64 z~%int16 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectionPoseList)))
  "Returns full string definition for message of type 'DetectionPoseList"
  (cl:format cl:nil "Header header~%Header image_header~%DetectionPose[] detectionposelist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: darknet_ros_msgs/DetectionPose~%float64 x~%float64 y~%float64 z~%int16 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectionPoseList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image_header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detectionposelist) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectionPoseList>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectionPoseList
    (cl:cons ':header (header msg))
    (cl:cons ':image_header (image_header msg))
    (cl:cons ':detectionposelist (detectionposelist msg))
))
