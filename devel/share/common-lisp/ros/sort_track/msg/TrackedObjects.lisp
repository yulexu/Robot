; Auto-generated. Do not edit!


(cl:in-package sort_track-msg)


;//! \htmlinclude TrackedObjects.msg.html

(cl:defclass <TrackedObjects> (roslisp-msg-protocol:ros-message)
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
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector sort_track-msg:TrackedObject)
   :initform (cl:make-array 0 :element-type 'sort_track-msg:TrackedObject :initial-element (cl:make-instance 'sort_track-msg:TrackedObject))))
)

(cl:defclass TrackedObjects (<TrackedObjects>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedObjects>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedObjects)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sort_track-msg:<TrackedObjects> is deprecated: use sort_track-msg:TrackedObjects instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackedObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sort_track-msg:header-val is deprecated.  Use sort_track-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_header-val :lambda-list '(m))
(cl:defmethod image_header-val ((m <TrackedObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sort_track-msg:image_header-val is deprecated.  Use sort_track-msg:image_header instead.")
  (image_header m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <TrackedObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sort_track-msg:objects-val is deprecated.  Use sort_track-msg:objects instead.")
  (objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedObjects>) ostream)
  "Serializes a message object of type '<TrackedObjects>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image_header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedObjects>) istream)
  "Deserializes a message object of type '<TrackedObjects>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image_header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sort_track-msg:TrackedObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedObjects>)))
  "Returns string type for a message object of type '<TrackedObjects>"
  "sort_track/TrackedObjects")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedObjects)))
  "Returns string type for a message object of type 'TrackedObjects"
  "sort_track/TrackedObjects")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedObjects>)))
  "Returns md5sum for a message object of type '<TrackedObjects>"
  "a44e334ae2403a52c52d73709b5b3461")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedObjects)))
  "Returns md5sum for a message object of type 'TrackedObjects"
  "a44e334ae2403a52c52d73709b5b3461")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedObjects>)))
  "Returns full string definition for message of type '<TrackedObjects>"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Header image_header~%TrackedObject[] objects~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sort_track/TrackedObject~%int32 xmin~%int32 ymin~%int32 xmax~%int32 ymax~%int32 id~%int32 center_x~%int32 center_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedObjects)))
  "Returns full string definition for message of type 'TrackedObjects"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Header image_header~%TrackedObject[] objects~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sort_track/TrackedObject~%int32 xmin~%int32 ymin~%int32 xmax~%int32 ymax~%int32 id~%int32 center_x~%int32 center_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedObjects>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image_header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedObjects>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedObjects
    (cl:cons ':header (header msg))
    (cl:cons ':image_header (image_header msg))
    (cl:cons ':objects (objects msg))
))
