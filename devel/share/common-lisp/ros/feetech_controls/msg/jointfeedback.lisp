; Auto-generated. Do not edit!


(cl:in-package feetech_controls-msg)


;//! \htmlinclude jointfeedback.msg.html

(cl:defclass <jointfeedback> (roslisp-msg-protocol:ros-message)
  ((jointvalues
    :reader jointvalues
    :initarg :jointvalues
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass jointfeedback (<jointfeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <jointfeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'jointfeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feetech_controls-msg:<jointfeedback> is deprecated: use feetech_controls-msg:jointfeedback instead.")))

(cl:ensure-generic-function 'jointvalues-val :lambda-list '(m))
(cl:defmethod jointvalues-val ((m <jointfeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-msg:jointvalues-val is deprecated.  Use feetech_controls-msg:jointvalues instead.")
  (jointvalues m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <jointfeedback>) ostream)
  "Serializes a message object of type '<jointfeedback>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'jointvalues))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'jointvalues))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <jointfeedback>) istream)
  "Deserializes a message object of type '<jointfeedback>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'jointvalues) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'jointvalues)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<jointfeedback>)))
  "Returns string type for a message object of type '<jointfeedback>"
  "feetech_controls/jointfeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'jointfeedback)))
  "Returns string type for a message object of type 'jointfeedback"
  "feetech_controls/jointfeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<jointfeedback>)))
  "Returns md5sum for a message object of type '<jointfeedback>"
  "a9d4993eb244bba482f1be1b903382b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'jointfeedback)))
  "Returns md5sum for a message object of type 'jointfeedback"
  "a9d4993eb244bba482f1be1b903382b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<jointfeedback>)))
  "Returns full string definition for message of type '<jointfeedback>"
  (cl:format cl:nil "float64[] jointvalues~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'jointfeedback)))
  "Returns full string definition for message of type 'jointfeedback"
  (cl:format cl:nil "float64[] jointvalues~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <jointfeedback>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'jointvalues) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <jointfeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'jointfeedback
    (cl:cons ':jointvalues (jointvalues msg))
))
