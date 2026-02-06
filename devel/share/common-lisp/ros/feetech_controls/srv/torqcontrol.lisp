; Auto-generated. Do not edit!


(cl:in-package feetech_controls-srv)


;//! \htmlinclude torqcontrol-request.msg.html

(cl:defclass <torqcontrol-request> (roslisp-msg-protocol:ros-message)
  ((idrec
    :reader idrec
    :initarg :idrec
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass torqcontrol-request (<torqcontrol-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <torqcontrol-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'torqcontrol-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feetech_controls-srv:<torqcontrol-request> is deprecated: use feetech_controls-srv:torqcontrol-request instead.")))

(cl:ensure-generic-function 'idrec-val :lambda-list '(m))
(cl:defmethod idrec-val ((m <torqcontrol-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-srv:idrec-val is deprecated.  Use feetech_controls-srv:idrec instead.")
  (idrec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <torqcontrol-request>) ostream)
  "Serializes a message object of type '<torqcontrol-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'idrec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'idrec))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <torqcontrol-request>) istream)
  "Deserializes a message object of type '<torqcontrol-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'idrec) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'idrec)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<torqcontrol-request>)))
  "Returns string type for a service object of type '<torqcontrol-request>"
  "feetech_controls/torqcontrolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'torqcontrol-request)))
  "Returns string type for a service object of type 'torqcontrol-request"
  "feetech_controls/torqcontrolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<torqcontrol-request>)))
  "Returns md5sum for a message object of type '<torqcontrol-request>"
  "c7fd7facee233a061827f86f586e5d13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'torqcontrol-request)))
  "Returns md5sum for a message object of type 'torqcontrol-request"
  "c7fd7facee233a061827f86f586e5d13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<torqcontrol-request>)))
  "Returns full string definition for message of type '<torqcontrol-request>"
  (cl:format cl:nil "int64[] idrec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'torqcontrol-request)))
  "Returns full string definition for message of type 'torqcontrol-request"
  (cl:format cl:nil "int64[] idrec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <torqcontrol-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'idrec) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <torqcontrol-request>))
  "Converts a ROS message object to a list"
  (cl:list 'torqcontrol-request
    (cl:cons ':idrec (idrec msg))
))
;//! \htmlinclude torqcontrol-response.msg.html

(cl:defclass <torqcontrol-response> (roslisp-msg-protocol:ros-message)
  ((idret
    :reader idret
    :initarg :idret
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass torqcontrol-response (<torqcontrol-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <torqcontrol-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'torqcontrol-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feetech_controls-srv:<torqcontrol-response> is deprecated: use feetech_controls-srv:torqcontrol-response instead.")))

(cl:ensure-generic-function 'idret-val :lambda-list '(m))
(cl:defmethod idret-val ((m <torqcontrol-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-srv:idret-val is deprecated.  Use feetech_controls-srv:idret instead.")
  (idret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <torqcontrol-response>) ostream)
  "Serializes a message object of type '<torqcontrol-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'idret))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'idret))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <torqcontrol-response>) istream)
  "Deserializes a message object of type '<torqcontrol-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'idret) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'idret)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<torqcontrol-response>)))
  "Returns string type for a service object of type '<torqcontrol-response>"
  "feetech_controls/torqcontrolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'torqcontrol-response)))
  "Returns string type for a service object of type 'torqcontrol-response"
  "feetech_controls/torqcontrolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<torqcontrol-response>)))
  "Returns md5sum for a message object of type '<torqcontrol-response>"
  "c7fd7facee233a061827f86f586e5d13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'torqcontrol-response)))
  "Returns md5sum for a message object of type 'torqcontrol-response"
  "c7fd7facee233a061827f86f586e5d13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<torqcontrol-response>)))
  "Returns full string definition for message of type '<torqcontrol-response>"
  (cl:format cl:nil "bool[] idret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'torqcontrol-response)))
  "Returns full string definition for message of type 'torqcontrol-response"
  (cl:format cl:nil "bool[] idret~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <torqcontrol-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'idret) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <torqcontrol-response>))
  "Converts a ROS message object to a list"
  (cl:list 'torqcontrol-response
    (cl:cons ':idret (idret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'torqcontrol)))
  'torqcontrol-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'torqcontrol)))
  'torqcontrol-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'torqcontrol)))
  "Returns string type for a service object of type '<torqcontrol>"
  "feetech_controls/torqcontrol")