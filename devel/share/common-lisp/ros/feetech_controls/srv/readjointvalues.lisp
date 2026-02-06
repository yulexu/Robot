; Auto-generated. Do not edit!


(cl:in-package feetech_controls-srv)


;//! \htmlinclude readjointvalues-request.msg.html

(cl:defclass <readjointvalues-request> (roslisp-msg-protocol:ros-message)
  ((idstart
    :reader idstart
    :initarg :idstart
    :type cl:integer
    :initform 0)
   (idend
    :reader idend
    :initarg :idend
    :type cl:integer
    :initform 0))
)

(cl:defclass readjointvalues-request (<readjointvalues-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <readjointvalues-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'readjointvalues-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feetech_controls-srv:<readjointvalues-request> is deprecated: use feetech_controls-srv:readjointvalues-request instead.")))

(cl:ensure-generic-function 'idstart-val :lambda-list '(m))
(cl:defmethod idstart-val ((m <readjointvalues-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-srv:idstart-val is deprecated.  Use feetech_controls-srv:idstart instead.")
  (idstart m))

(cl:ensure-generic-function 'idend-val :lambda-list '(m))
(cl:defmethod idend-val ((m <readjointvalues-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-srv:idend-val is deprecated.  Use feetech_controls-srv:idend instead.")
  (idend m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <readjointvalues-request>) ostream)
  "Serializes a message object of type '<readjointvalues-request>"
  (cl:let* ((signed (cl:slot-value msg 'idstart)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'idend)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <readjointvalues-request>) istream)
  "Deserializes a message object of type '<readjointvalues-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'idstart) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'idend) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<readjointvalues-request>)))
  "Returns string type for a service object of type '<readjointvalues-request>"
  "feetech_controls/readjointvaluesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'readjointvalues-request)))
  "Returns string type for a service object of type 'readjointvalues-request"
  "feetech_controls/readjointvaluesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<readjointvalues-request>)))
  "Returns md5sum for a message object of type '<readjointvalues-request>"
  "21a5fa9eeca5ec0f0f497e63ffbb5aa1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'readjointvalues-request)))
  "Returns md5sum for a message object of type 'readjointvalues-request"
  "21a5fa9eeca5ec0f0f497e63ffbb5aa1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<readjointvalues-request>)))
  "Returns full string definition for message of type '<readjointvalues-request>"
  (cl:format cl:nil "int64 idstart~%int64 idend~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'readjointvalues-request)))
  "Returns full string definition for message of type 'readjointvalues-request"
  (cl:format cl:nil "int64 idstart~%int64 idend~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <readjointvalues-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <readjointvalues-request>))
  "Converts a ROS message object to a list"
  (cl:list 'readjointvalues-request
    (cl:cons ':idstart (idstart msg))
    (cl:cons ':idend (idend msg))
))
;//! \htmlinclude readjointvalues-response.msg.html

(cl:defclass <readjointvalues-response> (roslisp-msg-protocol:ros-message)
  ((jointreadvalues
    :reader jointreadvalues
    :initarg :jointreadvalues
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass readjointvalues-response (<readjointvalues-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <readjointvalues-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'readjointvalues-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feetech_controls-srv:<readjointvalues-response> is deprecated: use feetech_controls-srv:readjointvalues-response instead.")))

(cl:ensure-generic-function 'jointreadvalues-val :lambda-list '(m))
(cl:defmethod jointreadvalues-val ((m <readjointvalues-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-srv:jointreadvalues-val is deprecated.  Use feetech_controls-srv:jointreadvalues instead.")
  (jointreadvalues m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <readjointvalues-response>) ostream)
  "Serializes a message object of type '<readjointvalues-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'jointreadvalues))))
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
   (cl:slot-value msg 'jointreadvalues))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <readjointvalues-response>) istream)
  "Deserializes a message object of type '<readjointvalues-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'jointreadvalues) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'jointreadvalues)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<readjointvalues-response>)))
  "Returns string type for a service object of type '<readjointvalues-response>"
  "feetech_controls/readjointvaluesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'readjointvalues-response)))
  "Returns string type for a service object of type 'readjointvalues-response"
  "feetech_controls/readjointvaluesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<readjointvalues-response>)))
  "Returns md5sum for a message object of type '<readjointvalues-response>"
  "21a5fa9eeca5ec0f0f497e63ffbb5aa1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'readjointvalues-response)))
  "Returns md5sum for a message object of type 'readjointvalues-response"
  "21a5fa9eeca5ec0f0f497e63ffbb5aa1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<readjointvalues-response>)))
  "Returns full string definition for message of type '<readjointvalues-response>"
  (cl:format cl:nil "float64[] jointreadvalues~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'readjointvalues-response)))
  "Returns full string definition for message of type 'readjointvalues-response"
  (cl:format cl:nil "float64[] jointreadvalues~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <readjointvalues-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'jointreadvalues) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <readjointvalues-response>))
  "Converts a ROS message object to a list"
  (cl:list 'readjointvalues-response
    (cl:cons ':jointreadvalues (jointreadvalues msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'readjointvalues)))
  'readjointvalues-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'readjointvalues)))
  'readjointvalues-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'readjointvalues)))
  "Returns string type for a service object of type '<readjointvalues>"
  "feetech_controls/readjointvalues")