; Auto-generated. Do not edit!


(cl:in-package feetech_controls-msg)


;//! \htmlinclude jointcontrols.msg.html

(cl:defclass <jointcontrols> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (jointvalues
    :reader jointvalues
    :initarg :jointvalues
    :type cl:float
    :initform 0.0))
)

(cl:defclass jointcontrols (<jointcontrols>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <jointcontrols>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'jointcontrols)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feetech_controls-msg:<jointcontrols> is deprecated: use feetech_controls-msg:jointcontrols instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <jointcontrols>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-msg:id-val is deprecated.  Use feetech_controls-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'jointvalues-val :lambda-list '(m))
(cl:defmethod jointvalues-val ((m <jointcontrols>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-msg:jointvalues-val is deprecated.  Use feetech_controls-msg:jointvalues instead.")
  (jointvalues m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <jointcontrols>) ostream)
  "Serializes a message object of type '<jointcontrols>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'jointvalues))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <jointcontrols>) istream)
  "Deserializes a message object of type '<jointcontrols>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jointvalues) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<jointcontrols>)))
  "Returns string type for a message object of type '<jointcontrols>"
  "feetech_controls/jointcontrols")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'jointcontrols)))
  "Returns string type for a message object of type 'jointcontrols"
  "feetech_controls/jointcontrols")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<jointcontrols>)))
  "Returns md5sum for a message object of type '<jointcontrols>"
  "7988f738ee37fb27a094c2d2252a41df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'jointcontrols)))
  "Returns md5sum for a message object of type 'jointcontrols"
  "7988f738ee37fb27a094c2d2252a41df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<jointcontrols>)))
  "Returns full string definition for message of type '<jointcontrols>"
  (cl:format cl:nil "int16 id~%float64 jointvalues~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'jointcontrols)))
  "Returns full string definition for message of type 'jointcontrols"
  (cl:format cl:nil "int16 id~%float64 jointvalues~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <jointcontrols>))
  (cl:+ 0
     2
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <jointcontrols>))
  "Converts a ROS message object to a list"
  (cl:list 'jointcontrols
    (cl:cons ':id (id msg))
    (cl:cons ':jointvalues (jointvalues msg))
))
