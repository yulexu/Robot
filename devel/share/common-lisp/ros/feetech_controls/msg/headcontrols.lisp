; Auto-generated. Do not edit!


(cl:in-package feetech_controls-msg)


;//! \htmlinclude headcontrols.msg.html

(cl:defclass <headcontrols> (roslisp-msg-protocol:ros-message)
  ((arucoid
    :reader arucoid
    :initarg :arucoid
    :type cl:fixnum
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (headjointvalues1
    :reader headjointvalues1
    :initarg :headjointvalues1
    :type cl:float
    :initform 0.0)
   (headjointvalues2
    :reader headjointvalues2
    :initarg :headjointvalues2
    :type cl:float
    :initform 0.0))
)

(cl:defclass headcontrols (<headcontrols>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <headcontrols>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'headcontrols)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name feetech_controls-msg:<headcontrols> is deprecated: use feetech_controls-msg:headcontrols instead.")))

(cl:ensure-generic-function 'arucoid-val :lambda-list '(m))
(cl:defmethod arucoid-val ((m <headcontrols>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-msg:arucoid-val is deprecated.  Use feetech_controls-msg:arucoid instead.")
  (arucoid m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <headcontrols>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-msg:type-val is deprecated.  Use feetech_controls-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'headjointvalues1-val :lambda-list '(m))
(cl:defmethod headjointvalues1-val ((m <headcontrols>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-msg:headjointvalues1-val is deprecated.  Use feetech_controls-msg:headjointvalues1 instead.")
  (headjointvalues1 m))

(cl:ensure-generic-function 'headjointvalues2-val :lambda-list '(m))
(cl:defmethod headjointvalues2-val ((m <headcontrols>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader feetech_controls-msg:headjointvalues2-val is deprecated.  Use feetech_controls-msg:headjointvalues2 instead.")
  (headjointvalues2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <headcontrols>) ostream)
  "Serializes a message object of type '<headcontrols>"
  (cl:let* ((signed (cl:slot-value msg 'arucoid)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'headjointvalues1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'headjointvalues2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <headcontrols>) istream)
  "Deserializes a message object of type '<headcontrols>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arucoid) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'headjointvalues1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'headjointvalues2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<headcontrols>)))
  "Returns string type for a message object of type '<headcontrols>"
  "feetech_controls/headcontrols")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'headcontrols)))
  "Returns string type for a message object of type 'headcontrols"
  "feetech_controls/headcontrols")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<headcontrols>)))
  "Returns md5sum for a message object of type '<headcontrols>"
  "1bdf8924bc3daacda9a890695dd945ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'headcontrols)))
  "Returns md5sum for a message object of type 'headcontrols"
  "1bdf8924bc3daacda9a890695dd945ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<headcontrols>)))
  "Returns full string definition for message of type '<headcontrols>"
  (cl:format cl:nil "int16 arucoid~%int16 type~%float32 headjointvalues1~%float32 headjointvalues2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'headcontrols)))
  "Returns full string definition for message of type 'headcontrols"
  (cl:format cl:nil "int16 arucoid~%int16 type~%float32 headjointvalues1~%float32 headjointvalues2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <headcontrols>))
  (cl:+ 0
     2
     2
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <headcontrols>))
  "Converts a ROS message object to a list"
  (cl:list 'headcontrols
    (cl:cons ':arucoid (arucoid msg))
    (cl:cons ':type (type msg))
    (cl:cons ':headjointvalues1 (headjointvalues1 msg))
    (cl:cons ':headjointvalues2 (headjointvalues2 msg))
))
