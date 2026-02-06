;; Auto-generated. Do not edit!


(when (boundp 'rfid::rfid_msg)
  (if (not (find-package "RFID"))
    (make-package "RFID"))
  (shadow 'rfid_msg (find-package "RFID")))
(unless (find-package "RFID::RFID_MSG")
  (make-package "RFID::RFID_MSG"))

(in-package "ROS")
;;//! \htmlinclude rfid_msg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rfid::rfid_msg
  :super ros::object
  :slots (_header _epc _time _idx _mode _ant _phase _rssi ))

(defmethod rfid::rfid_msg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:epc __epc) "")
    ((:time __time) 0)
    ((:idx __idx) 0)
    ((:mode __mode) 0)
    ((:ant __ant) 0)
    ((:phase __phase) 0.0)
    ((:rssi __rssi) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _epc (string __epc))
   (setq _time (round __time))
   (setq _idx (round __idx))
   (setq _mode (round __mode))
   (setq _ant (round __ant))
   (setq _phase (float __phase))
   (setq _rssi (float __rssi))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:epc
   (&optional __epc)
   (if __epc (setq _epc __epc)) _epc)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:idx
   (&optional __idx)
   (if __idx (setq _idx __idx)) _idx)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:ant
   (&optional __ant)
   (if __ant (setq _ant __ant)) _ant)
  (:phase
   (&optional __phase)
   (if __phase (setq _phase __phase)) _phase)
  (:rssi
   (&optional __rssi)
   (if __rssi (setq _rssi __rssi)) _rssi)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _epc
    4 (length _epc)
    ;; uint64 _time
    8
    ;; uint32 _idx
    4
    ;; uint32 _mode
    4
    ;; uint32 _ant
    4
    ;; float64 _phase
    8
    ;; float64 _rssi
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _epc
       (write-long (length _epc) s) (princ _epc s)
     ;; uint64 _time
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _time (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _time) (= (length (_time . bv)) 2)) ;; bignum
              (write-long (ash (elt (_time . bv) 0) 0) s)
              (write-long (ash (elt (_time . bv) 1) -1) s))
             ((and (class _time) (= (length (_time . bv)) 1)) ;; big1
              (write-long (elt (_time . bv) 0) s)
              (write-long (if (>= _time 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _time s)(write-long (if (>= _time 0) 0 #xffffffff) s)))
     ;; uint32 _idx
       (write-long _idx s)
     ;; uint32 _mode
       (write-long _mode s)
     ;; uint32 _ant
       (write-long _ant s)
     ;; float64 _phase
       (sys::poke _phase (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rssi
       (sys::poke _rssi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _epc
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _epc (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint64 _time
     
#+(or :alpha :irix6 :x86_64)
      (setf _time (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _time (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint32 _idx
     (setq _idx (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _ant
     (setq _ant (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _phase
     (setq _phase (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rssi
     (setq _rssi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rfid::rfid_msg :md5sum-) "8c4700ac787e3c2aae4f451754e549da")
(setf (get rfid::rfid_msg :datatype-) "rfid/rfid_msg")
(setf (get rfid::rfid_msg :definition-)
      "std_msgs/Header header
string epc
uint64 time
uint32 idx
uint32 mode
uint32 ant
float64 phase
float64 rssi

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

")



(provide :rfid/rfid_msg "8c4700ac787e3c2aae4f451754e549da")


