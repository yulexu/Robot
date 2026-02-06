;; Auto-generated. Do not edit!


(when (boundp 'feetech_controls::readjointvalues)
  (if (not (find-package "FEETECH_CONTROLS"))
    (make-package "FEETECH_CONTROLS"))
  (shadow 'readjointvalues (find-package "FEETECH_CONTROLS")))
(unless (find-package "FEETECH_CONTROLS::READJOINTVALUES")
  (make-package "FEETECH_CONTROLS::READJOINTVALUES"))
(unless (find-package "FEETECH_CONTROLS::READJOINTVALUESREQUEST")
  (make-package "FEETECH_CONTROLS::READJOINTVALUESREQUEST"))
(unless (find-package "FEETECH_CONTROLS::READJOINTVALUESRESPONSE")
  (make-package "FEETECH_CONTROLS::READJOINTVALUESRESPONSE"))

(in-package "ROS")





(defclass feetech_controls::readjointvaluesRequest
  :super ros::object
  :slots (_idstart _idend ))

(defmethod feetech_controls::readjointvaluesRequest
  (:init
   (&key
    ((:idstart __idstart) 0)
    ((:idend __idend) 0)
    )
   (send-super :init)
   (setq _idstart (round __idstart))
   (setq _idend (round __idend))
   self)
  (:idstart
   (&optional __idstart)
   (if __idstart (setq _idstart __idstart)) _idstart)
  (:idend
   (&optional __idend)
   (if __idend (setq _idend __idend)) _idend)
  (:serialization-length
   ()
   (+
    ;; int64 _idstart
    8
    ;; int64 _idend
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _idstart
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _idstart (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _idstart) (= (length (_idstart . bv)) 2)) ;; bignum
              (write-long (ash (elt (_idstart . bv) 0) 0) s)
              (write-long (ash (elt (_idstart . bv) 1) -1) s))
             ((and (class _idstart) (= (length (_idstart . bv)) 1)) ;; big1
              (write-long (elt (_idstart . bv) 0) s)
              (write-long (if (>= _idstart 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _idstart s)(write-long (if (>= _idstart 0) 0 #xffffffff) s)))
     ;; int64 _idend
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _idend (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _idend) (= (length (_idend . bv)) 2)) ;; bignum
              (write-long (ash (elt (_idend . bv) 0) 0) s)
              (write-long (ash (elt (_idend . bv) 1) -1) s))
             ((and (class _idend) (= (length (_idend . bv)) 1)) ;; big1
              (write-long (elt (_idend . bv) 0) s)
              (write-long (if (>= _idend 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _idend s)(write-long (if (>= _idend 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _idstart
#+(or :alpha :irix6 :x86_64)
      (setf _idstart (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _idstart (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _idend
#+(or :alpha :irix6 :x86_64)
      (setf _idend (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _idend (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass feetech_controls::readjointvaluesResponse
  :super ros::object
  :slots (_jointreadvalues ))

(defmethod feetech_controls::readjointvaluesResponse
  (:init
   (&key
    ((:jointreadvalues __jointreadvalues) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _jointreadvalues __jointreadvalues)
   self)
  (:jointreadvalues
   (&optional __jointreadvalues)
   (if __jointreadvalues (setq _jointreadvalues __jointreadvalues)) _jointreadvalues)
  (:serialization-length
   ()
   (+
    ;; float64[] _jointreadvalues
    (* 8    (length _jointreadvalues)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _jointreadvalues
     (write-long (length _jointreadvalues) s)
     (dotimes (i (length _jointreadvalues))
       (sys::poke (elt _jointreadvalues i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _jointreadvalues
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _jointreadvalues (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _jointreadvalues i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass feetech_controls::readjointvalues
  :super ros::object
  :slots ())

(setf (get feetech_controls::readjointvalues :md5sum-) "21a5fa9eeca5ec0f0f497e63ffbb5aa1")
(setf (get feetech_controls::readjointvalues :datatype-) "feetech_controls/readjointvalues")
(setf (get feetech_controls::readjointvalues :request) feetech_controls::readjointvaluesRequest)
(setf (get feetech_controls::readjointvalues :response) feetech_controls::readjointvaluesResponse)

(defmethod feetech_controls::readjointvaluesRequest
  (:response () (instance feetech_controls::readjointvaluesResponse :init)))

(setf (get feetech_controls::readjointvaluesRequest :md5sum-) "21a5fa9eeca5ec0f0f497e63ffbb5aa1")
(setf (get feetech_controls::readjointvaluesRequest :datatype-) "feetech_controls/readjointvaluesRequest")
(setf (get feetech_controls::readjointvaluesRequest :definition-)
      "int64 idstart
int64 idend
---
float64[] jointreadvalues

")

(setf (get feetech_controls::readjointvaluesResponse :md5sum-) "21a5fa9eeca5ec0f0f497e63ffbb5aa1")
(setf (get feetech_controls::readjointvaluesResponse :datatype-) "feetech_controls/readjointvaluesResponse")
(setf (get feetech_controls::readjointvaluesResponse :definition-)
      "int64 idstart
int64 idend
---
float64[] jointreadvalues

")



(provide :feetech_controls/readjointvalues "21a5fa9eeca5ec0f0f497e63ffbb5aa1")


