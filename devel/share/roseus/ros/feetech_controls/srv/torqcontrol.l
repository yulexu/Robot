;; Auto-generated. Do not edit!


(when (boundp 'feetech_controls::torqcontrol)
  (if (not (find-package "FEETECH_CONTROLS"))
    (make-package "FEETECH_CONTROLS"))
  (shadow 'torqcontrol (find-package "FEETECH_CONTROLS")))
(unless (find-package "FEETECH_CONTROLS::TORQCONTROL")
  (make-package "FEETECH_CONTROLS::TORQCONTROL"))
(unless (find-package "FEETECH_CONTROLS::TORQCONTROLREQUEST")
  (make-package "FEETECH_CONTROLS::TORQCONTROLREQUEST"))
(unless (find-package "FEETECH_CONTROLS::TORQCONTROLRESPONSE")
  (make-package "FEETECH_CONTROLS::TORQCONTROLRESPONSE"))

(in-package "ROS")





(defclass feetech_controls::torqcontrolRequest
  :super ros::object
  :slots (_idrec ))

(defmethod feetech_controls::torqcontrolRequest
  (:init
   (&key
    ((:idrec __idrec) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _idrec __idrec)
   self)
  (:idrec
   (&optional __idrec)
   (if __idrec (setq _idrec __idrec)) _idrec)
  (:serialization-length
   ()
   (+
    ;; int64[] _idrec
    (* 8    (length _idrec)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64[] _idrec
     (write-long (length _idrec) s)
     (dotimes (i (length _idrec))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _idrec i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _idrec i)) (= (length ((elt _idrec i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _idrec i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _idrec i) . bv) 1) -1) s))
             ((and (class (elt _idrec i)) (= (length ((elt _idrec i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _idrec i) . bv) 0) s)
              (write-long (if (>= (elt _idrec i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _idrec i) s)(write-long (if (>= (elt _idrec i) 0) 0 #xffffffff) s)))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64[] _idrec
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _idrec (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _idrec i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _idrec i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;;
   self)
  )

(defclass feetech_controls::torqcontrolResponse
  :super ros::object
  :slots (_idret ))

(defmethod feetech_controls::torqcontrolResponse
  (:init
   (&key
    ((:idret __idret) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _idret __idret)
   self)
  (:idret
   (&optional __idret)
   (if __idret (setq _idret __idret)) _idret)
  (:serialization-length
   ()
   (+
    ;; bool[] _idret
    (* 1    (length _idret)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool[] _idret
     (write-long (length _idret) s)
     (dotimes (i (length _idret))
       (if (elt _idret i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool[] _idret
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _idret (make-list n))
     (dotimes (i n)
     (setf (elt _idret i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(defclass feetech_controls::torqcontrol
  :super ros::object
  :slots ())

(setf (get feetech_controls::torqcontrol :md5sum-) "c7fd7facee233a061827f86f586e5d13")
(setf (get feetech_controls::torqcontrol :datatype-) "feetech_controls/torqcontrol")
(setf (get feetech_controls::torqcontrol :request) feetech_controls::torqcontrolRequest)
(setf (get feetech_controls::torqcontrol :response) feetech_controls::torqcontrolResponse)

(defmethod feetech_controls::torqcontrolRequest
  (:response () (instance feetech_controls::torqcontrolResponse :init)))

(setf (get feetech_controls::torqcontrolRequest :md5sum-) "c7fd7facee233a061827f86f586e5d13")
(setf (get feetech_controls::torqcontrolRequest :datatype-) "feetech_controls/torqcontrolRequest")
(setf (get feetech_controls::torqcontrolRequest :definition-)
      "int64[] idrec
---
bool[] idret

")

(setf (get feetech_controls::torqcontrolResponse :md5sum-) "c7fd7facee233a061827f86f586e5d13")
(setf (get feetech_controls::torqcontrolResponse :datatype-) "feetech_controls/torqcontrolResponse")
(setf (get feetech_controls::torqcontrolResponse :definition-)
      "int64[] idrec
---
bool[] idret

")



(provide :feetech_controls/torqcontrol "c7fd7facee233a061827f86f586e5d13")


