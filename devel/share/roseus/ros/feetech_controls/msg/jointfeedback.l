;; Auto-generated. Do not edit!


(when (boundp 'feetech_controls::jointfeedback)
  (if (not (find-package "FEETECH_CONTROLS"))
    (make-package "FEETECH_CONTROLS"))
  (shadow 'jointfeedback (find-package "FEETECH_CONTROLS")))
(unless (find-package "FEETECH_CONTROLS::JOINTFEEDBACK")
  (make-package "FEETECH_CONTROLS::JOINTFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude jointfeedback.msg.html


(defclass feetech_controls::jointfeedback
  :super ros::object
  :slots (_jointvalues ))

(defmethod feetech_controls::jointfeedback
  (:init
   (&key
    ((:jointvalues __jointvalues) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _jointvalues __jointvalues)
   self)
  (:jointvalues
   (&optional __jointvalues)
   (if __jointvalues (setq _jointvalues __jointvalues)) _jointvalues)
  (:serialization-length
   ()
   (+
    ;; float64[] _jointvalues
    (* 8    (length _jointvalues)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _jointvalues
     (write-long (length _jointvalues) s)
     (dotimes (i (length _jointvalues))
       (sys::poke (elt _jointvalues i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _jointvalues
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _jointvalues (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _jointvalues i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get feetech_controls::jointfeedback :md5sum-) "a9d4993eb244bba482f1be1b903382b4")
(setf (get feetech_controls::jointfeedback :datatype-) "feetech_controls/jointfeedback")
(setf (get feetech_controls::jointfeedback :definition-)
      "float64[] jointvalues

")



(provide :feetech_controls/jointfeedback "a9d4993eb244bba482f1be1b903382b4")


