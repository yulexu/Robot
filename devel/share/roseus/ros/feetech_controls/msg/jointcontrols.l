;; Auto-generated. Do not edit!


(when (boundp 'feetech_controls::jointcontrols)
  (if (not (find-package "FEETECH_CONTROLS"))
    (make-package "FEETECH_CONTROLS"))
  (shadow 'jointcontrols (find-package "FEETECH_CONTROLS")))
(unless (find-package "FEETECH_CONTROLS::JOINTCONTROLS")
  (make-package "FEETECH_CONTROLS::JOINTCONTROLS"))

(in-package "ROS")
;;//! \htmlinclude jointcontrols.msg.html


(defclass feetech_controls::jointcontrols
  :super ros::object
  :slots (_id _jointvalues ))

(defmethod feetech_controls::jointcontrols
  (:init
   (&key
    ((:id __id) 0)
    ((:jointvalues __jointvalues) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _jointvalues (float __jointvalues))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:jointvalues
   (&optional __jointvalues)
   (if __jointvalues (setq _jointvalues __jointvalues)) _jointvalues)
  (:serialization-length
   ()
   (+
    ;; int16 _id
    2
    ;; float64 _jointvalues
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _id
       (write-word _id s)
     ;; float64 _jointvalues
       (sys::poke _jointvalues (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _jointvalues
     (setq _jointvalues (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get feetech_controls::jointcontrols :md5sum-) "7988f738ee37fb27a094c2d2252a41df")
(setf (get feetech_controls::jointcontrols :datatype-) "feetech_controls/jointcontrols")
(setf (get feetech_controls::jointcontrols :definition-)
      "int16 id
float64 jointvalues

")



(provide :feetech_controls/jointcontrols "7988f738ee37fb27a094c2d2252a41df")


