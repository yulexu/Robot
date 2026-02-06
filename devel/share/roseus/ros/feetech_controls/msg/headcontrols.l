;; Auto-generated. Do not edit!


(when (boundp 'feetech_controls::headcontrols)
  (if (not (find-package "FEETECH_CONTROLS"))
    (make-package "FEETECH_CONTROLS"))
  (shadow 'headcontrols (find-package "FEETECH_CONTROLS")))
(unless (find-package "FEETECH_CONTROLS::HEADCONTROLS")
  (make-package "FEETECH_CONTROLS::HEADCONTROLS"))

(in-package "ROS")
;;//! \htmlinclude headcontrols.msg.html


(defclass feetech_controls::headcontrols
  :super ros::object
  :slots (_arucoid _type _headjointvalues1 _headjointvalues2 ))

(defmethod feetech_controls::headcontrols
  (:init
   (&key
    ((:arucoid __arucoid) 0)
    ((:type __type) 0)
    ((:headjointvalues1 __headjointvalues1) 0.0)
    ((:headjointvalues2 __headjointvalues2) 0.0)
    )
   (send-super :init)
   (setq _arucoid (round __arucoid))
   (setq _type (round __type))
   (setq _headjointvalues1 (float __headjointvalues1))
   (setq _headjointvalues2 (float __headjointvalues2))
   self)
  (:arucoid
   (&optional __arucoid)
   (if __arucoid (setq _arucoid __arucoid)) _arucoid)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:headjointvalues1
   (&optional __headjointvalues1)
   (if __headjointvalues1 (setq _headjointvalues1 __headjointvalues1)) _headjointvalues1)
  (:headjointvalues2
   (&optional __headjointvalues2)
   (if __headjointvalues2 (setq _headjointvalues2 __headjointvalues2)) _headjointvalues2)
  (:serialization-length
   ()
   (+
    ;; int16 _arucoid
    2
    ;; int16 _type
    2
    ;; float32 _headjointvalues1
    4
    ;; float32 _headjointvalues2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _arucoid
       (write-word _arucoid s)
     ;; int16 _type
       (write-word _type s)
     ;; float32 _headjointvalues1
       (sys::poke _headjointvalues1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _headjointvalues2
       (sys::poke _headjointvalues2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _arucoid
     (setq _arucoid (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _type
     (setq _type (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _headjointvalues1
     (setq _headjointvalues1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _headjointvalues2
     (setq _headjointvalues2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get feetech_controls::headcontrols :md5sum-) "1bdf8924bc3daacda9a890695dd945ec")
(setf (get feetech_controls::headcontrols :datatype-) "feetech_controls/headcontrols")
(setf (get feetech_controls::headcontrols :definition-)
      "int16 arucoid
int16 type
float32 headjointvalues1
float32 headjointvalues2

")



(provide :feetech_controls/headcontrols "1bdf8924bc3daacda9a890695dd945ec")


