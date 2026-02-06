;; Auto-generated. Do not edit!


(when (boundp 'sort_track::TrackedObject)
  (if (not (find-package "SORT_TRACK"))
    (make-package "SORT_TRACK"))
  (shadow 'TrackedObject (find-package "SORT_TRACK")))
(unless (find-package "SORT_TRACK::TRACKEDOBJECT")
  (make-package "SORT_TRACK::TRACKEDOBJECT"))

(in-package "ROS")
;;//! \htmlinclude TrackedObject.msg.html


(defclass sort_track::TrackedObject
  :super ros::object
  :slots (_xmin _ymin _xmax _ymax _id _center_x _center_y ))

(defmethod sort_track::TrackedObject
  (:init
   (&key
    ((:xmin __xmin) 0)
    ((:ymin __ymin) 0)
    ((:xmax __xmax) 0)
    ((:ymax __ymax) 0)
    ((:id __id) 0)
    ((:center_x __center_x) 0)
    ((:center_y __center_y) 0)
    )
   (send-super :init)
   (setq _xmin (round __xmin))
   (setq _ymin (round __ymin))
   (setq _xmax (round __xmax))
   (setq _ymax (round __ymax))
   (setq _id (round __id))
   (setq _center_x (round __center_x))
   (setq _center_y (round __center_y))
   self)
  (:xmin
   (&optional __xmin)
   (if __xmin (setq _xmin __xmin)) _xmin)
  (:ymin
   (&optional __ymin)
   (if __ymin (setq _ymin __ymin)) _ymin)
  (:xmax
   (&optional __xmax)
   (if __xmax (setq _xmax __xmax)) _xmax)
  (:ymax
   (&optional __ymax)
   (if __ymax (setq _ymax __ymax)) _ymax)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:serialization-length
   ()
   (+
    ;; int32 _xmin
    4
    ;; int32 _ymin
    4
    ;; int32 _xmax
    4
    ;; int32 _ymax
    4
    ;; int32 _id
    4
    ;; int32 _center_x
    4
    ;; int32 _center_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _xmin
       (write-long _xmin s)
     ;; int32 _ymin
       (write-long _ymin s)
     ;; int32 _xmax
       (write-long _xmax s)
     ;; int32 _ymax
       (write-long _ymax s)
     ;; int32 _id
       (write-long _id s)
     ;; int32 _center_x
       (write-long _center_x s)
     ;; int32 _center_y
       (write-long _center_y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _xmin
     (setq _xmin (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ymin
     (setq _ymin (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _xmax
     (setq _xmax (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ymax
     (setq _ymax (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _center_x
     (setq _center_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _center_y
     (setq _center_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get sort_track::TrackedObject :md5sum-) "9768f179b9235be5f03960ec9dc550a9")
(setf (get sort_track::TrackedObject :datatype-) "sort_track/TrackedObject")
(setf (get sort_track::TrackedObject :definition-)
      "int32 xmin
int32 ymin
int32 xmax
int32 ymax
int32 id
int32 center_x
int32 center_y


")



(provide :sort_track/TrackedObject "9768f179b9235be5f03960ec9dc550a9")


