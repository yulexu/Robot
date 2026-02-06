
(cl:in-package :asdf)

(defsystem "ros_hand_gesture_recognition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HandGesture" :depends-on ("_package_HandGesture"))
    (:file "_package_HandGesture" :depends-on ("_package"))
  ))