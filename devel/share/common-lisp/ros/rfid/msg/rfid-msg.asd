
(cl:in-package :asdf)

(defsystem "rfid-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "rfid_msg" :depends-on ("_package_rfid_msg"))
    (:file "_package_rfid_msg" :depends-on ("_package"))
  ))