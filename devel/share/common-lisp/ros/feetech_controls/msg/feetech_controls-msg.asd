
(cl:in-package :asdf)

(defsystem "feetech_controls-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "headcontrols" :depends-on ("_package_headcontrols"))
    (:file "_package_headcontrols" :depends-on ("_package"))
    (:file "jointcontrols" :depends-on ("_package_jointcontrols"))
    (:file "_package_jointcontrols" :depends-on ("_package"))
    (:file "jointfeedback" :depends-on ("_package_jointfeedback"))
    (:file "_package_jointfeedback" :depends-on ("_package"))
  ))