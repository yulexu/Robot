
(cl:in-package :asdf)

(defsystem "feetech_controls-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "readjointvalues" :depends-on ("_package_readjointvalues"))
    (:file "_package_readjointvalues" :depends-on ("_package"))
    (:file "torqcontrol" :depends-on ("_package_torqcontrol"))
    (:file "_package_torqcontrol" :depends-on ("_package"))
  ))