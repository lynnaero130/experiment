
(cl:in-package :asdf)

(defsystem "offb_posctl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "controlstate" :depends-on ("_package_controlstate"))
    (:file "_package_controlstate" :depends-on ("_package"))
  ))