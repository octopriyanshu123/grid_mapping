
(cl:in-package :asdf)

(defsystem "my_actuator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ang_lin_arr" :depends-on ("_package_ang_lin_arr"))
    (:file "_package_ang_lin_arr" :depends-on ("_package"))
  ))