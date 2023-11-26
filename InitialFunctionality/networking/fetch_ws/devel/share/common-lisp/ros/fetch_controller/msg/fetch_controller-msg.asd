
(cl:in-package :asdf)

(defsystem "fetch_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "controller_state" :depends-on ("_package_controller_state"))
    (:file "_package_controller_state" :depends-on ("_package"))
  ))