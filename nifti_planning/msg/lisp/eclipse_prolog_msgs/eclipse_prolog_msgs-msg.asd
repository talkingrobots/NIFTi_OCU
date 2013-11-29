
(in-package :asdf)

(defsystem "eclipse_prolog_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "ActionScheduled" :depends-on ("_package"))
    (:file "_package_ActionScheduled" :depends-on ("_package"))
    ))
