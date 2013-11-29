
(in-package :asdf)

(defsystem "eclipse_prolog_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "Task" :depends-on ("_package"))
    (:file "_package_Task" :depends-on ("_package"))
    ))
