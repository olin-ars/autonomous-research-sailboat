
(cl:in-package :asdf)

(defsystem "oars_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjFrame" :depends-on ("_package_ObjFrame"))
    (:file "_package_ObjFrame" :depends-on ("_package"))
  ))