
(cl:in-package :asdf)

(defsystem "gesture_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CameraInfo" :depends-on ("_package_CameraInfo"))
    (:file "_package_CameraInfo" :depends-on ("_package"))
  ))