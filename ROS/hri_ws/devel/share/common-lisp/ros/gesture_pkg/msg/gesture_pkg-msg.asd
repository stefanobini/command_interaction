
(cl:in-package :asdf)

(defsystem "gesture_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
    (:file "Gesture" :depends-on ("_package_Gesture"))
    (:file "_package_Gesture" :depends-on ("_package"))
    (:file "SystemHealth" :depends-on ("_package_SystemHealth"))
    (:file "_package_SystemHealth" :depends-on ("_package"))
  ))