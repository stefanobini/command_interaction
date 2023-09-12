
(cl:in-package :asdf)

(defsystem "speech_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :speech_pkg-msg
)
  :components ((:file "_package")
    (:file "Classification" :depends-on ("_package_Classification"))
    (:file "_package_Classification" :depends-on ("_package"))
    (:file "ClassificationMSI" :depends-on ("_package_ClassificationMSI"))
    (:file "_package_ClassificationMSI" :depends-on ("_package"))
    (:file "Manager" :depends-on ("_package_Manager"))
    (:file "_package_Manager" :depends-on ("_package"))
    (:file "Talker" :depends-on ("_package_Talker"))
    (:file "_package_Talker" :depends-on ("_package"))
  ))