
(cl:in-package :asdf)

(defsystem "speech_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :speech_pkg-msg
)
  :components ((:file "_package")
    (:file "SCR" :depends-on ("_package_SCR"))
    (:file "_package_SCR" :depends-on ("_package"))
    (:file "SpeechManager" :depends-on ("_package_SpeechManager"))
    (:file "_package_SpeechManager" :depends-on ("_package"))
  ))