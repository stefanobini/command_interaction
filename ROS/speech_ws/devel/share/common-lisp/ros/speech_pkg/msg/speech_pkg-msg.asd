
(cl:in-package :asdf)

(defsystem "speech_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
    (:file "Gesture" :depends-on ("_package_Gesture"))
    (:file "_package_Gesture" :depends-on ("_package"))
    (:file "Speech" :depends-on ("_package_Speech"))
    (:file "_package_Speech" :depends-on ("_package"))
    (:file "SpeechData" :depends-on ("_package_SpeechData"))
    (:file "_package_SpeechData" :depends-on ("_package"))
  ))