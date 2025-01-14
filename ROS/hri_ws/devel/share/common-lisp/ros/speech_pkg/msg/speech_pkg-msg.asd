
(cl:in-package :asdf)

(defsystem "speech_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
    (:file "Gesture" :depends-on ("_package_Gesture"))
    (:file "_package_Gesture" :depends-on ("_package"))
    (:file "IntentIRI" :depends-on ("_package_IntentIRI"))
    (:file "_package_IntentIRI" :depends-on ("_package"))
    (:file "Speech" :depends-on ("_package_Speech"))
    (:file "_package_Speech" :depends-on ("_package"))
    (:file "SpeechData" :depends-on ("_package_SpeechData"))
    (:file "_package_SpeechData" :depends-on ("_package"))
    (:file "SystemHealth" :depends-on ("_package_SystemHealth"))
    (:file "_package_SystemHealth" :depends-on ("_package"))
  ))