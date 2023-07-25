;; Auto-generated. Do not edit!


(when (boundp 'speech_pkg::Manager)
  (if (not (find-package "SPEECH_PKG"))
    (make-package "SPEECH_PKG"))
  (shadow 'Manager (find-package "SPEECH_PKG")))
(unless (find-package "SPEECH_PKG::MANAGER")
  (make-package "SPEECH_PKG::MANAGER"))
(unless (find-package "SPEECH_PKG::MANAGERREQUEST")
  (make-package "SPEECH_PKG::MANAGERREQUEST"))
(unless (find-package "SPEECH_PKG::MANAGERRESPONSE")
  (make-package "SPEECH_PKG::MANAGERRESPONSE"))

(in-package "ROS")





(defclass speech_pkg::ManagerRequest
  :super ros::object
  :slots (_data ))

(defmethod speech_pkg::ManagerRequest
  (:init
   (&key
    ((:data __data) (instance speech_pkg::SpeechData :init))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; speech_pkg/SpeechData _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; speech_pkg/SpeechData _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; speech_pkg/SpeechData _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(defclass speech_pkg::ManagerResponse
  :super ros::object
  :slots (_flag ))

(defmethod speech_pkg::ManagerResponse
  (:init
   (&key
    ((:flag __flag) nil)
    )
   (send-super :init)
   (setq _flag __flag)
   self)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; bool _flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass speech_pkg::Manager
  :super ros::object
  :slots ())

(setf (get speech_pkg::Manager :md5sum-) "426efa73e6c8d460d7543a8bf442644f")
(setf (get speech_pkg::Manager :datatype-) "speech_pkg/Manager")
(setf (get speech_pkg::Manager :request) speech_pkg::ManagerRequest)
(setf (get speech_pkg::Manager :response) speech_pkg::ManagerResponse)

(defmethod speech_pkg::ManagerRequest
  (:response () (instance speech_pkg::ManagerResponse :init)))

(setf (get speech_pkg::ManagerRequest :md5sum-) "426efa73e6c8d460d7543a8bf442644f")
(setf (get speech_pkg::ManagerRequest :datatype-) "speech_pkg/ManagerRequest")
(setf (get speech_pkg::ManagerRequest :definition-)
      "speech_pkg/SpeechData data

================================================================================
MSG: speech_pkg/SpeechData
int16[] data
int16 doa
float64 start_time
---
bool flag
")

(setf (get speech_pkg::ManagerResponse :md5sum-) "426efa73e6c8d460d7543a8bf442644f")
(setf (get speech_pkg::ManagerResponse :datatype-) "speech_pkg/ManagerResponse")
(setf (get speech_pkg::ManagerResponse :definition-)
      "speech_pkg/SpeechData data

================================================================================
MSG: speech_pkg/SpeechData
int16[] data
int16 doa
float64 start_time
---
bool flag
")



(provide :speech_pkg/Manager "426efa73e6c8d460d7543a8bf442644f")


