;; Auto-generated. Do not edit!


(when (boundp 'speech_pkg::SpeechManager)
  (if (not (find-package "SPEECH_PKG"))
    (make-package "SPEECH_PKG"))
  (shadow 'SpeechManager (find-package "SPEECH_PKG")))
(unless (find-package "SPEECH_PKG::SPEECHMANAGER")
  (make-package "SPEECH_PKG::SPEECHMANAGER"))
(unless (find-package "SPEECH_PKG::SPEECHMANAGERREQUEST")
  (make-package "SPEECH_PKG::SPEECHMANAGERREQUEST"))
(unless (find-package "SPEECH_PKG::SPEECHMANAGERRESPONSE")
  (make-package "SPEECH_PKG::SPEECHMANAGERRESPONSE"))

(in-package "ROS")





(defclass speech_pkg::SpeechManagerRequest
  :super ros::object
  :slots (_data ))

(defmethod speech_pkg::SpeechManagerRequest
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

(defclass speech_pkg::SpeechManagerResponse
  :super ros::object
  :slots (_flag ))

(defmethod speech_pkg::SpeechManagerResponse
  (:init
   (&key
    ((:flag __flag) nil)
    )
   (send-super :init)
   (setq _flag __flag)
   self)
  (:flag
   (&optional (__flag :null))
   (if (not (eq __flag :null)) (setq _flag __flag)) _flag)
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

(defclass speech_pkg::SpeechManager
  :super ros::object
  :slots ())

(setf (get speech_pkg::SpeechManager :md5sum-) "426efa73e6c8d460d7543a8bf442644f")
(setf (get speech_pkg::SpeechManager :datatype-) "speech_pkg/SpeechManager")
(setf (get speech_pkg::SpeechManager :request) speech_pkg::SpeechManagerRequest)
(setf (get speech_pkg::SpeechManager :response) speech_pkg::SpeechManagerResponse)

(defmethod speech_pkg::SpeechManagerRequest
  (:response () (instance speech_pkg::SpeechManagerResponse :init)))

(setf (get speech_pkg::SpeechManagerRequest :md5sum-) "426efa73e6c8d460d7543a8bf442644f")
(setf (get speech_pkg::SpeechManagerRequest :datatype-) "speech_pkg/SpeechManagerRequest")
(setf (get speech_pkg::SpeechManagerRequest :definition-)
      "speech_pkg/SpeechData data

================================================================================
MSG: speech_pkg/SpeechData
int16[] data
int16 doa
float64 start_time
---
bool flag
")

(setf (get speech_pkg::SpeechManagerResponse :md5sum-) "426efa73e6c8d460d7543a8bf442644f")
(setf (get speech_pkg::SpeechManagerResponse :datatype-) "speech_pkg/SpeechManagerResponse")
(setf (get speech_pkg::SpeechManagerResponse :definition-)
      "speech_pkg/SpeechData data

================================================================================
MSG: speech_pkg/SpeechData
int16[] data
int16 doa
float64 start_time
---
bool flag
")



(provide :speech_pkg/SpeechManager "426efa73e6c8d460d7543a8bf442644f")


