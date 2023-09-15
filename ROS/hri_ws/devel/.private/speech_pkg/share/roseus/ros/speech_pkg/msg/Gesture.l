;; Auto-generated. Do not edit!


(when (boundp 'speech_pkg::Gesture)
  (if (not (find-package "SPEECH_PKG"))
    (make-package "SPEECH_PKG"))
  (shadow 'Gesture (find-package "SPEECH_PKG")))
(unless (find-package "SPEECH_PKG::GESTURE")
  (make-package "SPEECH_PKG::GESTURE"))

(in-package "ROS")
;;//! \htmlinclude Gesture.msg.html


(defclass speech_pkg::Gesture
  :super ros::object
  :slots (_id _type _timestamp _command _confidence ))

(defmethod speech_pkg::Gesture
  (:init
   (&key
    ((:id __id) "")
    ((:type __type) "")
    ((:timestamp __timestamp) "")
    ((:command __command) (instance speech_pkg::Command :init))
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _type (string __type))
   (setq _timestamp (string __timestamp))
   (setq _command __command)
   (setq _confidence (float __confidence))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:command
   (&rest __command)
   (if (keywordp (car __command))
       (send* _command __command)
     (progn
       (if __command (setq _command (car __command)))
       _command)))
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; string _type
    4 (length _type)
    ;; string _timestamp
    4 (length _timestamp)
    ;; speech_pkg/Command _command
    (send _command :serialization-length)
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; string _timestamp
       (write-long (length _timestamp) s) (princ _timestamp s)
     ;; speech_pkg/Command _command
       (send _command :serialize s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _timestamp
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _timestamp (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; speech_pkg/Command _command
     (send _command :deserialize buf ptr-) (incf ptr- (send _command :serialization-length))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get speech_pkg::Gesture :md5sum-) "c31bef49fddac6ccf19a75e92a735ec5")
(setf (get speech_pkg::Gesture :datatype-) "speech_pkg/Gesture")
(setf (get speech_pkg::Gesture :definition-)
      "# 

# Identify the complete Entity/topic name (UNISA.SpeechGestureAnalysis.Speech)
string id
# Identify the Entity/topic type (Speech)
string type
#Two-integer timestamp that is expressed as:
# * timestamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * timestamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
string timestamp
# Identify the command detected by Speech/Command system, it contains the following information:
# * Numeric identifier associated with the command
# * Textual description of the gesture, in English
# * Textual description of the gesture, in Italian
Command command
# Value between 0 and 1 which indicates the reliability in the command identification and classification
float32 confidence
================================================================================
MSG: speech_pkg/Command
# Identify the command detected by Speech/Command system, it contains the following information:

# Numeric identifier associated with the command
uint32 label
# Textual description of the gesture, in English
string english
# Textual description of the gesture, in Italian
string italian
")



(provide :speech_pkg/Gesture "c31bef49fddac6ccf19a75e92a735ec5")


