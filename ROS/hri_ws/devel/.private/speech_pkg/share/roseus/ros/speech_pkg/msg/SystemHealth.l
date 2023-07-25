;; Auto-generated. Do not edit!


(when (boundp 'speech_pkg::SystemHealth)
  (if (not (find-package "SPEECH_PKG"))
    (make-package "SPEECH_PKG"))
  (shadow 'SystemHealth (find-package "SPEECH_PKG")))
(unless (find-package "SPEECH_PKG::SYSTEMHEALTH")
  (make-package "SPEECH_PKG::SYSTEMHEALTH"))

(in-package "ROS")
;;//! \htmlinclude SystemHealth.msg.html


(defclass speech_pkg::SystemHealth
  :super ros::object
  :slots (_id _type _timestamp _status ))

(defmethod speech_pkg::SystemHealth
  (:init
   (&key
    ((:id __id) "")
    ((:type __type) "")
    ((:timestamp __timestamp) "")
    ((:status __status) "")
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _type (string __type))
   (setq _timestamp (string __timestamp))
   (setq _status (string __status))
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
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; string _type
    4 (length _type)
    ;; string _timestamp
    4 (length _timestamp)
    ;; string _status
    4 (length _status)
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
     ;; string _status
       (write-long (length _status) s) (princ _status s)
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
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get speech_pkg::SystemHealth :md5sum-) "2ca1632d366bfbb66f928108e6362e1d")
(setf (get speech_pkg::SystemHealth :datatype-) "speech_pkg/SystemHealth")
(setf (get speech_pkg::SystemHealth :definition-)
      "# Identify the complete Entity/topic name (UNISA.SpeechGestureAnalysis.Speech)
string id
# Identify the Entity/topic type (Speech)
string type
#Two-integer timestamp that is expressed as:
# * timestamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * timestamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
string timestamp
# Show the status information
string status
")



(provide :speech_pkg/SystemHealth "2ca1632d366bfbb66f928108e6362e1d")


