;; Auto-generated. Do not edit!


(when (boundp 'speech_pkg::Classification)
  (if (not (find-package "SPEECH_PKG"))
    (make-package "SPEECH_PKG"))
  (shadow 'Classification (find-package "SPEECH_PKG")))
(unless (find-package "SPEECH_PKG::CLASSIFICATION")
  (make-package "SPEECH_PKG::CLASSIFICATION"))
(unless (find-package "SPEECH_PKG::CLASSIFICATIONREQUEST")
  (make-package "SPEECH_PKG::CLASSIFICATIONREQUEST"))
(unless (find-package "SPEECH_PKG::CLASSIFICATIONRESPONSE")
  (make-package "SPEECH_PKG::CLASSIFICATIONRESPONSE"))

(in-package "ROS")





(defclass speech_pkg::ClassificationRequest
  :super ros::object
  :slots (_data ))

(defmethod speech_pkg::ClassificationRequest
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

(defclass speech_pkg::ClassificationResponse
  :super ros::object
  :slots (_cmd _probs ))

(defmethod speech_pkg::ClassificationResponse
  (:init
   (&key
    ((:cmd __cmd) 0)
    ((:probs __probs) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _cmd (round __cmd))
   (setq _probs __probs)
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:probs
   (&optional __probs)
   (if __probs (setq _probs __probs)) _probs)
  (:serialization-length
   ()
   (+
    ;; int8 _cmd
    1
    ;; float32[] _probs
    (* 4    (length _probs)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _cmd
       (write-byte _cmd s)
     ;; float32[] _probs
     (write-long (length _probs) s)
     (dotimes (i (length _probs))
       (sys::poke (elt _probs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _cmd
     (setq _cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _cmd 127) (setq _cmd (- _cmd 256)))
   ;; float32[] _probs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _probs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _probs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass speech_pkg::Classification
  :super ros::object
  :slots ())

(setf (get speech_pkg::Classification :md5sum-) "896849feb7849138871db12c46e37d25")
(setf (get speech_pkg::Classification :datatype-) "speech_pkg/Classification")
(setf (get speech_pkg::Classification :request) speech_pkg::ClassificationRequest)
(setf (get speech_pkg::Classification :response) speech_pkg::ClassificationResponse)

(defmethod speech_pkg::ClassificationRequest
  (:response () (instance speech_pkg::ClassificationResponse :init)))

(setf (get speech_pkg::ClassificationRequest :md5sum-) "896849feb7849138871db12c46e37d25")
(setf (get speech_pkg::ClassificationRequest :datatype-) "speech_pkg/ClassificationRequest")
(setf (get speech_pkg::ClassificationRequest :definition-)
      "speech_pkg/SpeechData data

================================================================================
MSG: speech_pkg/SpeechData
int16[] data
int16 doa
float64 start_time
---
int8 cmd
float32[] probs
")

(setf (get speech_pkg::ClassificationResponse :md5sum-) "896849feb7849138871db12c46e37d25")
(setf (get speech_pkg::ClassificationResponse :datatype-) "speech_pkg/ClassificationResponse")
(setf (get speech_pkg::ClassificationResponse :definition-)
      "speech_pkg/SpeechData data

================================================================================
MSG: speech_pkg/SpeechData
int16[] data
int16 doa
float64 start_time
---
int8 cmd
float32[] probs
")



(provide :speech_pkg/Classification "896849feb7849138871db12c46e37d25")


