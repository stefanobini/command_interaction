;; Auto-generated. Do not edit!


(when (boundp 'speech_pkg::ClassificationMSI)
  (if (not (find-package "SPEECH_PKG"))
    (make-package "SPEECH_PKG"))
  (shadow 'ClassificationMSI (find-package "SPEECH_PKG")))
(unless (find-package "SPEECH_PKG::CLASSIFICATIONMSI")
  (make-package "SPEECH_PKG::CLASSIFICATIONMSI"))
(unless (find-package "SPEECH_PKG::CLASSIFICATIONMSIREQUEST")
  (make-package "SPEECH_PKG::CLASSIFICATIONMSIREQUEST"))
(unless (find-package "SPEECH_PKG::CLASSIFICATIONMSIRESPONSE")
  (make-package "SPEECH_PKG::CLASSIFICATIONMSIRESPONSE"))

(in-package "ROS")





(defclass speech_pkg::ClassificationMSIRequest
  :super ros::object
  :slots (_data ))

(defmethod speech_pkg::ClassificationMSIRequest
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

(defclass speech_pkg::ClassificationMSIResponse
  :super ros::object
  :slots (_intent _int_probs _explicit _exp_probs _implicit _imp_probs ))

(defmethod speech_pkg::ClassificationMSIResponse
  (:init
   (&key
    ((:intent __intent) 0)
    ((:int_probs __int_probs) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:explicit __explicit) 0)
    ((:exp_probs __exp_probs) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:implicit __implicit) 0)
    ((:imp_probs __imp_probs) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _intent (round __intent))
   (setq _int_probs __int_probs)
   (setq _explicit (round __explicit))
   (setq _exp_probs __exp_probs)
   (setq _implicit (round __implicit))
   (setq _imp_probs __imp_probs)
   self)
  (:intent
   (&optional __intent)
   (if __intent (setq _intent __intent)) _intent)
  (:int_probs
   (&optional __int_probs)
   (if __int_probs (setq _int_probs __int_probs)) _int_probs)
  (:explicit
   (&optional __explicit)
   (if __explicit (setq _explicit __explicit)) _explicit)
  (:exp_probs
   (&optional __exp_probs)
   (if __exp_probs (setq _exp_probs __exp_probs)) _exp_probs)
  (:implicit
   (&optional __implicit)
   (if __implicit (setq _implicit __implicit)) _implicit)
  (:imp_probs
   (&optional __imp_probs)
   (if __imp_probs (setq _imp_probs __imp_probs)) _imp_probs)
  (:serialization-length
   ()
   (+
    ;; int8 _intent
    1
    ;; float32[] _int_probs
    (* 4    (length _int_probs)) 4
    ;; int8 _explicit
    1
    ;; float32[] _exp_probs
    (* 4    (length _exp_probs)) 4
    ;; int8 _implicit
    1
    ;; float32[] _imp_probs
    (* 4    (length _imp_probs)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _intent
       (write-byte _intent s)
     ;; float32[] _int_probs
     (write-long (length _int_probs) s)
     (dotimes (i (length _int_probs))
       (sys::poke (elt _int_probs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int8 _explicit
       (write-byte _explicit s)
     ;; float32[] _exp_probs
     (write-long (length _exp_probs) s)
     (dotimes (i (length _exp_probs))
       (sys::poke (elt _exp_probs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int8 _implicit
       (write-byte _implicit s)
     ;; float32[] _imp_probs
     (write-long (length _imp_probs) s)
     (dotimes (i (length _imp_probs))
       (sys::poke (elt _imp_probs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _intent
     (setq _intent (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _intent 127) (setq _intent (- _intent 256)))
   ;; float32[] _int_probs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _int_probs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _int_probs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int8 _explicit
     (setq _explicit (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _explicit 127) (setq _explicit (- _explicit 256)))
   ;; float32[] _exp_probs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _exp_probs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _exp_probs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int8 _implicit
     (setq _implicit (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _implicit 127) (setq _implicit (- _implicit 256)))
   ;; float32[] _imp_probs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _imp_probs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _imp_probs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass speech_pkg::ClassificationMSI
  :super ros::object
  :slots ())

(setf (get speech_pkg::ClassificationMSI :md5sum-) "86bdd504432c8f4cd29e4aa496bd70a0")
(setf (get speech_pkg::ClassificationMSI :datatype-) "speech_pkg/ClassificationMSI")
(setf (get speech_pkg::ClassificationMSI :request) speech_pkg::ClassificationMSIRequest)
(setf (get speech_pkg::ClassificationMSI :response) speech_pkg::ClassificationMSIResponse)

(defmethod speech_pkg::ClassificationMSIRequest
  (:response () (instance speech_pkg::ClassificationMSIResponse :init)))

(setf (get speech_pkg::ClassificationMSIRequest :md5sum-) "86bdd504432c8f4cd29e4aa496bd70a0")
(setf (get speech_pkg::ClassificationMSIRequest :datatype-) "speech_pkg/ClassificationMSIRequest")
(setf (get speech_pkg::ClassificationMSIRequest :definition-)
      "speech_pkg/SpeechData data

================================================================================
MSG: speech_pkg/SpeechData
int16[] data
int16 doa
float64 start_time
---
int8 intent
float32[] int_probs
int8 explicit
float32[] exp_probs
int8 implicit
float32[] imp_probs
")

(setf (get speech_pkg::ClassificationMSIResponse :md5sum-) "86bdd504432c8f4cd29e4aa496bd70a0")
(setf (get speech_pkg::ClassificationMSIResponse :datatype-) "speech_pkg/ClassificationMSIResponse")
(setf (get speech_pkg::ClassificationMSIResponse :definition-)
      "speech_pkg/SpeechData data

================================================================================
MSG: speech_pkg/SpeechData
int16[] data
int16 doa
float64 start_time
---
int8 intent
float32[] int_probs
int8 explicit
float32[] exp_probs
int8 implicit
float32[] imp_probs
")



(provide :speech_pkg/ClassificationMSI "86bdd504432c8f4cd29e4aa496bd70a0")


