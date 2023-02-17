;; Auto-generated. Do not edit!


(when (boundp 'speech_pkg::Talker)
  (if (not (find-package "SPEECH_PKG"))
    (make-package "SPEECH_PKG"))
  (shadow 'Talker (find-package "SPEECH_PKG")))
(unless (find-package "SPEECH_PKG::TALKER")
  (make-package "SPEECH_PKG::TALKER"))
(unless (find-package "SPEECH_PKG::TALKERREQUEST")
  (make-package "SPEECH_PKG::TALKERREQUEST"))
(unless (find-package "SPEECH_PKG::TALKERRESPONSE")
  (make-package "SPEECH_PKG::TALKERRESPONSE"))

(in-package "ROS")





(defclass speech_pkg::TalkerRequest
  :super ros::object
  :slots (_cmd _probs ))

(defmethod speech_pkg::TalkerRequest
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

(defclass speech_pkg::TalkerResponse
  :super ros::object
  :slots (_flag ))

(defmethod speech_pkg::TalkerResponse
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

(defclass speech_pkg::Talker
  :super ros::object
  :slots ())

(setf (get speech_pkg::Talker :md5sum-) "b542a23d9f2300b346a3653dfa7b8829")
(setf (get speech_pkg::Talker :datatype-) "speech_pkg/Talker")
(setf (get speech_pkg::Talker :request) speech_pkg::TalkerRequest)
(setf (get speech_pkg::Talker :response) speech_pkg::TalkerResponse)

(defmethod speech_pkg::TalkerRequest
  (:response () (instance speech_pkg::TalkerResponse :init)))

(setf (get speech_pkg::TalkerRequest :md5sum-) "b542a23d9f2300b346a3653dfa7b8829")
(setf (get speech_pkg::TalkerRequest :datatype-) "speech_pkg/TalkerRequest")
(setf (get speech_pkg::TalkerRequest :definition-)
      "int8 cmd
float32[] probs
---
bool flag
")

(setf (get speech_pkg::TalkerResponse :md5sum-) "b542a23d9f2300b346a3653dfa7b8829")
(setf (get speech_pkg::TalkerResponse :datatype-) "speech_pkg/TalkerResponse")
(setf (get speech_pkg::TalkerResponse :definition-)
      "int8 cmd
float32[] probs
---
bool flag
")



(provide :speech_pkg/Talker "b542a23d9f2300b346a3653dfa7b8829")


