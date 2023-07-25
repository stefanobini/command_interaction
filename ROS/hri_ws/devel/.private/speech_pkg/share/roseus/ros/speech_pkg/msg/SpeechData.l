;; Auto-generated. Do not edit!


(when (boundp 'speech_pkg::SpeechData)
  (if (not (find-package "SPEECH_PKG"))
    (make-package "SPEECH_PKG"))
  (shadow 'SpeechData (find-package "SPEECH_PKG")))
(unless (find-package "SPEECH_PKG::SPEECHDATA")
  (make-package "SPEECH_PKG::SPEECHDATA"))

(in-package "ROS")
;;//! \htmlinclude SpeechData.msg.html


(defclass speech_pkg::SpeechData
  :super ros::object
  :slots (_data _doa _start_time _end_time ))

(defmethod speech_pkg::SpeechData
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    ((:doa __doa) 0)
    ((:start_time __start_time) 0.0)
    ((:end_time __end_time) 0.0)
    )
   (send-super :init)
   (setq _data __data)
   (setq _doa (round __doa))
   (setq _start_time (float __start_time))
   (setq _end_time (float __end_time))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:doa
   (&optional __doa)
   (if __doa (setq _doa __doa)) _doa)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:end_time
   (&optional __end_time)
   (if __end_time (setq _end_time __end_time)) _end_time)
  (:serialization-length
   ()
   (+
    ;; int16[] _data
    (* 2    (length _data)) 4
    ;; int16 _doa
    2
    ;; float64 _start_time
    8
    ;; float64 _end_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-word (elt _data i) s)
       )
     ;; int16 _doa
       (write-word _doa s)
     ;; float64 _start_time
       (sys::poke _start_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _end_time
       (sys::poke _end_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16 _doa
     (setq _doa (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _start_time
     (setq _start_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _end_time
     (setq _end_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get speech_pkg::SpeechData :md5sum-) "a2d258c719b9a4af008aff70e592c3a6")
(setf (get speech_pkg::SpeechData :datatype-) "speech_pkg/SpeechData")
(setf (get speech_pkg::SpeechData :definition-)
      "int16[] data
int16 doa
float64 start_time
float64 end_time
")



(provide :speech_pkg/SpeechData "a2d258c719b9a4af008aff70e592c3a6")


