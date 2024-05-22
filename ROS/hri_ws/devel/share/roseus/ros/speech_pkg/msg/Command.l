;; Auto-generated. Do not edit!


(when (boundp 'speech_pkg::Command)
  (if (not (find-package "SPEECH_PKG"))
    (make-package "SPEECH_PKG"))
  (shadow 'Command (find-package "SPEECH_PKG")))
(unless (find-package "SPEECH_PKG::COMMAND")
  (make-package "SPEECH_PKG::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude Command.msg.html


(defclass speech_pkg::Command
  :super ros::object
  :slots (_label _english _italian ))

(defmethod speech_pkg::Command
  (:init
   (&key
    ((:label __label) 0)
    ((:english __english) "")
    ((:italian __italian) "")
    )
   (send-super :init)
   (setq _label (round __label))
   (setq _english (string __english))
   (setq _italian (string __italian))
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:english
   (&optional __english)
   (if __english (setq _english __english)) _english)
  (:italian
   (&optional __italian)
   (if __italian (setq _italian __italian)) _italian)
  (:serialization-length
   ()
   (+
    ;; uint32 _label
    4
    ;; string _english
    4 (length _english)
    ;; string _italian
    4 (length _italian)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _label
       (write-long _label s)
     ;; string _english
       (write-long (length _english) s) (princ _english s)
     ;; string _italian
       (write-long (length _italian) s) (princ _italian s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _label
     (setq _label (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _english
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _english (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _italian
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _italian (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get speech_pkg::Command :md5sum-) "3a48558522c26082d66040db118e6fd8")
(setf (get speech_pkg::Command :datatype-) "speech_pkg/Command")
(setf (get speech_pkg::Command :definition-)
      "# Identify the command detected by Speech/Command system, it contains the following information:

# Numeric identifier associated with the command
uint32 label
# Textual description of the gesture, in English
string english
# Textual description of the gesture, in Italian
string italian
")



(provide :speech_pkg/Command "3a48558522c26082d66040db118e6fd8")


