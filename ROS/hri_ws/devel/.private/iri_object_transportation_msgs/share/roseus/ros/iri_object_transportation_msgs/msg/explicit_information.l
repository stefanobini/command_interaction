;; Auto-generated. Do not edit!


(when (boundp 'iri_object_transportation_msgs::explicit_information)
  (if (not (find-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
    (make-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
  (shadow 'explicit_information (find-package "IRI_OBJECT_TRANSPORTATION_MSGS")))
(unless (find-package "IRI_OBJECT_TRANSPORTATION_MSGS::EXPLICIT_INFORMATION")
  (make-package "IRI_OBJECT_TRANSPORTATION_MSGS::EXPLICIT_INFORMATION"))

(in-package "ROS")
;;//! \htmlinclude explicit_information.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass iri_object_transportation_msgs::explicit_information
  :super ros::object
  :slots (_header _fsr_values _sw_values ))

(defmethod iri_object_transportation_msgs::explicit_information
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:fsr_values __fsr_values) (make-array 5 :initial-element 0.0 :element-type :float))
    ((:sw_values __sw_values) (let (r) (dotimes (i 5) (push nil r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _fsr_values __fsr_values)
   (setq _sw_values __sw_values)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:fsr_values
   (&optional __fsr_values)
   (if __fsr_values (setq _fsr_values __fsr_values)) _fsr_values)
  (:sw_values
   (&optional __sw_values)
   (if __sw_values (setq _sw_values __sw_values)) _sw_values)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[5] _fsr_values
    (* 8    5)
    ;; bool[5] _sw_values
    (* 1    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[5] _fsr_values
     (dotimes (i 5)
       (sys::poke (elt _fsr_values i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool[5] _sw_values
     (dotimes (i 5)
       (if (elt _sw_values i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[5] _fsr_values
   (dotimes (i (length _fsr_values))
     (setf (elt _fsr_values i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool[5] _sw_values
   (dotimes (i (length _sw_values))
     (setf (elt _sw_values i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;;
   self)
  )

(setf (get iri_object_transportation_msgs::explicit_information :md5sum-) "62dc5f03f1a505652316491d926de860")
(setf (get iri_object_transportation_msgs::explicit_information :datatype-) "iri_object_transportation_msgs/explicit_information")
(setf (get iri_object_transportation_msgs::explicit_information :definition-)
      "Header header
float64[5] fsr_values
bool[5] sw_values

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :iri_object_transportation_msgs/explicit_information "62dc5f03f1a505652316491d926de860")


