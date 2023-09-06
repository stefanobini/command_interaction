;; Auto-generated. Do not edit!


(when (boundp 'iri_object_transportation_msgs::localForcesCoefficients)
  (if (not (find-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
    (make-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
  (shadow 'localForcesCoefficients (find-package "IRI_OBJECT_TRANSPORTATION_MSGS")))
(unless (find-package "IRI_OBJECT_TRANSPORTATION_MSGS::LOCALFORCESCOEFFICIENTS")
  (make-package "IRI_OBJECT_TRANSPORTATION_MSGS::LOCALFORCESCOEFFICIENTS"))

(in-package "ROS")
;;//! \htmlinclude localForcesCoefficients.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass iri_object_transportation_msgs::localForcesCoefficients
  :super ros::object
  :slots (_header _goal_coefficient _attractor_coefficient _obstacles_coefficients _total_coefficient ))

(defmethod iri_object_transportation_msgs::localForcesCoefficients
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:goal_coefficient __goal_coefficient) 0.0)
    ((:attractor_coefficient __attractor_coefficient) 0.0)
    ((:obstacles_coefficients __obstacles_coefficients) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:total_coefficient __total_coefficient) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _goal_coefficient (float __goal_coefficient))
   (setq _attractor_coefficient (float __attractor_coefficient))
   (setq _obstacles_coefficients __obstacles_coefficients)
   (setq _total_coefficient (float __total_coefficient))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:goal_coefficient
   (&optional __goal_coefficient)
   (if __goal_coefficient (setq _goal_coefficient __goal_coefficient)) _goal_coefficient)
  (:attractor_coefficient
   (&optional __attractor_coefficient)
   (if __attractor_coefficient (setq _attractor_coefficient __attractor_coefficient)) _attractor_coefficient)
  (:obstacles_coefficients
   (&optional __obstacles_coefficients)
   (if __obstacles_coefficients (setq _obstacles_coefficients __obstacles_coefficients)) _obstacles_coefficients)
  (:total_coefficient
   (&optional __total_coefficient)
   (if __total_coefficient (setq _total_coefficient __total_coefficient)) _total_coefficient)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _goal_coefficient
    4
    ;; float32 _attractor_coefficient
    4
    ;; float32[] _obstacles_coefficients
    (* 4    (length _obstacles_coefficients)) 4
    ;; float32 _total_coefficient
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _goal_coefficient
       (sys::poke _goal_coefficient (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _attractor_coefficient
       (sys::poke _attractor_coefficient (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _obstacles_coefficients
     (write-long (length _obstacles_coefficients) s)
     (dotimes (i (length _obstacles_coefficients))
       (sys::poke (elt _obstacles_coefficients i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _total_coefficient
       (sys::poke _total_coefficient (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _goal_coefficient
     (setq _goal_coefficient (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _attractor_coefficient
     (setq _attractor_coefficient (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _obstacles_coefficients
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles_coefficients (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _obstacles_coefficients i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _total_coefficient
     (setq _total_coefficient (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get iri_object_transportation_msgs::localForcesCoefficients :md5sum-) "c9b5321a9227a9197e9c265bb9376d55")
(setf (get iri_object_transportation_msgs::localForcesCoefficients :datatype-) "iri_object_transportation_msgs/localForcesCoefficients")
(setf (get iri_object_transportation_msgs::localForcesCoefficients :definition-)
      "Header header
float32 goal_coefficient
float32 attractor_coefficient
float32[] obstacles_coefficients
float32 total_coefficient

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



(provide :iri_object_transportation_msgs/localForcesCoefficients "c9b5321a9227a9197e9c265bb9376d55")


