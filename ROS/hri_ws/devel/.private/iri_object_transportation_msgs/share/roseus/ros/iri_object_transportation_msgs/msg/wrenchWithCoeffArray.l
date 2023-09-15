;; Auto-generated. Do not edit!


(when (boundp 'iri_object_transportation_msgs::wrenchWithCoeffArray)
  (if (not (find-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
    (make-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
  (shadow 'wrenchWithCoeffArray (find-package "IRI_OBJECT_TRANSPORTATION_MSGS")))
(unless (find-package "IRI_OBJECT_TRANSPORTATION_MSGS::WRENCHWITHCOEFFARRAY")
  (make-package "IRI_OBJECT_TRANSPORTATION_MSGS::WRENCHWITHCOEFFARRAY"))

(in-package "ROS")
;;//! \htmlinclude wrenchWithCoeffArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass iri_object_transportation_msgs::wrenchWithCoeffArray
  :super ros::object
  :slots (_header _wrench_array ))

(defmethod iri_object_transportation_msgs::wrenchWithCoeffArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:wrench_array __wrench_array) (let (r) (dotimes (i 0) (push (instance iri_object_transportation_msgs::wrenchStampedWithCoeff :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _wrench_array __wrench_array)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:wrench_array
   (&rest __wrench_array)
   (if (keywordp (car __wrench_array))
       (send* _wrench_array __wrench_array)
     (progn
       (if __wrench_array (setq _wrench_array (car __wrench_array)))
       _wrench_array)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; iri_object_transportation_msgs/wrenchStampedWithCoeff[] _wrench_array
    (apply #'+ (send-all _wrench_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; iri_object_transportation_msgs/wrenchStampedWithCoeff[] _wrench_array
     (write-long (length _wrench_array) s)
     (dolist (elem _wrench_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; iri_object_transportation_msgs/wrenchStampedWithCoeff[] _wrench_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wrench_array (let (r) (dotimes (i n) (push (instance iri_object_transportation_msgs::wrenchStampedWithCoeff :init) r)) r))
     (dolist (elem- _wrench_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get iri_object_transportation_msgs::wrenchWithCoeffArray :md5sum-) "7ae070e6b909e88cd2009fda3a6faf85")
(setf (get iri_object_transportation_msgs::wrenchWithCoeffArray :datatype-) "iri_object_transportation_msgs/wrenchWithCoeffArray")
(setf (get iri_object_transportation_msgs::wrenchWithCoeffArray :definition-)
      "Header header
iri_object_transportation_msgs/wrenchStampedWithCoeff[] wrench_array

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

================================================================================
MSG: iri_object_transportation_msgs/wrenchStampedWithCoeff
geometry_msgs/WrenchStamped external_force
float32 coefficient

================================================================================
MSG: geometry_msgs/WrenchStamped
# A wrench with reference coordinate frame and timestamp
Header header
Wrench wrench

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :iri_object_transportation_msgs/wrenchWithCoeffArray "7ae070e6b909e88cd2009fda3a6faf85")


