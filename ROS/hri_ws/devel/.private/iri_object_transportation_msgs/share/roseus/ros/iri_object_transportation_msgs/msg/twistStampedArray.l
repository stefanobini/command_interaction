;; Auto-generated. Do not edit!


(when (boundp 'iri_object_transportation_msgs::twistStampedArray)
  (if (not (find-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
    (make-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
  (shadow 'twistStampedArray (find-package "IRI_OBJECT_TRANSPORTATION_MSGS")))
(unless (find-package "IRI_OBJECT_TRANSPORTATION_MSGS::TWISTSTAMPEDARRAY")
  (make-package "IRI_OBJECT_TRANSPORTATION_MSGS::TWISTSTAMPEDARRAY"))

(in-package "ROS")
;;//! \htmlinclude twistStampedArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass iri_object_transportation_msgs::twistStampedArray
  :super ros::object
  :slots (_header _twist_array ))

(defmethod iri_object_transportation_msgs::twistStampedArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:twist_array __twist_array) (let (r) (dotimes (i 0) (push (instance iri_object_transportation_msgs::twistStamped :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _twist_array __twist_array)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:twist_array
   (&rest __twist_array)
   (if (keywordp (car __twist_array))
       (send* _twist_array __twist_array)
     (progn
       (if __twist_array (setq _twist_array (car __twist_array)))
       _twist_array)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; iri_object_transportation_msgs/twistStamped[] _twist_array
    (apply #'+ (send-all _twist_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; iri_object_transportation_msgs/twistStamped[] _twist_array
     (write-long (length _twist_array) s)
     (dolist (elem _twist_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; iri_object_transportation_msgs/twistStamped[] _twist_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _twist_array (let (r) (dotimes (i n) (push (instance iri_object_transportation_msgs::twistStamped :init) r)) r))
     (dolist (elem- _twist_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get iri_object_transportation_msgs::twistStampedArray :md5sum-) "03e348c69a5cfa2196a94fd09cb86416")
(setf (get iri_object_transportation_msgs::twistStampedArray :datatype-) "iri_object_transportation_msgs/twistStampedArray")
(setf (get iri_object_transportation_msgs::twistStampedArray :definition-)
      "std_msgs/Header header
iri_object_transportation_msgs/twistStamped[] twist_array

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
MSG: iri_object_transportation_msgs/twistStamped
std_msgs/Header header
geometry_msgs/Twist twist

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

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



(provide :iri_object_transportation_msgs/twistStampedArray "03e348c69a5cfa2196a94fd09cb86416")


