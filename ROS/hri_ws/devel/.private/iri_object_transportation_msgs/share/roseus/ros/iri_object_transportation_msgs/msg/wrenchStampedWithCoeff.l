;; Auto-generated. Do not edit!


(when (boundp 'iri_object_transportation_msgs::wrenchStampedWithCoeff)
  (if (not (find-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
    (make-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
  (shadow 'wrenchStampedWithCoeff (find-package "IRI_OBJECT_TRANSPORTATION_MSGS")))
(unless (find-package "IRI_OBJECT_TRANSPORTATION_MSGS::WRENCHSTAMPEDWITHCOEFF")
  (make-package "IRI_OBJECT_TRANSPORTATION_MSGS::WRENCHSTAMPEDWITHCOEFF"))

(in-package "ROS")
;;//! \htmlinclude wrenchStampedWithCoeff.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass iri_object_transportation_msgs::wrenchStampedWithCoeff
  :super ros::object
  :slots (_external_force _coefficient ))

(defmethod iri_object_transportation_msgs::wrenchStampedWithCoeff
  (:init
   (&key
    ((:external_force __external_force) (instance geometry_msgs::WrenchStamped :init))
    ((:coefficient __coefficient) 0.0)
    )
   (send-super :init)
   (setq _external_force __external_force)
   (setq _coefficient (float __coefficient))
   self)
  (:external_force
   (&rest __external_force)
   (if (keywordp (car __external_force))
       (send* _external_force __external_force)
     (progn
       (if __external_force (setq _external_force (car __external_force)))
       _external_force)))
  (:coefficient
   (&optional __coefficient)
   (if __coefficient (setq _coefficient __coefficient)) _coefficient)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/WrenchStamped _external_force
    (send _external_force :serialization-length)
    ;; float32 _coefficient
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/WrenchStamped _external_force
       (send _external_force :serialize s)
     ;; float32 _coefficient
       (sys::poke _coefficient (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/WrenchStamped _external_force
     (send _external_force :deserialize buf ptr-) (incf ptr- (send _external_force :serialization-length))
   ;; float32 _coefficient
     (setq _coefficient (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get iri_object_transportation_msgs::wrenchStampedWithCoeff :md5sum-) "4f94a203c7491df5185f74fa3237bfc7")
(setf (get iri_object_transportation_msgs::wrenchStampedWithCoeff :datatype-) "iri_object_transportation_msgs/wrenchStampedWithCoeff")
(setf (get iri_object_transportation_msgs::wrenchStampedWithCoeff :definition-)
      "geometry_msgs/WrenchStamped external_force
float32 coefficient

================================================================================
MSG: geometry_msgs/WrenchStamped
# A wrench with reference coordinate frame and timestamp
Header header
Wrench wrench

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



(provide :iri_object_transportation_msgs/wrenchStampedWithCoeff "4f94a203c7491df5185f74fa3237bfc7")


