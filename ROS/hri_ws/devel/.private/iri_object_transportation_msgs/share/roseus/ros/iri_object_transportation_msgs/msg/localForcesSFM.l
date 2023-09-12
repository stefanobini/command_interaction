;; Auto-generated. Do not edit!


(when (boundp 'iri_object_transportation_msgs::localForcesSFM)
  (if (not (find-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
    (make-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
  (shadow 'localForcesSFM (find-package "IRI_OBJECT_TRANSPORTATION_MSGS")))
(unless (find-package "IRI_OBJECT_TRANSPORTATION_MSGS::LOCALFORCESSFM")
  (make-package "IRI_OBJECT_TRANSPORTATION_MSGS::LOCALFORCESSFM"))

(in-package "ROS")
;;//! \htmlinclude localForcesSFM.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass iri_object_transportation_msgs::localForcesSFM
  :super ros::object
  :slots (_header _goal_force _attractor_force _obstacles_forces _total_force ))

(defmethod iri_object_transportation_msgs::localForcesSFM
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:goal_force __goal_force) (instance geometry_msgs::WrenchStamped :init))
    ((:attractor_force __attractor_force) (instance geometry_msgs::WrenchStamped :init))
    ((:obstacles_forces __obstacles_forces) (instance iri_object_transportation_msgs::wrenchStampedArray :init))
    ((:total_force __total_force) (instance geometry_msgs::WrenchStamped :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _goal_force __goal_force)
   (setq _attractor_force __attractor_force)
   (setq _obstacles_forces __obstacles_forces)
   (setq _total_force __total_force)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:goal_force
   (&rest __goal_force)
   (if (keywordp (car __goal_force))
       (send* _goal_force __goal_force)
     (progn
       (if __goal_force (setq _goal_force (car __goal_force)))
       _goal_force)))
  (:attractor_force
   (&rest __attractor_force)
   (if (keywordp (car __attractor_force))
       (send* _attractor_force __attractor_force)
     (progn
       (if __attractor_force (setq _attractor_force (car __attractor_force)))
       _attractor_force)))
  (:obstacles_forces
   (&rest __obstacles_forces)
   (if (keywordp (car __obstacles_forces))
       (send* _obstacles_forces __obstacles_forces)
     (progn
       (if __obstacles_forces (setq _obstacles_forces (car __obstacles_forces)))
       _obstacles_forces)))
  (:total_force
   (&rest __total_force)
   (if (keywordp (car __total_force))
       (send* _total_force __total_force)
     (progn
       (if __total_force (setq _total_force (car __total_force)))
       _total_force)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/WrenchStamped _goal_force
    (send _goal_force :serialization-length)
    ;; geometry_msgs/WrenchStamped _attractor_force
    (send _attractor_force :serialization-length)
    ;; iri_object_transportation_msgs/wrenchStampedArray _obstacles_forces
    (send _obstacles_forces :serialization-length)
    ;; geometry_msgs/WrenchStamped _total_force
    (send _total_force :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/WrenchStamped _goal_force
       (send _goal_force :serialize s)
     ;; geometry_msgs/WrenchStamped _attractor_force
       (send _attractor_force :serialize s)
     ;; iri_object_transportation_msgs/wrenchStampedArray _obstacles_forces
       (send _obstacles_forces :serialize s)
     ;; geometry_msgs/WrenchStamped _total_force
       (send _total_force :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/WrenchStamped _goal_force
     (send _goal_force :deserialize buf ptr-) (incf ptr- (send _goal_force :serialization-length))
   ;; geometry_msgs/WrenchStamped _attractor_force
     (send _attractor_force :deserialize buf ptr-) (incf ptr- (send _attractor_force :serialization-length))
   ;; iri_object_transportation_msgs/wrenchStampedArray _obstacles_forces
     (send _obstacles_forces :deserialize buf ptr-) (incf ptr- (send _obstacles_forces :serialization-length))
   ;; geometry_msgs/WrenchStamped _total_force
     (send _total_force :deserialize buf ptr-) (incf ptr- (send _total_force :serialization-length))
   ;;
   self)
  )

(setf (get iri_object_transportation_msgs::localForcesSFM :md5sum-) "f1cd253465458d5a2c178d26ef88bcf8")
(setf (get iri_object_transportation_msgs::localForcesSFM :datatype-) "iri_object_transportation_msgs/localForcesSFM")
(setf (get iri_object_transportation_msgs::localForcesSFM :definition-)
      "Header header
geometry_msgs/WrenchStamped goal_force
geometry_msgs/WrenchStamped attractor_force
iri_object_transportation_msgs/wrenchStampedArray obstacles_forces
geometry_msgs/WrenchStamped total_force

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
================================================================================
MSG: iri_object_transportation_msgs/wrenchStampedArray
Header header
geometry_msgs/WrenchStamped[] wrench_array

")



(provide :iri_object_transportation_msgs/localForcesSFM "f1cd253465458d5a2c178d26ef88bcf8")


