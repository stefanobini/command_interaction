;; Auto-generated. Do not edit!


(when (boundp 'iri_object_transportation_msgs::narrowPathMarkersArray)
  (if (not (find-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
    (make-package "IRI_OBJECT_TRANSPORTATION_MSGS"))
  (shadow 'narrowPathMarkersArray (find-package "IRI_OBJECT_TRANSPORTATION_MSGS")))
(unless (find-package "IRI_OBJECT_TRANSPORTATION_MSGS::NARROWPATHMARKERSARRAY")
  (make-package "IRI_OBJECT_TRANSPORTATION_MSGS::NARROWPATHMARKERSARRAY"))

(in-package "ROS")
;;//! \htmlinclude narrowPathMarkersArray.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass iri_object_transportation_msgs::narrowPathMarkersArray
  :super ros::object
  :slots (_header _marker_valid_array _marker_pose_array _marker_wrench_array _marker_goal_array ))

(defmethod iri_object_transportation_msgs::narrowPathMarkersArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:marker_valid_array __marker_valid_array) (let (r) (dotimes (i 0) (push (instance std_msgs::Bool :init) r)) r))
    ((:marker_pose_array __marker_pose_array) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:marker_wrench_array __marker_wrench_array) (let (r) (dotimes (i 0) (push (instance geometry_msgs::WrenchStamped :init) r)) r))
    ((:marker_goal_array __marker_goal_array) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _marker_valid_array __marker_valid_array)
   (setq _marker_pose_array __marker_pose_array)
   (setq _marker_wrench_array __marker_wrench_array)
   (setq _marker_goal_array __marker_goal_array)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:marker_valid_array
   (&rest __marker_valid_array)
   (if (keywordp (car __marker_valid_array))
       (send* _marker_valid_array __marker_valid_array)
     (progn
       (if __marker_valid_array (setq _marker_valid_array (car __marker_valid_array)))
       _marker_valid_array)))
  (:marker_pose_array
   (&rest __marker_pose_array)
   (if (keywordp (car __marker_pose_array))
       (send* _marker_pose_array __marker_pose_array)
     (progn
       (if __marker_pose_array (setq _marker_pose_array (car __marker_pose_array)))
       _marker_pose_array)))
  (:marker_wrench_array
   (&rest __marker_wrench_array)
   (if (keywordp (car __marker_wrench_array))
       (send* _marker_wrench_array __marker_wrench_array)
     (progn
       (if __marker_wrench_array (setq _marker_wrench_array (car __marker_wrench_array)))
       _marker_wrench_array)))
  (:marker_goal_array
   (&rest __marker_goal_array)
   (if (keywordp (car __marker_goal_array))
       (send* _marker_goal_array __marker_goal_array)
     (progn
       (if __marker_goal_array (setq _marker_goal_array (car __marker_goal_array)))
       _marker_goal_array)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Bool[] _marker_valid_array
    (apply #'+ (send-all _marker_valid_array :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _marker_pose_array
    (apply #'+ (send-all _marker_pose_array :serialization-length)) 4
    ;; geometry_msgs/WrenchStamped[] _marker_wrench_array
    (apply #'+ (send-all _marker_wrench_array :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _marker_goal_array
    (apply #'+ (send-all _marker_goal_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Bool[] _marker_valid_array
     (write-long (length _marker_valid_array) s)
     (dolist (elem _marker_valid_array)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseStamped[] _marker_pose_array
     (write-long (length _marker_pose_array) s)
     (dolist (elem _marker_pose_array)
       (send elem :serialize s)
       )
     ;; geometry_msgs/WrenchStamped[] _marker_wrench_array
     (write-long (length _marker_wrench_array) s)
     (dolist (elem _marker_wrench_array)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseStamped[] _marker_goal_array
     (write-long (length _marker_goal_array) s)
     (dolist (elem _marker_goal_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Bool[] _marker_valid_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_valid_array (let (r) (dotimes (i n) (push (instance std_msgs::Bool :init) r)) r))
     (dolist (elem- _marker_valid_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _marker_pose_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_pose_array (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _marker_pose_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/WrenchStamped[] _marker_wrench_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_wrench_array (let (r) (dotimes (i n) (push (instance geometry_msgs::WrenchStamped :init) r)) r))
     (dolist (elem- _marker_wrench_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _marker_goal_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_goal_array (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _marker_goal_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get iri_object_transportation_msgs::narrowPathMarkersArray :md5sum-) "e651f252e9d70e2375713d846ee9b4ad")
(setf (get iri_object_transportation_msgs::narrowPathMarkersArray :datatype-) "iri_object_transportation_msgs/narrowPathMarkersArray")
(setf (get iri_object_transportation_msgs::narrowPathMarkersArray :definition-)
      "Header header
std_msgs/Bool[] marker_valid_array
geometry_msgs/PoseStamped[] marker_pose_array
geometry_msgs/WrenchStamped[] marker_wrench_array
geometry_msgs/PoseStamped[] marker_goal_array

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
MSG: std_msgs/Bool
bool data
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

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



(provide :iri_object_transportation_msgs/narrowPathMarkersArray "e651f252e9d70e2375713d846ee9b4ad")


