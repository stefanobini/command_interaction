; Auto-generated. Do not edit!


(cl:in-package iri_object_transportation_msgs-msg)


;//! \htmlinclude narrowPathMarkersArray.msg.html

(cl:defclass <narrowPathMarkersArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (marker_valid_array
    :reader marker_valid_array
    :initarg :marker_valid_array
    :type (cl:vector std_msgs-msg:Bool)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Bool :initial-element (cl:make-instance 'std_msgs-msg:Bool)))
   (marker_pose_array
    :reader marker_pose_array
    :initarg :marker_pose_array
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
   (marker_wrench_array
    :reader marker_wrench_array
    :initarg :marker_wrench_array
    :type (cl:vector geometry_msgs-msg:WrenchStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:WrenchStamped :initial-element (cl:make-instance 'geometry_msgs-msg:WrenchStamped)))
   (marker_goal_array
    :reader marker_goal_array
    :initarg :marker_goal_array
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped))))
)

(cl:defclass narrowPathMarkersArray (<narrowPathMarkersArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <narrowPathMarkersArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'narrowPathMarkersArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_object_transportation_msgs-msg:<narrowPathMarkersArray> is deprecated: use iri_object_transportation_msgs-msg:narrowPathMarkersArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <narrowPathMarkersArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:header-val is deprecated.  Use iri_object_transportation_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'marker_valid_array-val :lambda-list '(m))
(cl:defmethod marker_valid_array-val ((m <narrowPathMarkersArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:marker_valid_array-val is deprecated.  Use iri_object_transportation_msgs-msg:marker_valid_array instead.")
  (marker_valid_array m))

(cl:ensure-generic-function 'marker_pose_array-val :lambda-list '(m))
(cl:defmethod marker_pose_array-val ((m <narrowPathMarkersArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:marker_pose_array-val is deprecated.  Use iri_object_transportation_msgs-msg:marker_pose_array instead.")
  (marker_pose_array m))

(cl:ensure-generic-function 'marker_wrench_array-val :lambda-list '(m))
(cl:defmethod marker_wrench_array-val ((m <narrowPathMarkersArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:marker_wrench_array-val is deprecated.  Use iri_object_transportation_msgs-msg:marker_wrench_array instead.")
  (marker_wrench_array m))

(cl:ensure-generic-function 'marker_goal_array-val :lambda-list '(m))
(cl:defmethod marker_goal_array-val ((m <narrowPathMarkersArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:marker_goal_array-val is deprecated.  Use iri_object_transportation_msgs-msg:marker_goal_array instead.")
  (marker_goal_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <narrowPathMarkersArray>) ostream)
  "Serializes a message object of type '<narrowPathMarkersArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'marker_valid_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'marker_valid_array))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'marker_pose_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'marker_pose_array))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'marker_wrench_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'marker_wrench_array))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'marker_goal_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'marker_goal_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <narrowPathMarkersArray>) istream)
  "Deserializes a message object of type '<narrowPathMarkersArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'marker_valid_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'marker_valid_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Bool))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'marker_pose_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'marker_pose_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'marker_wrench_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'marker_wrench_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:WrenchStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'marker_goal_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'marker_goal_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<narrowPathMarkersArray>)))
  "Returns string type for a message object of type '<narrowPathMarkersArray>"
  "iri_object_transportation_msgs/narrowPathMarkersArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'narrowPathMarkersArray)))
  "Returns string type for a message object of type 'narrowPathMarkersArray"
  "iri_object_transportation_msgs/narrowPathMarkersArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<narrowPathMarkersArray>)))
  "Returns md5sum for a message object of type '<narrowPathMarkersArray>"
  "e651f252e9d70e2375713d846ee9b4ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'narrowPathMarkersArray)))
  "Returns md5sum for a message object of type 'narrowPathMarkersArray"
  "e651f252e9d70e2375713d846ee9b4ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<narrowPathMarkersArray>)))
  "Returns full string definition for message of type '<narrowPathMarkersArray>"
  (cl:format cl:nil "Header header~%std_msgs/Bool[] marker_valid_array~%geometry_msgs/PoseStamped[] marker_pose_array~%geometry_msgs/WrenchStamped[] marker_wrench_array~%geometry_msgs/PoseStamped[] marker_goal_array~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'narrowPathMarkersArray)))
  "Returns full string definition for message of type 'narrowPathMarkersArray"
  (cl:format cl:nil "Header header~%std_msgs/Bool[] marker_valid_array~%geometry_msgs/PoseStamped[] marker_pose_array~%geometry_msgs/WrenchStamped[] marker_wrench_array~%geometry_msgs/PoseStamped[] marker_goal_array~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <narrowPathMarkersArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'marker_valid_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'marker_pose_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'marker_wrench_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'marker_goal_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <narrowPathMarkersArray>))
  "Converts a ROS message object to a list"
  (cl:list 'narrowPathMarkersArray
    (cl:cons ':header (header msg))
    (cl:cons ':marker_valid_array (marker_valid_array msg))
    (cl:cons ':marker_pose_array (marker_pose_array msg))
    (cl:cons ':marker_wrench_array (marker_wrench_array msg))
    (cl:cons ':marker_goal_array (marker_goal_array msg))
))
