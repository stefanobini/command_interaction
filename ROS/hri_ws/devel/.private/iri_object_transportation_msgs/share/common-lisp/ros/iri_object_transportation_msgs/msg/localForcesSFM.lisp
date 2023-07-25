; Auto-generated. Do not edit!


(cl:in-package iri_object_transportation_msgs-msg)


;//! \htmlinclude localForcesSFM.msg.html

(cl:defclass <localForcesSFM> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_force
    :reader goal_force
    :initarg :goal_force
    :type geometry_msgs-msg:WrenchStamped
    :initform (cl:make-instance 'geometry_msgs-msg:WrenchStamped))
   (attractor_force
    :reader attractor_force
    :initarg :attractor_force
    :type geometry_msgs-msg:WrenchStamped
    :initform (cl:make-instance 'geometry_msgs-msg:WrenchStamped))
   (obstacles_forces
    :reader obstacles_forces
    :initarg :obstacles_forces
    :type iri_object_transportation_msgs-msg:wrenchStampedArray
    :initform (cl:make-instance 'iri_object_transportation_msgs-msg:wrenchStampedArray))
   (total_force
    :reader total_force
    :initarg :total_force
    :type geometry_msgs-msg:WrenchStamped
    :initform (cl:make-instance 'geometry_msgs-msg:WrenchStamped)))
)

(cl:defclass localForcesSFM (<localForcesSFM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <localForcesSFM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'localForcesSFM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_object_transportation_msgs-msg:<localForcesSFM> is deprecated: use iri_object_transportation_msgs-msg:localForcesSFM instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <localForcesSFM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:header-val is deprecated.  Use iri_object_transportation_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_force-val :lambda-list '(m))
(cl:defmethod goal_force-val ((m <localForcesSFM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:goal_force-val is deprecated.  Use iri_object_transportation_msgs-msg:goal_force instead.")
  (goal_force m))

(cl:ensure-generic-function 'attractor_force-val :lambda-list '(m))
(cl:defmethod attractor_force-val ((m <localForcesSFM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:attractor_force-val is deprecated.  Use iri_object_transportation_msgs-msg:attractor_force instead.")
  (attractor_force m))

(cl:ensure-generic-function 'obstacles_forces-val :lambda-list '(m))
(cl:defmethod obstacles_forces-val ((m <localForcesSFM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:obstacles_forces-val is deprecated.  Use iri_object_transportation_msgs-msg:obstacles_forces instead.")
  (obstacles_forces m))

(cl:ensure-generic-function 'total_force-val :lambda-list '(m))
(cl:defmethod total_force-val ((m <localForcesSFM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:total_force-val is deprecated.  Use iri_object_transportation_msgs-msg:total_force instead.")
  (total_force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <localForcesSFM>) ostream)
  "Serializes a message object of type '<localForcesSFM>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attractor_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obstacles_forces) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'total_force) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <localForcesSFM>) istream)
  "Deserializes a message object of type '<localForcesSFM>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attractor_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obstacles_forces) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'total_force) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<localForcesSFM>)))
  "Returns string type for a message object of type '<localForcesSFM>"
  "iri_object_transportation_msgs/localForcesSFM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'localForcesSFM)))
  "Returns string type for a message object of type 'localForcesSFM"
  "iri_object_transportation_msgs/localForcesSFM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<localForcesSFM>)))
  "Returns md5sum for a message object of type '<localForcesSFM>"
  "f1cd253465458d5a2c178d26ef88bcf8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'localForcesSFM)))
  "Returns md5sum for a message object of type 'localForcesSFM"
  "f1cd253465458d5a2c178d26ef88bcf8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<localForcesSFM>)))
  "Returns full string definition for message of type '<localForcesSFM>"
  (cl:format cl:nil "Header header~%geometry_msgs/WrenchStamped goal_force~%geometry_msgs/WrenchStamped attractor_force~%iri_object_transportation_msgs/wrenchStampedArray obstacles_forces~%geometry_msgs/WrenchStamped total_force~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: iri_object_transportation_msgs/wrenchStampedArray~%Header header~%geometry_msgs/WrenchStamped[] wrench_array~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'localForcesSFM)))
  "Returns full string definition for message of type 'localForcesSFM"
  (cl:format cl:nil "Header header~%geometry_msgs/WrenchStamped goal_force~%geometry_msgs/WrenchStamped attractor_force~%iri_object_transportation_msgs/wrenchStampedArray obstacles_forces~%geometry_msgs/WrenchStamped total_force~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: iri_object_transportation_msgs/wrenchStampedArray~%Header header~%geometry_msgs/WrenchStamped[] wrench_array~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <localForcesSFM>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attractor_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obstacles_forces))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'total_force))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <localForcesSFM>))
  "Converts a ROS message object to a list"
  (cl:list 'localForcesSFM
    (cl:cons ':header (header msg))
    (cl:cons ':goal_force (goal_force msg))
    (cl:cons ':attractor_force (attractor_force msg))
    (cl:cons ':obstacles_forces (obstacles_forces msg))
    (cl:cons ':total_force (total_force msg))
))
