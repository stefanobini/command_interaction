; Auto-generated. Do not edit!


(cl:in-package iri_object_transportation_msgs-msg)


;//! \htmlinclude wrenchStampedWithCoeff.msg.html

(cl:defclass <wrenchStampedWithCoeff> (roslisp-msg-protocol:ros-message)
  ((external_force
    :reader external_force
    :initarg :external_force
    :type geometry_msgs-msg:WrenchStamped
    :initform (cl:make-instance 'geometry_msgs-msg:WrenchStamped))
   (coefficient
    :reader coefficient
    :initarg :coefficient
    :type cl:float
    :initform 0.0))
)

(cl:defclass wrenchStampedWithCoeff (<wrenchStampedWithCoeff>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wrenchStampedWithCoeff>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wrenchStampedWithCoeff)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_object_transportation_msgs-msg:<wrenchStampedWithCoeff> is deprecated: use iri_object_transportation_msgs-msg:wrenchStampedWithCoeff instead.")))

(cl:ensure-generic-function 'external_force-val :lambda-list '(m))
(cl:defmethod external_force-val ((m <wrenchStampedWithCoeff>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:external_force-val is deprecated.  Use iri_object_transportation_msgs-msg:external_force instead.")
  (external_force m))

(cl:ensure-generic-function 'coefficient-val :lambda-list '(m))
(cl:defmethod coefficient-val ((m <wrenchStampedWithCoeff>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:coefficient-val is deprecated.  Use iri_object_transportation_msgs-msg:coefficient instead.")
  (coefficient m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wrenchStampedWithCoeff>) ostream)
  "Serializes a message object of type '<wrenchStampedWithCoeff>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'external_force) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'coefficient))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wrenchStampedWithCoeff>) istream)
  "Deserializes a message object of type '<wrenchStampedWithCoeff>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'external_force) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'coefficient) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wrenchStampedWithCoeff>)))
  "Returns string type for a message object of type '<wrenchStampedWithCoeff>"
  "iri_object_transportation_msgs/wrenchStampedWithCoeff")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wrenchStampedWithCoeff)))
  "Returns string type for a message object of type 'wrenchStampedWithCoeff"
  "iri_object_transportation_msgs/wrenchStampedWithCoeff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wrenchStampedWithCoeff>)))
  "Returns md5sum for a message object of type '<wrenchStampedWithCoeff>"
  "4f94a203c7491df5185f74fa3237bfc7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wrenchStampedWithCoeff)))
  "Returns md5sum for a message object of type 'wrenchStampedWithCoeff"
  "4f94a203c7491df5185f74fa3237bfc7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wrenchStampedWithCoeff>)))
  "Returns full string definition for message of type '<wrenchStampedWithCoeff>"
  (cl:format cl:nil "geometry_msgs/WrenchStamped external_force~%float32 coefficient~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wrenchStampedWithCoeff)))
  "Returns full string definition for message of type 'wrenchStampedWithCoeff"
  (cl:format cl:nil "geometry_msgs/WrenchStamped external_force~%float32 coefficient~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wrenchStampedWithCoeff>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'external_force))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wrenchStampedWithCoeff>))
  "Converts a ROS message object to a list"
  (cl:list 'wrenchStampedWithCoeff
    (cl:cons ':external_force (external_force msg))
    (cl:cons ':coefficient (coefficient msg))
))
