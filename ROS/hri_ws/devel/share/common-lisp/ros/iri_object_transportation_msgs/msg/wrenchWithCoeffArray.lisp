; Auto-generated. Do not edit!


(cl:in-package iri_object_transportation_msgs-msg)


;//! \htmlinclude wrenchWithCoeffArray.msg.html

(cl:defclass <wrenchWithCoeffArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (wrench_array
    :reader wrench_array
    :initarg :wrench_array
    :type (cl:vector iri_object_transportation_msgs-msg:wrenchStampedWithCoeff)
   :initform (cl:make-array 0 :element-type 'iri_object_transportation_msgs-msg:wrenchStampedWithCoeff :initial-element (cl:make-instance 'iri_object_transportation_msgs-msg:wrenchStampedWithCoeff))))
)

(cl:defclass wrenchWithCoeffArray (<wrenchWithCoeffArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wrenchWithCoeffArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wrenchWithCoeffArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_object_transportation_msgs-msg:<wrenchWithCoeffArray> is deprecated: use iri_object_transportation_msgs-msg:wrenchWithCoeffArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <wrenchWithCoeffArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:header-val is deprecated.  Use iri_object_transportation_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'wrench_array-val :lambda-list '(m))
(cl:defmethod wrench_array-val ((m <wrenchWithCoeffArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:wrench_array-val is deprecated.  Use iri_object_transportation_msgs-msg:wrench_array instead.")
  (wrench_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wrenchWithCoeffArray>) ostream)
  "Serializes a message object of type '<wrenchWithCoeffArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'wrench_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'wrench_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wrenchWithCoeffArray>) istream)
  "Deserializes a message object of type '<wrenchWithCoeffArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'wrench_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'wrench_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'iri_object_transportation_msgs-msg:wrenchStampedWithCoeff))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wrenchWithCoeffArray>)))
  "Returns string type for a message object of type '<wrenchWithCoeffArray>"
  "iri_object_transportation_msgs/wrenchWithCoeffArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wrenchWithCoeffArray)))
  "Returns string type for a message object of type 'wrenchWithCoeffArray"
  "iri_object_transportation_msgs/wrenchWithCoeffArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wrenchWithCoeffArray>)))
  "Returns md5sum for a message object of type '<wrenchWithCoeffArray>"
  "7ae070e6b909e88cd2009fda3a6faf85")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wrenchWithCoeffArray)))
  "Returns md5sum for a message object of type 'wrenchWithCoeffArray"
  "7ae070e6b909e88cd2009fda3a6faf85")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wrenchWithCoeffArray>)))
  "Returns full string definition for message of type '<wrenchWithCoeffArray>"
  (cl:format cl:nil "Header header~%iri_object_transportation_msgs/wrenchStampedWithCoeff[] wrench_array~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: iri_object_transportation_msgs/wrenchStampedWithCoeff~%geometry_msgs/WrenchStamped external_force~%float32 coefficient~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wrenchWithCoeffArray)))
  "Returns full string definition for message of type 'wrenchWithCoeffArray"
  (cl:format cl:nil "Header header~%iri_object_transportation_msgs/wrenchStampedWithCoeff[] wrench_array~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: iri_object_transportation_msgs/wrenchStampedWithCoeff~%geometry_msgs/WrenchStamped external_force~%float32 coefficient~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wrenchWithCoeffArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'wrench_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wrenchWithCoeffArray>))
  "Converts a ROS message object to a list"
  (cl:list 'wrenchWithCoeffArray
    (cl:cons ':header (header msg))
    (cl:cons ':wrench_array (wrench_array msg))
))
