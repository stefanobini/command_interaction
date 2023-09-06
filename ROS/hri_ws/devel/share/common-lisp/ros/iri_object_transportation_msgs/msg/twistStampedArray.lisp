; Auto-generated. Do not edit!


(cl:in-package iri_object_transportation_msgs-msg)


;//! \htmlinclude twistStampedArray.msg.html

(cl:defclass <twistStampedArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (twist_array
    :reader twist_array
    :initarg :twist_array
    :type (cl:vector iri_object_transportation_msgs-msg:twistStamped)
   :initform (cl:make-array 0 :element-type 'iri_object_transportation_msgs-msg:twistStamped :initial-element (cl:make-instance 'iri_object_transportation_msgs-msg:twistStamped))))
)

(cl:defclass twistStampedArray (<twistStampedArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <twistStampedArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'twistStampedArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_object_transportation_msgs-msg:<twistStampedArray> is deprecated: use iri_object_transportation_msgs-msg:twistStampedArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <twistStampedArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:header-val is deprecated.  Use iri_object_transportation_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'twist_array-val :lambda-list '(m))
(cl:defmethod twist_array-val ((m <twistStampedArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:twist_array-val is deprecated.  Use iri_object_transportation_msgs-msg:twist_array instead.")
  (twist_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <twistStampedArray>) ostream)
  "Serializes a message object of type '<twistStampedArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'twist_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'twist_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <twistStampedArray>) istream)
  "Deserializes a message object of type '<twistStampedArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'twist_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'twist_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'iri_object_transportation_msgs-msg:twistStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<twistStampedArray>)))
  "Returns string type for a message object of type '<twistStampedArray>"
  "iri_object_transportation_msgs/twistStampedArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'twistStampedArray)))
  "Returns string type for a message object of type 'twistStampedArray"
  "iri_object_transportation_msgs/twistStampedArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<twistStampedArray>)))
  "Returns md5sum for a message object of type '<twistStampedArray>"
  "03e348c69a5cfa2196a94fd09cb86416")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'twistStampedArray)))
  "Returns md5sum for a message object of type 'twistStampedArray"
  "03e348c69a5cfa2196a94fd09cb86416")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<twistStampedArray>)))
  "Returns full string definition for message of type '<twistStampedArray>"
  (cl:format cl:nil "std_msgs/Header header~%iri_object_transportation_msgs/twistStamped[] twist_array~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: iri_object_transportation_msgs/twistStamped~%std_msgs/Header header~%geometry_msgs/Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'twistStampedArray)))
  "Returns full string definition for message of type 'twistStampedArray"
  (cl:format cl:nil "std_msgs/Header header~%iri_object_transportation_msgs/twistStamped[] twist_array~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: iri_object_transportation_msgs/twistStamped~%std_msgs/Header header~%geometry_msgs/Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <twistStampedArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'twist_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <twistStampedArray>))
  "Converts a ROS message object to a list"
  (cl:list 'twistStampedArray
    (cl:cons ':header (header msg))
    (cl:cons ':twist_array (twist_array msg))
))
