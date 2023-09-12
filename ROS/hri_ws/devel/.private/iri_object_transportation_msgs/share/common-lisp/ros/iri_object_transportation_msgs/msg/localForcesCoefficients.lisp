; Auto-generated. Do not edit!


(cl:in-package iri_object_transportation_msgs-msg)


;//! \htmlinclude localForcesCoefficients.msg.html

(cl:defclass <localForcesCoefficients> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_coefficient
    :reader goal_coefficient
    :initarg :goal_coefficient
    :type cl:float
    :initform 0.0)
   (attractor_coefficient
    :reader attractor_coefficient
    :initarg :attractor_coefficient
    :type cl:float
    :initform 0.0)
   (obstacles_coefficients
    :reader obstacles_coefficients
    :initarg :obstacles_coefficients
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (total_coefficient
    :reader total_coefficient
    :initarg :total_coefficient
    :type cl:float
    :initform 0.0))
)

(cl:defclass localForcesCoefficients (<localForcesCoefficients>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <localForcesCoefficients>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'localForcesCoefficients)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_object_transportation_msgs-msg:<localForcesCoefficients> is deprecated: use iri_object_transportation_msgs-msg:localForcesCoefficients instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <localForcesCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:header-val is deprecated.  Use iri_object_transportation_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_coefficient-val :lambda-list '(m))
(cl:defmethod goal_coefficient-val ((m <localForcesCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:goal_coefficient-val is deprecated.  Use iri_object_transportation_msgs-msg:goal_coefficient instead.")
  (goal_coefficient m))

(cl:ensure-generic-function 'attractor_coefficient-val :lambda-list '(m))
(cl:defmethod attractor_coefficient-val ((m <localForcesCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:attractor_coefficient-val is deprecated.  Use iri_object_transportation_msgs-msg:attractor_coefficient instead.")
  (attractor_coefficient m))

(cl:ensure-generic-function 'obstacles_coefficients-val :lambda-list '(m))
(cl:defmethod obstacles_coefficients-val ((m <localForcesCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:obstacles_coefficients-val is deprecated.  Use iri_object_transportation_msgs-msg:obstacles_coefficients instead.")
  (obstacles_coefficients m))

(cl:ensure-generic-function 'total_coefficient-val :lambda-list '(m))
(cl:defmethod total_coefficient-val ((m <localForcesCoefficients>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:total_coefficient-val is deprecated.  Use iri_object_transportation_msgs-msg:total_coefficient instead.")
  (total_coefficient m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <localForcesCoefficients>) ostream)
  "Serializes a message object of type '<localForcesCoefficients>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'goal_coefficient))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'attractor_coefficient))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles_coefficients))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'obstacles_coefficients))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_coefficient))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <localForcesCoefficients>) istream)
  "Deserializes a message object of type '<localForcesCoefficients>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_coefficient) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'attractor_coefficient) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles_coefficients) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles_coefficients)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_coefficient) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<localForcesCoefficients>)))
  "Returns string type for a message object of type '<localForcesCoefficients>"
  "iri_object_transportation_msgs/localForcesCoefficients")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'localForcesCoefficients)))
  "Returns string type for a message object of type 'localForcesCoefficients"
  "iri_object_transportation_msgs/localForcesCoefficients")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<localForcesCoefficients>)))
  "Returns md5sum for a message object of type '<localForcesCoefficients>"
  "c9b5321a9227a9197e9c265bb9376d55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'localForcesCoefficients)))
  "Returns md5sum for a message object of type 'localForcesCoefficients"
  "c9b5321a9227a9197e9c265bb9376d55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<localForcesCoefficients>)))
  "Returns full string definition for message of type '<localForcesCoefficients>"
  (cl:format cl:nil "Header header~%float32 goal_coefficient~%float32 attractor_coefficient~%float32[] obstacles_coefficients~%float32 total_coefficient~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'localForcesCoefficients)))
  "Returns full string definition for message of type 'localForcesCoefficients"
  (cl:format cl:nil "Header header~%float32 goal_coefficient~%float32 attractor_coefficient~%float32[] obstacles_coefficients~%float32 total_coefficient~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <localForcesCoefficients>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles_coefficients) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <localForcesCoefficients>))
  "Converts a ROS message object to a list"
  (cl:list 'localForcesCoefficients
    (cl:cons ':header (header msg))
    (cl:cons ':goal_coefficient (goal_coefficient msg))
    (cl:cons ':attractor_coefficient (attractor_coefficient msg))
    (cl:cons ':obstacles_coefficients (obstacles_coefficients msg))
    (cl:cons ':total_coefficient (total_coefficient msg))
))
