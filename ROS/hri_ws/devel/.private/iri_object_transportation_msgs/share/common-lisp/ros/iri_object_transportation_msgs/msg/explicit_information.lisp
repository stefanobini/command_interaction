; Auto-generated. Do not edit!


(cl:in-package iri_object_transportation_msgs-msg)


;//! \htmlinclude explicit_information.msg.html

(cl:defclass <explicit_information> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (fsr_values
    :reader fsr_values
    :initarg :fsr_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 5 :element-type 'cl:float :initial-element 0.0))
   (sw_values
    :reader sw_values
    :initarg :sw_values
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 5 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass explicit_information (<explicit_information>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <explicit_information>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'explicit_information)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name iri_object_transportation_msgs-msg:<explicit_information> is deprecated: use iri_object_transportation_msgs-msg:explicit_information instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <explicit_information>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:header-val is deprecated.  Use iri_object_transportation_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'fsr_values-val :lambda-list '(m))
(cl:defmethod fsr_values-val ((m <explicit_information>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:fsr_values-val is deprecated.  Use iri_object_transportation_msgs-msg:fsr_values instead.")
  (fsr_values m))

(cl:ensure-generic-function 'sw_values-val :lambda-list '(m))
(cl:defmethod sw_values-val ((m <explicit_information>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader iri_object_transportation_msgs-msg:sw_values-val is deprecated.  Use iri_object_transportation_msgs-msg:sw_values instead.")
  (sw_values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <explicit_information>) ostream)
  "Serializes a message object of type '<explicit_information>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'fsr_values))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'sw_values))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <explicit_information>) istream)
  "Deserializes a message object of type '<explicit_information>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'fsr_values) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'fsr_values)))
    (cl:dotimes (i 5)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'sw_values) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'sw_values)))
    (cl:dotimes (i 5)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<explicit_information>)))
  "Returns string type for a message object of type '<explicit_information>"
  "iri_object_transportation_msgs/explicit_information")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'explicit_information)))
  "Returns string type for a message object of type 'explicit_information"
  "iri_object_transportation_msgs/explicit_information")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<explicit_information>)))
  "Returns md5sum for a message object of type '<explicit_information>"
  "62dc5f03f1a505652316491d926de860")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'explicit_information)))
  "Returns md5sum for a message object of type 'explicit_information"
  "62dc5f03f1a505652316491d926de860")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<explicit_information>)))
  "Returns full string definition for message of type '<explicit_information>"
  (cl:format cl:nil "Header header~%float64[5] fsr_values~%bool[5] sw_values~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'explicit_information)))
  "Returns full string definition for message of type 'explicit_information"
  (cl:format cl:nil "Header header~%float64[5] fsr_values~%bool[5] sw_values~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <explicit_information>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'fsr_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'sw_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <explicit_information>))
  "Converts a ROS message object to a list"
  (cl:list 'explicit_information
    (cl:cons ':header (header msg))
    (cl:cons ':fsr_values (fsr_values msg))
    (cl:cons ':sw_values (sw_values msg))
))
