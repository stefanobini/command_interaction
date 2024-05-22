; Auto-generated. Do not edit!


(cl:in-package speech_pkg-msg)


;//! \htmlinclude SystemHealth.msg.html

(cl:defclass <SystemHealth> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass SystemHealth (<SystemHealth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SystemHealth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SystemHealth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-msg:<SystemHealth> is deprecated: use speech_pkg-msg:SystemHealth instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SystemHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:id-val is deprecated.  Use speech_pkg-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SystemHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:type-val is deprecated.  Use speech_pkg-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <SystemHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:timestamp-val is deprecated.  Use speech_pkg-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SystemHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:status-val is deprecated.  Use speech_pkg-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SystemHealth>) ostream)
  "Serializes a message object of type '<SystemHealth>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'timestamp))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SystemHealth>) istream)
  "Deserializes a message object of type '<SystemHealth>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'timestamp) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SystemHealth>)))
  "Returns string type for a message object of type '<SystemHealth>"
  "speech_pkg/SystemHealth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SystemHealth)))
  "Returns string type for a message object of type 'SystemHealth"
  "speech_pkg/SystemHealth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SystemHealth>)))
  "Returns md5sum for a message object of type '<SystemHealth>"
  "2ca1632d366bfbb66f928108e6362e1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SystemHealth)))
  "Returns md5sum for a message object of type 'SystemHealth"
  "2ca1632d366bfbb66f928108e6362e1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SystemHealth>)))
  "Returns full string definition for message of type '<SystemHealth>"
  (cl:format cl:nil "# Identify the complete Entity/topic name (UNISA.SpeechGestureAnalysis.Speech)~%string id~%# Identify the Entity/topic type (Speech)~%string type~%#Two-integer timestamp that is expressed as:~%# * timestamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * timestamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%string timestamp~%# Show the status information~%string status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SystemHealth)))
  "Returns full string definition for message of type 'SystemHealth"
  (cl:format cl:nil "# Identify the complete Entity/topic name (UNISA.SpeechGestureAnalysis.Speech)~%string id~%# Identify the Entity/topic type (Speech)~%string type~%#Two-integer timestamp that is expressed as:~%# * timestamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * timestamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%string timestamp~%# Show the status information~%string status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SystemHealth>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'timestamp))
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SystemHealth>))
  "Converts a ROS message object to a list"
  (cl:list 'SystemHealth
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':status (status msg))
))
