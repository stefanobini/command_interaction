; Auto-generated. Do not edit!


(cl:in-package speech_pkg-msg)


;//! \htmlinclude Speech.msg.html

(cl:defclass <Speech> (roslisp-msg-protocol:ros-message)
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
   (command
    :reader command
    :initarg :command
    :type speech_pkg-msg:Command
    :initform (cl:make-instance 'speech_pkg-msg:Command))
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass Speech (<Speech>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Speech>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Speech)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-msg:<Speech> is deprecated: use speech_pkg-msg:Speech instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Speech>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:id-val is deprecated.  Use speech_pkg-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Speech>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:type-val is deprecated.  Use speech_pkg-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <Speech>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:timestamp-val is deprecated.  Use speech_pkg-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <Speech>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:command-val is deprecated.  Use speech_pkg-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <Speech>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:confidence-val is deprecated.  Use speech_pkg-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Speech>) ostream)
  "Serializes a message object of type '<Speech>"
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Speech>) istream)
  "Deserializes a message object of type '<Speech>"
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Speech>)))
  "Returns string type for a message object of type '<Speech>"
  "speech_pkg/Speech")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speech)))
  "Returns string type for a message object of type 'Speech"
  "speech_pkg/Speech")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Speech>)))
  "Returns md5sum for a message object of type '<Speech>"
  "c31bef49fddac6ccf19a75e92a735ec5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Speech)))
  "Returns md5sum for a message object of type 'Speech"
  "c31bef49fddac6ccf19a75e92a735ec5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Speech>)))
  "Returns full string definition for message of type '<Speech>"
  (cl:format cl:nil "# ~%~%# Identify the complete Entity/topic name (UNISA.SpeechGestureAnalysis.Speech)~%string id~%# Identify the Entity/topic type (Speech)~%string type~%#Two-integer timestamp that is expressed as:~%# * timestamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * timestamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%string timestamp~%# Identify the command detected by Speech/Command system, it contains the following information:~%# * Numeric identifier associated with the command~%# * Textual description of the gesture, in English~%# * Textual description of the gesture, in Italian~%Command command~%# Value between 0 and 1 which indicates the reliability in the command identification and classification~%float32 confidence~%================================================================================~%MSG: speech_pkg/Command~%# Identify the command detected by Speech/Command system, it contains the following information:~%~%# Numeric identifier associated with the command~%uint32 label~%# Textual description of the gesture, in English~%string english~%# Textual description of the gesture, in Italian~%string italian~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Speech)))
  "Returns full string definition for message of type 'Speech"
  (cl:format cl:nil "# ~%~%# Identify the complete Entity/topic name (UNISA.SpeechGestureAnalysis.Speech)~%string id~%# Identify the Entity/topic type (Speech)~%string type~%#Two-integer timestamp that is expressed as:~%# * timestamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * timestamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%string timestamp~%# Identify the command detected by Speech/Command system, it contains the following information:~%# * Numeric identifier associated with the command~%# * Textual description of the gesture, in English~%# * Textual description of the gesture, in Italian~%Command command~%# Value between 0 and 1 which indicates the reliability in the command identification and classification~%float32 confidence~%================================================================================~%MSG: speech_pkg/Command~%# Identify the command detected by Speech/Command system, it contains the following information:~%~%# Numeric identifier associated with the command~%uint32 label~%# Textual description of the gesture, in English~%string english~%# Textual description of the gesture, in Italian~%string italian~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Speech>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'timestamp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Speech>))
  "Converts a ROS message object to a list"
  (cl:list 'Speech
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':command (command msg))
    (cl:cons ':confidence (confidence msg))
))
