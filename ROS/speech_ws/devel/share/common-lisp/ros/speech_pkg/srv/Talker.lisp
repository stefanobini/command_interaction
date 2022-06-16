; Auto-generated. Do not edit!


(cl:in-package speech_pkg-srv)


;//! \htmlinclude Talker-request.msg.html

(cl:defclass <Talker-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0)
   (probs
    :reader probs
    :initarg :probs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Talker-request (<Talker-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Talker-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Talker-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<Talker-request> is deprecated: use speech_pkg-srv:Talker-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <Talker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:cmd-val is deprecated.  Use speech_pkg-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'probs-val :lambda-list '(m))
(cl:defmethod probs-val ((m <Talker-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:probs-val is deprecated.  Use speech_pkg-srv:probs instead.")
  (probs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Talker-request>) ostream)
  "Serializes a message object of type '<Talker-request>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'probs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'probs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Talker-request>) istream)
  "Deserializes a message object of type '<Talker-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'probs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'probs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Talker-request>)))
  "Returns string type for a service object of type '<Talker-request>"
  "speech_pkg/TalkerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Talker-request)))
  "Returns string type for a service object of type 'Talker-request"
  "speech_pkg/TalkerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Talker-request>)))
  "Returns md5sum for a message object of type '<Talker-request>"
  "b542a23d9f2300b346a3653dfa7b8829")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Talker-request)))
  "Returns md5sum for a message object of type 'Talker-request"
  "b542a23d9f2300b346a3653dfa7b8829")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Talker-request>)))
  "Returns full string definition for message of type '<Talker-request>"
  (cl:format cl:nil "int8 cmd~%float32[] probs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Talker-request)))
  "Returns full string definition for message of type 'Talker-request"
  (cl:format cl:nil "int8 cmd~%float32[] probs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Talker-request>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'probs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Talker-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Talker-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':probs (probs msg))
))
;//! \htmlinclude Talker-response.msg.html

(cl:defclass <Talker-response> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Talker-response (<Talker-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Talker-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Talker-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<Talker-response> is deprecated: use speech_pkg-srv:Talker-response instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <Talker-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:flag-val is deprecated.  Use speech_pkg-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Talker-response>) ostream)
  "Serializes a message object of type '<Talker-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Talker-response>) istream)
  "Deserializes a message object of type '<Talker-response>"
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Talker-response>)))
  "Returns string type for a service object of type '<Talker-response>"
  "speech_pkg/TalkerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Talker-response)))
  "Returns string type for a service object of type 'Talker-response"
  "speech_pkg/TalkerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Talker-response>)))
  "Returns md5sum for a message object of type '<Talker-response>"
  "b542a23d9f2300b346a3653dfa7b8829")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Talker-response)))
  "Returns md5sum for a message object of type 'Talker-response"
  "b542a23d9f2300b346a3653dfa7b8829")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Talker-response>)))
  "Returns full string definition for message of type '<Talker-response>"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Talker-response)))
  "Returns full string definition for message of type 'Talker-response"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Talker-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Talker-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Talker-response
    (cl:cons ':flag (flag msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Talker)))
  'Talker-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Talker)))
  'Talker-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Talker)))
  "Returns string type for a service object of type '<Talker>"
  "speech_pkg/Talker")