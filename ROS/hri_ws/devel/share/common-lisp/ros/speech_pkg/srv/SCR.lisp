; Auto-generated. Do not edit!


(cl:in-package speech_pkg-srv)


;//! \htmlinclude SCR-request.msg.html

(cl:defclass <SCR-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type speech_pkg-msg:SpeechData
    :initform (cl:make-instance 'speech_pkg-msg:SpeechData)))
)

(cl:defclass SCR-request (<SCR-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SCR-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SCR-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<SCR-request> is deprecated: use speech_pkg-srv:SCR-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SCR-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:data-val is deprecated.  Use speech_pkg-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SCR-request>) ostream)
  "Serializes a message object of type '<SCR-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SCR-request>) istream)
  "Deserializes a message object of type '<SCR-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SCR-request>)))
  "Returns string type for a service object of type '<SCR-request>"
  "speech_pkg/SCRRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SCR-request)))
  "Returns string type for a service object of type 'SCR-request"
  "speech_pkg/SCRRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SCR-request>)))
  "Returns md5sum for a message object of type '<SCR-request>"
  "896849feb7849138871db12c46e37d25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SCR-request)))
  "Returns md5sum for a message object of type 'SCR-request"
  "896849feb7849138871db12c46e37d25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SCR-request>)))
  "Returns full string definition for message of type '<SCR-request>"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SCR-request)))
  "Returns full string definition for message of type 'SCR-request"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SCR-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SCR-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SCR-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SCR-response.msg.html

(cl:defclass <SCR-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SCR-response (<SCR-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SCR-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SCR-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<SCR-response> is deprecated: use speech_pkg-srv:SCR-response instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <SCR-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:cmd-val is deprecated.  Use speech_pkg-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'probs-val :lambda-list '(m))
(cl:defmethod probs-val ((m <SCR-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:probs-val is deprecated.  Use speech_pkg-srv:probs instead.")
  (probs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SCR-response>) ostream)
  "Serializes a message object of type '<SCR-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SCR-response>) istream)
  "Deserializes a message object of type '<SCR-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SCR-response>)))
  "Returns string type for a service object of type '<SCR-response>"
  "speech_pkg/SCRResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SCR-response)))
  "Returns string type for a service object of type 'SCR-response"
  "speech_pkg/SCRResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SCR-response>)))
  "Returns md5sum for a message object of type '<SCR-response>"
  "896849feb7849138871db12c46e37d25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SCR-response)))
  "Returns md5sum for a message object of type 'SCR-response"
  "896849feb7849138871db12c46e37d25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SCR-response>)))
  "Returns full string definition for message of type '<SCR-response>"
  (cl:format cl:nil "int8 cmd~%float32[] probs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SCR-response)))
  "Returns full string definition for message of type 'SCR-response"
  (cl:format cl:nil "int8 cmd~%float32[] probs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SCR-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'probs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SCR-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SCR-response
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':probs (probs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SCR)))
  'SCR-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SCR)))
  'SCR-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SCR)))
  "Returns string type for a service object of type '<SCR>"
  "speech_pkg/SCR")