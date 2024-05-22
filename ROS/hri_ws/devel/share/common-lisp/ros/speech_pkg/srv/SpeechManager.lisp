; Auto-generated. Do not edit!


(cl:in-package speech_pkg-srv)


;//! \htmlinclude SpeechManager-request.msg.html

(cl:defclass <SpeechManager-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type speech_pkg-msg:SpeechData
    :initform (cl:make-instance 'speech_pkg-msg:SpeechData)))
)

(cl:defclass SpeechManager-request (<SpeechManager-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeechManager-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeechManager-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<SpeechManager-request> is deprecated: use speech_pkg-srv:SpeechManager-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SpeechManager-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:data-val is deprecated.  Use speech_pkg-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeechManager-request>) ostream)
  "Serializes a message object of type '<SpeechManager-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeechManager-request>) istream)
  "Deserializes a message object of type '<SpeechManager-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeechManager-request>)))
  "Returns string type for a service object of type '<SpeechManager-request>"
  "speech_pkg/SpeechManagerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechManager-request)))
  "Returns string type for a service object of type 'SpeechManager-request"
  "speech_pkg/SpeechManagerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeechManager-request>)))
  "Returns md5sum for a message object of type '<SpeechManager-request>"
  "426efa73e6c8d460d7543a8bf442644f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeechManager-request)))
  "Returns md5sum for a message object of type 'SpeechManager-request"
  "426efa73e6c8d460d7543a8bf442644f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeechManager-request>)))
  "Returns full string definition for message of type '<SpeechManager-request>"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeechManager-request)))
  "Returns full string definition for message of type 'SpeechManager-request"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeechManager-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeechManager-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeechManager-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SpeechManager-response.msg.html

(cl:defclass <SpeechManager-response> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SpeechManager-response (<SpeechManager-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeechManager-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeechManager-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<SpeechManager-response> is deprecated: use speech_pkg-srv:SpeechManager-response instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <SpeechManager-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:flag-val is deprecated.  Use speech_pkg-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeechManager-response>) ostream)
  "Serializes a message object of type '<SpeechManager-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeechManager-response>) istream)
  "Deserializes a message object of type '<SpeechManager-response>"
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeechManager-response>)))
  "Returns string type for a service object of type '<SpeechManager-response>"
  "speech_pkg/SpeechManagerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechManager-response)))
  "Returns string type for a service object of type 'SpeechManager-response"
  "speech_pkg/SpeechManagerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeechManager-response>)))
  "Returns md5sum for a message object of type '<SpeechManager-response>"
  "426efa73e6c8d460d7543a8bf442644f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeechManager-response)))
  "Returns md5sum for a message object of type 'SpeechManager-response"
  "426efa73e6c8d460d7543a8bf442644f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeechManager-response>)))
  "Returns full string definition for message of type '<SpeechManager-response>"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeechManager-response)))
  "Returns full string definition for message of type 'SpeechManager-response"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeechManager-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeechManager-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeechManager-response
    (cl:cons ':flag (flag msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SpeechManager)))
  'SpeechManager-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SpeechManager)))
  'SpeechManager-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechManager)))
  "Returns string type for a service object of type '<SpeechManager>"
  "speech_pkg/SpeechManager")