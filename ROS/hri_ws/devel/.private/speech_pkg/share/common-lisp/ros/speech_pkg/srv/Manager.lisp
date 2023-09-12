; Auto-generated. Do not edit!


(cl:in-package speech_pkg-srv)


;//! \htmlinclude Manager-request.msg.html

(cl:defclass <Manager-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type speech_pkg-msg:SpeechData
    :initform (cl:make-instance 'speech_pkg-msg:SpeechData)))
)

(cl:defclass Manager-request (<Manager-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Manager-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Manager-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<Manager-request> is deprecated: use speech_pkg-srv:Manager-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Manager-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:data-val is deprecated.  Use speech_pkg-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Manager-request>) ostream)
  "Serializes a message object of type '<Manager-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Manager-request>) istream)
  "Deserializes a message object of type '<Manager-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Manager-request>)))
  "Returns string type for a service object of type '<Manager-request>"
  "speech_pkg/ManagerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Manager-request)))
  "Returns string type for a service object of type 'Manager-request"
  "speech_pkg/ManagerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Manager-request>)))
  "Returns md5sum for a message object of type '<Manager-request>"
  "426efa73e6c8d460d7543a8bf442644f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Manager-request)))
  "Returns md5sum for a message object of type 'Manager-request"
  "426efa73e6c8d460d7543a8bf442644f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Manager-request>)))
  "Returns full string definition for message of type '<Manager-request>"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Manager-request)))
  "Returns full string definition for message of type 'Manager-request"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Manager-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Manager-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Manager-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude Manager-response.msg.html

(cl:defclass <Manager-response> (roslisp-msg-protocol:ros-message)
  ((flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Manager-response (<Manager-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Manager-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Manager-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<Manager-response> is deprecated: use speech_pkg-srv:Manager-response instead.")))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <Manager-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:flag-val is deprecated.  Use speech_pkg-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Manager-response>) ostream)
  "Serializes a message object of type '<Manager-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Manager-response>) istream)
  "Deserializes a message object of type '<Manager-response>"
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Manager-response>)))
  "Returns string type for a service object of type '<Manager-response>"
  "speech_pkg/ManagerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Manager-response)))
  "Returns string type for a service object of type 'Manager-response"
  "speech_pkg/ManagerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Manager-response>)))
  "Returns md5sum for a message object of type '<Manager-response>"
  "426efa73e6c8d460d7543a8bf442644f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Manager-response)))
  "Returns md5sum for a message object of type 'Manager-response"
  "426efa73e6c8d460d7543a8bf442644f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Manager-response>)))
  "Returns full string definition for message of type '<Manager-response>"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Manager-response)))
  "Returns full string definition for message of type 'Manager-response"
  (cl:format cl:nil "bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Manager-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Manager-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Manager-response
    (cl:cons ':flag (flag msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Manager)))
  'Manager-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Manager)))
  'Manager-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Manager)))
  "Returns string type for a service object of type '<Manager>"
  "speech_pkg/Manager")