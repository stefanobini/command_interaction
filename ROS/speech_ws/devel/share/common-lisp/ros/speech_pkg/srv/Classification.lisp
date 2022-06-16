; Auto-generated. Do not edit!


(cl:in-package speech_pkg-srv)


;//! \htmlinclude Classification-request.msg.html

(cl:defclass <Classification-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type speech_pkg-msg:SpeechData
    :initform (cl:make-instance 'speech_pkg-msg:SpeechData)))
)

(cl:defclass Classification-request (<Classification-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Classification-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Classification-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<Classification-request> is deprecated: use speech_pkg-srv:Classification-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Classification-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:data-val is deprecated.  Use speech_pkg-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Classification-request>) ostream)
  "Serializes a message object of type '<Classification-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Classification-request>) istream)
  "Deserializes a message object of type '<Classification-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Classification-request>)))
  "Returns string type for a service object of type '<Classification-request>"
  "speech_pkg/ClassificationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Classification-request)))
  "Returns string type for a service object of type 'Classification-request"
  "speech_pkg/ClassificationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Classification-request>)))
  "Returns md5sum for a message object of type '<Classification-request>"
  "896849feb7849138871db12c46e37d25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Classification-request)))
  "Returns md5sum for a message object of type 'Classification-request"
  "896849feb7849138871db12c46e37d25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Classification-request>)))
  "Returns full string definition for message of type '<Classification-request>"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Classification-request)))
  "Returns full string definition for message of type 'Classification-request"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Classification-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Classification-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Classification-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude Classification-response.msg.html

(cl:defclass <Classification-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Classification-response (<Classification-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Classification-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Classification-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<Classification-response> is deprecated: use speech_pkg-srv:Classification-response instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <Classification-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:cmd-val is deprecated.  Use speech_pkg-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'probs-val :lambda-list '(m))
(cl:defmethod probs-val ((m <Classification-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:probs-val is deprecated.  Use speech_pkg-srv:probs instead.")
  (probs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Classification-response>) ostream)
  "Serializes a message object of type '<Classification-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Classification-response>) istream)
  "Deserializes a message object of type '<Classification-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Classification-response>)))
  "Returns string type for a service object of type '<Classification-response>"
  "speech_pkg/ClassificationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Classification-response)))
  "Returns string type for a service object of type 'Classification-response"
  "speech_pkg/ClassificationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Classification-response>)))
  "Returns md5sum for a message object of type '<Classification-response>"
  "896849feb7849138871db12c46e37d25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Classification-response)))
  "Returns md5sum for a message object of type 'Classification-response"
  "896849feb7849138871db12c46e37d25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Classification-response>)))
  "Returns full string definition for message of type '<Classification-response>"
  (cl:format cl:nil "int8 cmd~%float32[] probs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Classification-response)))
  "Returns full string definition for message of type 'Classification-response"
  (cl:format cl:nil "int8 cmd~%float32[] probs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Classification-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'probs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Classification-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Classification-response
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':probs (probs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Classification)))
  'Classification-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Classification)))
  'Classification-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Classification)))
  "Returns string type for a service object of type '<Classification>"
  "speech_pkg/Classification")