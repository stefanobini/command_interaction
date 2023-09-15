; Auto-generated. Do not edit!


(cl:in-package speech_pkg-srv)


;//! \htmlinclude ClassificationMSI-request.msg.html

(cl:defclass <ClassificationMSI-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type speech_pkg-msg:SpeechData
    :initform (cl:make-instance 'speech_pkg-msg:SpeechData)))
)

(cl:defclass ClassificationMSI-request (<ClassificationMSI-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClassificationMSI-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClassificationMSI-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<ClassificationMSI-request> is deprecated: use speech_pkg-srv:ClassificationMSI-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ClassificationMSI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:data-val is deprecated.  Use speech_pkg-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClassificationMSI-request>) ostream)
  "Serializes a message object of type '<ClassificationMSI-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClassificationMSI-request>) istream)
  "Deserializes a message object of type '<ClassificationMSI-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClassificationMSI-request>)))
  "Returns string type for a service object of type '<ClassificationMSI-request>"
  "speech_pkg/ClassificationMSIRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClassificationMSI-request)))
  "Returns string type for a service object of type 'ClassificationMSI-request"
  "speech_pkg/ClassificationMSIRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClassificationMSI-request>)))
  "Returns md5sum for a message object of type '<ClassificationMSI-request>"
  "86bdd504432c8f4cd29e4aa496bd70a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClassificationMSI-request)))
  "Returns md5sum for a message object of type 'ClassificationMSI-request"
  "86bdd504432c8f4cd29e4aa496bd70a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClassificationMSI-request>)))
  "Returns full string definition for message of type '<ClassificationMSI-request>"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClassificationMSI-request)))
  "Returns full string definition for message of type 'ClassificationMSI-request"
  (cl:format cl:nil "speech_pkg/SpeechData data~%~%================================================================================~%MSG: speech_pkg/SpeechData~%int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClassificationMSI-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClassificationMSI-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ClassificationMSI-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude ClassificationMSI-response.msg.html

(cl:defclass <ClassificationMSI-response> (roslisp-msg-protocol:ros-message)
  ((intent
    :reader intent
    :initarg :intent
    :type cl:fixnum
    :initform 0)
   (int_probs
    :reader int_probs
    :initarg :int_probs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (explicit
    :reader explicit
    :initarg :explicit
    :type cl:fixnum
    :initform 0)
   (exp_probs
    :reader exp_probs
    :initarg :exp_probs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (implicit
    :reader implicit
    :initarg :implicit
    :type cl:fixnum
    :initform 0)
   (imp_probs
    :reader imp_probs
    :initarg :imp_probs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ClassificationMSI-response (<ClassificationMSI-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClassificationMSI-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClassificationMSI-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-srv:<ClassificationMSI-response> is deprecated: use speech_pkg-srv:ClassificationMSI-response instead.")))

(cl:ensure-generic-function 'intent-val :lambda-list '(m))
(cl:defmethod intent-val ((m <ClassificationMSI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:intent-val is deprecated.  Use speech_pkg-srv:intent instead.")
  (intent m))

(cl:ensure-generic-function 'int_probs-val :lambda-list '(m))
(cl:defmethod int_probs-val ((m <ClassificationMSI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:int_probs-val is deprecated.  Use speech_pkg-srv:int_probs instead.")
  (int_probs m))

(cl:ensure-generic-function 'explicit-val :lambda-list '(m))
(cl:defmethod explicit-val ((m <ClassificationMSI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:explicit-val is deprecated.  Use speech_pkg-srv:explicit instead.")
  (explicit m))

(cl:ensure-generic-function 'exp_probs-val :lambda-list '(m))
(cl:defmethod exp_probs-val ((m <ClassificationMSI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:exp_probs-val is deprecated.  Use speech_pkg-srv:exp_probs instead.")
  (exp_probs m))

(cl:ensure-generic-function 'implicit-val :lambda-list '(m))
(cl:defmethod implicit-val ((m <ClassificationMSI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:implicit-val is deprecated.  Use speech_pkg-srv:implicit instead.")
  (implicit m))

(cl:ensure-generic-function 'imp_probs-val :lambda-list '(m))
(cl:defmethod imp_probs-val ((m <ClassificationMSI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-srv:imp_probs-val is deprecated.  Use speech_pkg-srv:imp_probs instead.")
  (imp_probs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClassificationMSI-response>) ostream)
  "Serializes a message object of type '<ClassificationMSI-response>"
  (cl:let* ((signed (cl:slot-value msg 'intent)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'int_probs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'int_probs))
  (cl:let* ((signed (cl:slot-value msg 'explicit)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'exp_probs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'exp_probs))
  (cl:let* ((signed (cl:slot-value msg 'implicit)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'imp_probs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'imp_probs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClassificationMSI-response>) istream)
  "Deserializes a message object of type '<ClassificationMSI-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'intent) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'int_probs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'int_probs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'explicit) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'exp_probs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'exp_probs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'implicit) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'imp_probs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'imp_probs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClassificationMSI-response>)))
  "Returns string type for a service object of type '<ClassificationMSI-response>"
  "speech_pkg/ClassificationMSIResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClassificationMSI-response)))
  "Returns string type for a service object of type 'ClassificationMSI-response"
  "speech_pkg/ClassificationMSIResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClassificationMSI-response>)))
  "Returns md5sum for a message object of type '<ClassificationMSI-response>"
  "86bdd504432c8f4cd29e4aa496bd70a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClassificationMSI-response)))
  "Returns md5sum for a message object of type 'ClassificationMSI-response"
  "86bdd504432c8f4cd29e4aa496bd70a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClassificationMSI-response>)))
  "Returns full string definition for message of type '<ClassificationMSI-response>"
  (cl:format cl:nil "int8 intent~%float32[] int_probs~%int8 explicit~%float32[] exp_probs~%int8 implicit~%float32[] imp_probs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClassificationMSI-response)))
  "Returns full string definition for message of type 'ClassificationMSI-response"
  (cl:format cl:nil "int8 intent~%float32[] int_probs~%int8 explicit~%float32[] exp_probs~%int8 implicit~%float32[] imp_probs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClassificationMSI-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'int_probs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'exp_probs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'imp_probs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClassificationMSI-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ClassificationMSI-response
    (cl:cons ':intent (intent msg))
    (cl:cons ':int_probs (int_probs msg))
    (cl:cons ':explicit (explicit msg))
    (cl:cons ':exp_probs (exp_probs msg))
    (cl:cons ':implicit (implicit msg))
    (cl:cons ':imp_probs (imp_probs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ClassificationMSI)))
  'ClassificationMSI-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ClassificationMSI)))
  'ClassificationMSI-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClassificationMSI)))
  "Returns string type for a service object of type '<ClassificationMSI>"
  "speech_pkg/ClassificationMSI")