; Auto-generated. Do not edit!


(cl:in-package speech_pkg-msg)


;//! \htmlinclude SpeechData.msg.html

(cl:defclass <SpeechData> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (doa
    :reader doa
    :initarg :doa
    :type cl:fixnum
    :initform 0)
   (start_time
    :reader start_time
    :initarg :start_time
    :type cl:float
    :initform 0.0)
   (end_time
    :reader end_time
    :initarg :end_time
    :type cl:float
    :initform 0.0))
)

(cl:defclass SpeechData (<SpeechData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeechData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeechData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speech_pkg-msg:<SpeechData> is deprecated: use speech_pkg-msg:SpeechData instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SpeechData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:data-val is deprecated.  Use speech_pkg-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'doa-val :lambda-list '(m))
(cl:defmethod doa-val ((m <SpeechData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:doa-val is deprecated.  Use speech_pkg-msg:doa instead.")
  (doa m))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <SpeechData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:start_time-val is deprecated.  Use speech_pkg-msg:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'end_time-val :lambda-list '(m))
(cl:defmethod end_time-val ((m <SpeechData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speech_pkg-msg:end_time-val is deprecated.  Use speech_pkg-msg:end_time instead.")
  (end_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeechData>) ostream)
  "Serializes a message object of type '<SpeechData>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
  (cl:let* ((signed (cl:slot-value msg 'doa)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'start_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'end_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeechData>) istream)
  "Deserializes a message object of type '<SpeechData>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'doa) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'start_time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'end_time) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeechData>)))
  "Returns string type for a message object of type '<SpeechData>"
  "speech_pkg/SpeechData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechData)))
  "Returns string type for a message object of type 'SpeechData"
  "speech_pkg/SpeechData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeechData>)))
  "Returns md5sum for a message object of type '<SpeechData>"
  "a2d258c719b9a4af008aff70e592c3a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeechData)))
  "Returns md5sum for a message object of type 'SpeechData"
  "a2d258c719b9a4af008aff70e592c3a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeechData>)))
  "Returns full string definition for message of type '<SpeechData>"
  (cl:format cl:nil "int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeechData)))
  "Returns full string definition for message of type 'SpeechData"
  (cl:format cl:nil "int16[] data~%int16 doa~%float64 start_time~%float64 end_time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeechData>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     2
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeechData>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeechData
    (cl:cons ':data (data msg))
    (cl:cons ':doa (doa msg))
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':end_time (end_time msg))
))
