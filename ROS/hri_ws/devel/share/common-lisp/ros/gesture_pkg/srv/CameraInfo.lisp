; Auto-generated. Do not edit!


(cl:in-package gesture_pkg-srv)


;//! \htmlinclude CameraInfo-request.msg.html

(cl:defclass <CameraInfo-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CameraInfo-request (<CameraInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gesture_pkg-srv:<CameraInfo-request> is deprecated: use gesture_pkg-srv:CameraInfo-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraInfo-request>) ostream)
  "Serializes a message object of type '<CameraInfo-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraInfo-request>) istream)
  "Deserializes a message object of type '<CameraInfo-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraInfo-request>)))
  "Returns string type for a service object of type '<CameraInfo-request>"
  "gesture_pkg/CameraInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraInfo-request)))
  "Returns string type for a service object of type 'CameraInfo-request"
  "gesture_pkg/CameraInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraInfo-request>)))
  "Returns md5sum for a message object of type '<CameraInfo-request>"
  "1e0f6cc9c6dfad0bee4253b15158a925")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraInfo-request)))
  "Returns md5sum for a message object of type 'CameraInfo-request"
  "1e0f6cc9c6dfad0bee4253b15158a925")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraInfo-request>)))
  "Returns full string definition for message of type '<CameraInfo-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraInfo-request)))
  "Returns full string definition for message of type 'CameraInfo-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraInfo-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraInfo-request
))
;//! \htmlinclude CameraInfo-response.msg.html

(cl:defclass <CameraInfo-response> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (hfov
    :reader hfov
    :initarg :hfov
    :type cl:float
    :initform 0.0)
   (vfov
    :reader vfov
    :initarg :vfov
    :type cl:float
    :initform 0.0))
)

(cl:defclass CameraInfo-response (<CameraInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gesture_pkg-srv:<CameraInfo-response> is deprecated: use gesture_pkg-srv:CameraInfo-response instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <CameraInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gesture_pkg-srv:width-val is deprecated.  Use gesture_pkg-srv:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <CameraInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gesture_pkg-srv:height-val is deprecated.  Use gesture_pkg-srv:height instead.")
  (height m))

(cl:ensure-generic-function 'hfov-val :lambda-list '(m))
(cl:defmethod hfov-val ((m <CameraInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gesture_pkg-srv:hfov-val is deprecated.  Use gesture_pkg-srv:hfov instead.")
  (hfov m))

(cl:ensure-generic-function 'vfov-val :lambda-list '(m))
(cl:defmethod vfov-val ((m <CameraInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gesture_pkg-srv:vfov-val is deprecated.  Use gesture_pkg-srv:vfov instead.")
  (vfov m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraInfo-response>) ostream)
  "Serializes a message object of type '<CameraInfo-response>"
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hfov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vfov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraInfo-response>) istream)
  "Deserializes a message object of type '<CameraInfo-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hfov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vfov) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraInfo-response>)))
  "Returns string type for a service object of type '<CameraInfo-response>"
  "gesture_pkg/CameraInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraInfo-response)))
  "Returns string type for a service object of type 'CameraInfo-response"
  "gesture_pkg/CameraInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraInfo-response>)))
  "Returns md5sum for a message object of type '<CameraInfo-response>"
  "1e0f6cc9c6dfad0bee4253b15158a925")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraInfo-response)))
  "Returns md5sum for a message object of type 'CameraInfo-response"
  "1e0f6cc9c6dfad0bee4253b15158a925")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraInfo-response>)))
  "Returns full string definition for message of type '<CameraInfo-response>"
  (cl:format cl:nil "int64 width~%int64 height~%float32 hfov~%float32 vfov~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraInfo-response)))
  "Returns full string definition for message of type 'CameraInfo-response"
  (cl:format cl:nil "int64 width~%int64 height~%float32 hfov~%float32 vfov~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraInfo-response>))
  (cl:+ 0
     8
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraInfo-response
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':hfov (hfov msg))
    (cl:cons ':vfov (vfov msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CameraInfo)))
  'CameraInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CameraInfo)))
  'CameraInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraInfo)))
  "Returns string type for a service object of type '<CameraInfo>"
  "gesture_pkg/CameraInfo")