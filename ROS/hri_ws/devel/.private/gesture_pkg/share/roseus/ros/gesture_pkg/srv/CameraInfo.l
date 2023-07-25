;; Auto-generated. Do not edit!


(when (boundp 'gesture_pkg::CameraInfo)
  (if (not (find-package "GESTURE_PKG"))
    (make-package "GESTURE_PKG"))
  (shadow 'CameraInfo (find-package "GESTURE_PKG")))
(unless (find-package "GESTURE_PKG::CAMERAINFO")
  (make-package "GESTURE_PKG::CAMERAINFO"))
(unless (find-package "GESTURE_PKG::CAMERAINFOREQUEST")
  (make-package "GESTURE_PKG::CAMERAINFOREQUEST"))
(unless (find-package "GESTURE_PKG::CAMERAINFORESPONSE")
  (make-package "GESTURE_PKG::CAMERAINFORESPONSE"))

(in-package "ROS")





(defclass gesture_pkg::CameraInfoRequest
  :super ros::object
  :slots ())

(defmethod gesture_pkg::CameraInfoRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass gesture_pkg::CameraInfoResponse
  :super ros::object
  :slots (_width _height _hfov _vfov ))

(defmethod gesture_pkg::CameraInfoResponse
  (:init
   (&key
    ((:width __width) 0)
    ((:height __height) 0)
    ((:hfov __hfov) 0.0)
    ((:vfov __vfov) 0.0)
    )
   (send-super :init)
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _hfov (float __hfov))
   (setq _vfov (float __vfov))
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:hfov
   (&optional __hfov)
   (if __hfov (setq _hfov __hfov)) _hfov)
  (:vfov
   (&optional __vfov)
   (if __vfov (setq _vfov __vfov)) _vfov)
  (:serialization-length
   ()
   (+
    ;; int64 _width
    8
    ;; int64 _height
    8
    ;; float32 _hfov
    4
    ;; float32 _vfov
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _width
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _width (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _width) (= (length (_width . bv)) 2)) ;; bignum
              (write-long (ash (elt (_width . bv) 0) 0) s)
              (write-long (ash (elt (_width . bv) 1) -1) s))
             ((and (class _width) (= (length (_width . bv)) 1)) ;; big1
              (write-long (elt (_width . bv) 0) s)
              (write-long (if (>= _width 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _width s)(write-long (if (>= _width 0) 0 #xffffffff) s)))
     ;; int64 _height
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _height (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _height) (= (length (_height . bv)) 2)) ;; bignum
              (write-long (ash (elt (_height . bv) 0) 0) s)
              (write-long (ash (elt (_height . bv) 1) -1) s))
             ((and (class _height) (= (length (_height . bv)) 1)) ;; big1
              (write-long (elt (_height . bv) 0) s)
              (write-long (if (>= _height 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _height s)(write-long (if (>= _height 0) 0 #xffffffff) s)))
     ;; float32 _hfov
       (sys::poke _hfov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vfov
       (sys::poke _vfov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _width
#+(or :alpha :irix6 :x86_64)
      (setf _width (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _width (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _height
#+(or :alpha :irix6 :x86_64)
      (setf _height (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _height (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float32 _hfov
     (setq _hfov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vfov
     (setq _vfov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass gesture_pkg::CameraInfo
  :super ros::object
  :slots ())

(setf (get gesture_pkg::CameraInfo :md5sum-) "1e0f6cc9c6dfad0bee4253b15158a925")
(setf (get gesture_pkg::CameraInfo :datatype-) "gesture_pkg/CameraInfo")
(setf (get gesture_pkg::CameraInfo :request) gesture_pkg::CameraInfoRequest)
(setf (get gesture_pkg::CameraInfo :response) gesture_pkg::CameraInfoResponse)

(defmethod gesture_pkg::CameraInfoRequest
  (:response () (instance gesture_pkg::CameraInfoResponse :init)))

(setf (get gesture_pkg::CameraInfoRequest :md5sum-) "1e0f6cc9c6dfad0bee4253b15158a925")
(setf (get gesture_pkg::CameraInfoRequest :datatype-) "gesture_pkg/CameraInfoRequest")
(setf (get gesture_pkg::CameraInfoRequest :definition-)
      "---
int64 width
int64 height
float32 hfov
float32 vfov
")

(setf (get gesture_pkg::CameraInfoResponse :md5sum-) "1e0f6cc9c6dfad0bee4253b15158a925")
(setf (get gesture_pkg::CameraInfoResponse :datatype-) "gesture_pkg/CameraInfoResponse")
(setf (get gesture_pkg::CameraInfoResponse :definition-)
      "---
int64 width
int64 height
float32 hfov
float32 vfov
")



(provide :gesture_pkg/CameraInfo "1e0f6cc9c6dfad0bee4253b15158a925")


