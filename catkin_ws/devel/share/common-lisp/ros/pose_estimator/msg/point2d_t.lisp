; Auto-generated. Do not edit!


(cl:in-package pose_estimator-msg)


;//! \htmlinclude point2d_t.msg.html

(cl:defclass <point2d_t> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass point2d_t (<point2d_t>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <point2d_t>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'point2d_t)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pose_estimator-msg:<point2d_t> is deprecated: use pose_estimator-msg:point2d_t instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <point2d_t>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:x-val is deprecated.  Use pose_estimator-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <point2d_t>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:y-val is deprecated.  Use pose_estimator-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <point2d_t>) ostream)
  "Serializes a message object of type '<point2d_t>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <point2d_t>) istream)
  "Deserializes a message object of type '<point2d_t>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<point2d_t>)))
  "Returns string type for a message object of type '<point2d_t>"
  "pose_estimator/point2d_t")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'point2d_t)))
  "Returns string type for a message object of type 'point2d_t"
  "pose_estimator/point2d_t")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<point2d_t>)))
  "Returns md5sum for a message object of type '<point2d_t>"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'point2d_t)))
  "Returns md5sum for a message object of type 'point2d_t"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<point2d_t>)))
  "Returns full string definition for message of type '<point2d_t>"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'point2d_t)))
  "Returns full string definition for message of type 'point2d_t"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <point2d_t>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <point2d_t>))
  "Converts a ROS message object to a list"
  (cl:list 'point2d_t
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
