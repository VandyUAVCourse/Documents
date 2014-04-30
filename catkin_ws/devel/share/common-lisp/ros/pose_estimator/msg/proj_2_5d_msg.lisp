; Auto-generated. Do not edit!


(cl:in-package pose_estimator-msg)


;//! \htmlinclude proj_2_5d_msg.msg.html

(cl:defclass <proj_2_5d_msg> (roslisp-msg-protocol:ros-message)
  ((refPoints
    :reader refPoints
    :initarg :refPoints
    :type (cl:vector pose_estimator-msg:point2d_t)
   :initform (cl:make-array 0 :element-type 'pose_estimator-msg:point2d_t :initial-element (cl:make-instance 'pose_estimator-msg:point2d_t)))
   (targetPoints
    :reader targetPoints
    :initarg :targetPoints
    :type (cl:vector pose_estimator-msg:point2d_t)
   :initform (cl:make-array 0 :element-type 'pose_estimator-msg:point2d_t :initial-element (cl:make-instance 'pose_estimator-msg:point2d_t))))
)

(cl:defclass proj_2_5d_msg (<proj_2_5d_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <proj_2_5d_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'proj_2_5d_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pose_estimator-msg:<proj_2_5d_msg> is deprecated: use pose_estimator-msg:proj_2_5d_msg instead.")))

(cl:ensure-generic-function 'refPoints-val :lambda-list '(m))
(cl:defmethod refPoints-val ((m <proj_2_5d_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:refPoints-val is deprecated.  Use pose_estimator-msg:refPoints instead.")
  (refPoints m))

(cl:ensure-generic-function 'targetPoints-val :lambda-list '(m))
(cl:defmethod targetPoints-val ((m <proj_2_5d_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:targetPoints-val is deprecated.  Use pose_estimator-msg:targetPoints instead.")
  (targetPoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <proj_2_5d_msg>) ostream)
  "Serializes a message object of type '<proj_2_5d_msg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'refPoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'refPoints))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'targetPoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'targetPoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <proj_2_5d_msg>) istream)
  "Deserializes a message object of type '<proj_2_5d_msg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'refPoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'refPoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pose_estimator-msg:point2d_t))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'targetPoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'targetPoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pose_estimator-msg:point2d_t))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<proj_2_5d_msg>)))
  "Returns string type for a message object of type '<proj_2_5d_msg>"
  "pose_estimator/proj_2_5d_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'proj_2_5d_msg)))
  "Returns string type for a message object of type 'proj_2_5d_msg"
  "pose_estimator/proj_2_5d_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<proj_2_5d_msg>)))
  "Returns md5sum for a message object of type '<proj_2_5d_msg>"
  "79c33411ebcb4d92753eedd76f986950")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'proj_2_5d_msg)))
  "Returns md5sum for a message object of type 'proj_2_5d_msg"
  "79c33411ebcb4d92753eedd76f986950")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<proj_2_5d_msg>)))
  "Returns full string definition for message of type '<proj_2_5d_msg>"
  (cl:format cl:nil "# Reference points~%point2d_t[] refPoints~%~%# Target points~%point2d_t[] targetPoints~%~%================================================================================~%MSG: pose_estimator/point2d_t~%float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'proj_2_5d_msg)))
  "Returns full string definition for message of type 'proj_2_5d_msg"
  (cl:format cl:nil "# Reference points~%point2d_t[] refPoints~%~%# Target points~%point2d_t[] targetPoints~%~%================================================================================~%MSG: pose_estimator/point2d_t~%float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <proj_2_5d_msg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'refPoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'targetPoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <proj_2_5d_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'proj_2_5d_msg
    (cl:cons ':refPoints (refPoints msg))
    (cl:cons ':targetPoints (targetPoints msg))
))
