; Auto-generated. Do not edit!


(cl:in-package navmap-msg)


;//! \htmlinclude raw_imu.msg.html

(cl:defclass <raw_imu> (roslisp-msg-protocol:ros-message)
  ((xacc
    :reader xacc
    :initarg :xacc
    :type cl:fixnum
    :initform 0)
   (yacc
    :reader yacc
    :initarg :yacc
    :type cl:fixnum
    :initform 0)
   (zacc
    :reader zacc
    :initarg :zacc
    :type cl:fixnum
    :initform 0)
   (xgyro
    :reader xgyro
    :initarg :xgyro
    :type cl:fixnum
    :initform 0)
   (ygyro
    :reader ygyro
    :initarg :ygyro
    :type cl:fixnum
    :initform 0)
   (zgyro
    :reader zgyro
    :initarg :zgyro
    :type cl:fixnum
    :initform 0))
)

(cl:defclass raw_imu (<raw_imu>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <raw_imu>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'raw_imu)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navmap-msg:<raw_imu> is deprecated: use navmap-msg:raw_imu instead.")))

(cl:ensure-generic-function 'xacc-val :lambda-list '(m))
(cl:defmethod xacc-val ((m <raw_imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navmap-msg:xacc-val is deprecated.  Use navmap-msg:xacc instead.")
  (xacc m))

(cl:ensure-generic-function 'yacc-val :lambda-list '(m))
(cl:defmethod yacc-val ((m <raw_imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navmap-msg:yacc-val is deprecated.  Use navmap-msg:yacc instead.")
  (yacc m))

(cl:ensure-generic-function 'zacc-val :lambda-list '(m))
(cl:defmethod zacc-val ((m <raw_imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navmap-msg:zacc-val is deprecated.  Use navmap-msg:zacc instead.")
  (zacc m))

(cl:ensure-generic-function 'xgyro-val :lambda-list '(m))
(cl:defmethod xgyro-val ((m <raw_imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navmap-msg:xgyro-val is deprecated.  Use navmap-msg:xgyro instead.")
  (xgyro m))

(cl:ensure-generic-function 'ygyro-val :lambda-list '(m))
(cl:defmethod ygyro-val ((m <raw_imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navmap-msg:ygyro-val is deprecated.  Use navmap-msg:ygyro instead.")
  (ygyro m))

(cl:ensure-generic-function 'zgyro-val :lambda-list '(m))
(cl:defmethod zgyro-val ((m <raw_imu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navmap-msg:zgyro-val is deprecated.  Use navmap-msg:zgyro instead.")
  (zgyro m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <raw_imu>) ostream)
  "Serializes a message object of type '<raw_imu>"
  (cl:let* ((signed (cl:slot-value msg 'xacc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'yacc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'zacc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'xgyro)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ygyro)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'zgyro)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <raw_imu>) istream)
  "Deserializes a message object of type '<raw_imu>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'xacc) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'yacc) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'zacc) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'xgyro) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ygyro) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'zgyro) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<raw_imu>)))
  "Returns string type for a message object of type '<raw_imu>"
  "navmap/raw_imu")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'raw_imu)))
  "Returns string type for a message object of type 'raw_imu"
  "navmap/raw_imu")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<raw_imu>)))
  "Returns md5sum for a message object of type '<raw_imu>"
  "3507f0b63f5790cbc052576b2a56f94a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'raw_imu)))
  "Returns md5sum for a message object of type 'raw_imu"
  "3507f0b63f5790cbc052576b2a56f94a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<raw_imu>)))
  "Returns full string definition for message of type '<raw_imu>"
  (cl:format cl:nil "int16 xacc~%int16 yacc~%int16 zacc~%int16 xgyro~%int16 ygyro~%int16 zgyro~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'raw_imu)))
  "Returns full string definition for message of type 'raw_imu"
  (cl:format cl:nil "int16 xacc~%int16 yacc~%int16 zacc~%int16 xgyro~%int16 ygyro~%int16 zgyro~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <raw_imu>))
  (cl:+ 0
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <raw_imu>))
  "Converts a ROS message object to a list"
  (cl:list 'raw_imu
    (cl:cons ':xacc (xacc msg))
    (cl:cons ':yacc (yacc msg))
    (cl:cons ':zacc (zacc msg))
    (cl:cons ':xgyro (xgyro msg))
    (cl:cons ':ygyro (ygyro msg))
    (cl:cons ':zgyro (zgyro msg))
))
