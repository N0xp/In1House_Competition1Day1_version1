; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-srv)


;//! \htmlinclude MotorSpeed-request.msg.html

(cl:defclass <MotorSpeed-request> (roslisp-msg-protocol:ros-message)
  ((motor
    :reader motor
    :initarg :motor
    :type cl:integer
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotorSpeed-request (<MotorSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<MotorSpeed-request> is deprecated: use vmxpi_ros-srv:MotorSpeed-request instead.")))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <MotorSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-srv:motor-val is deprecated.  Use vmxpi_ros-srv:motor instead.")
  (motor m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <MotorSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-srv:speed-val is deprecated.  Use vmxpi_ros-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorSpeed-request>) ostream)
  "Serializes a message object of type '<MotorSpeed-request>"
  (cl:let* ((signed (cl:slot-value msg 'motor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorSpeed-request>) istream)
  "Deserializes a message object of type '<MotorSpeed-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorSpeed-request>)))
  "Returns string type for a service object of type '<MotorSpeed-request>"
  "vmxpi_ros/MotorSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorSpeed-request)))
  "Returns string type for a service object of type 'MotorSpeed-request"
  "vmxpi_ros/MotorSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorSpeed-request>)))
  "Returns md5sum for a message object of type '<MotorSpeed-request>"
  "c74fe58695fd9150f0fc1ab8000015b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorSpeed-request)))
  "Returns md5sum for a message object of type 'MotorSpeed-request"
  "c74fe58695fd9150f0fc1ab8000015b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorSpeed-request>)))
  "Returns full string definition for message of type '<MotorSpeed-request>"
  (cl:format cl:nil "int32 motor~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorSpeed-request)))
  "Returns full string definition for message of type 'MotorSpeed-request"
  (cl:format cl:nil "int32 motor~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorSpeed-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorSpeed-request
    (cl:cons ':motor (motor msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude MotorSpeed-response.msg.html

(cl:defclass <MotorSpeed-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MotorSpeed-response (<MotorSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<MotorSpeed-response> is deprecated: use vmxpi_ros-srv:MotorSpeed-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorSpeed-response>) ostream)
  "Serializes a message object of type '<MotorSpeed-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorSpeed-response>) istream)
  "Deserializes a message object of type '<MotorSpeed-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorSpeed-response>)))
  "Returns string type for a service object of type '<MotorSpeed-response>"
  "vmxpi_ros/MotorSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorSpeed-response)))
  "Returns string type for a service object of type 'MotorSpeed-response"
  "vmxpi_ros/MotorSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorSpeed-response>)))
  "Returns md5sum for a message object of type '<MotorSpeed-response>"
  "c74fe58695fd9150f0fc1ab8000015b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorSpeed-response)))
  "Returns md5sum for a message object of type 'MotorSpeed-response"
  "c74fe58695fd9150f0fc1ab8000015b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorSpeed-response>)))
  "Returns full string definition for message of type '<MotorSpeed-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorSpeed-response)))
  "Returns full string definition for message of type 'MotorSpeed-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorSpeed-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorSpeed-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MotorSpeed)))
  'MotorSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MotorSpeed)))
  'MotorSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorSpeed)))
  "Returns string type for a service object of type '<MotorSpeed>"
  "vmxpi_ros/MotorSpeed")