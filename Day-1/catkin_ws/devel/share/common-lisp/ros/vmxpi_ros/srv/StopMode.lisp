; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-srv)


;//! \htmlinclude StopMode-request.msg.html

(cl:defclass <StopMode-request> (roslisp-msg-protocol:ros-message)
  ((motor
    :reader motor
    :initarg :motor
    :type cl:fixnum
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass StopMode-request (<StopMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<StopMode-request> is deprecated: use vmxpi_ros-srv:StopMode-request instead.")))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <StopMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-srv:motor-val is deprecated.  Use vmxpi_ros-srv:motor instead.")
  (motor m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <StopMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-srv:mode-val is deprecated.  Use vmxpi_ros-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopMode-request>) ostream)
  "Serializes a message object of type '<StopMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'motor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopMode-request>) istream)
  "Deserializes a message object of type '<StopMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopMode-request>)))
  "Returns string type for a service object of type '<StopMode-request>"
  "vmxpi_ros/StopModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopMode-request)))
  "Returns string type for a service object of type 'StopMode-request"
  "vmxpi_ros/StopModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopMode-request>)))
  "Returns md5sum for a message object of type '<StopMode-request>"
  "f4be1905b0e869ab5f98286edb652845")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopMode-request)))
  "Returns md5sum for a message object of type 'StopMode-request"
  "f4be1905b0e869ab5f98286edb652845")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopMode-request>)))
  "Returns full string definition for message of type '<StopMode-request>"
  (cl:format cl:nil "int8 motor~%int8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopMode-request)))
  "Returns full string definition for message of type 'StopMode-request"
  (cl:format cl:nil "int8 motor~%int8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopMode-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopMode-request
    (cl:cons ':motor (motor msg))
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude StopMode-response.msg.html

(cl:defclass <StopMode-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopMode-response (<StopMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<StopMode-response> is deprecated: use vmxpi_ros-srv:StopMode-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopMode-response>) ostream)
  "Serializes a message object of type '<StopMode-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopMode-response>) istream)
  "Deserializes a message object of type '<StopMode-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopMode-response>)))
  "Returns string type for a service object of type '<StopMode-response>"
  "vmxpi_ros/StopModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopMode-response)))
  "Returns string type for a service object of type 'StopMode-response"
  "vmxpi_ros/StopModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopMode-response>)))
  "Returns md5sum for a message object of type '<StopMode-response>"
  "f4be1905b0e869ab5f98286edb652845")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopMode-response)))
  "Returns md5sum for a message object of type 'StopMode-response"
  "f4be1905b0e869ab5f98286edb652845")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopMode-response>)))
  "Returns full string definition for message of type '<StopMode-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopMode-response)))
  "Returns full string definition for message of type 'StopMode-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopMode-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopMode-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopMode)))
  'StopMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopMode)))
  'StopMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopMode)))
  "Returns string type for a service object of type '<StopMode>"
  "vmxpi_ros/StopMode")