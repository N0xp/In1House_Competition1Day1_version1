; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-srv)


;//! \htmlinclude Int-request.msg.html

(cl:defclass <Int-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass Int-request (<Int-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Int-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Int-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<Int-request> is deprecated: use vmxpi_ros-srv:Int-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Int-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-srv:data-val is deprecated.  Use vmxpi_ros-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Int-request>) ostream)
  "Serializes a message object of type '<Int-request>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Int-request>) istream)
  "Deserializes a message object of type '<Int-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Int-request>)))
  "Returns string type for a service object of type '<Int-request>"
  "vmxpi_ros/IntRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Int-request)))
  "Returns string type for a service object of type 'Int-request"
  "vmxpi_ros/IntRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Int-request>)))
  "Returns md5sum for a message object of type '<Int-request>"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Int-request)))
  "Returns md5sum for a message object of type 'Int-request"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Int-request>)))
  "Returns full string definition for message of type '<Int-request>"
  (cl:format cl:nil "int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Int-request)))
  "Returns full string definition for message of type 'Int-request"
  (cl:format cl:nil "int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Int-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Int-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Int-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude Int-response.msg.html

(cl:defclass <Int-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Int-response (<Int-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Int-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Int-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<Int-response> is deprecated: use vmxpi_ros-srv:Int-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Int-response>) ostream)
  "Serializes a message object of type '<Int-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Int-response>) istream)
  "Deserializes a message object of type '<Int-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Int-response>)))
  "Returns string type for a service object of type '<Int-response>"
  "vmxpi_ros/IntResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Int-response)))
  "Returns string type for a service object of type 'Int-response"
  "vmxpi_ros/IntResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Int-response>)))
  "Returns md5sum for a message object of type '<Int-response>"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Int-response)))
  "Returns md5sum for a message object of type 'Int-response"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Int-response>)))
  "Returns full string definition for message of type '<Int-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Int-response)))
  "Returns full string definition for message of type 'Int-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Int-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Int-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Int-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Int)))
  'Int-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Int)))
  'Int-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Int)))
  "Returns string type for a service object of type '<Int>"
  "vmxpi_ros/Int")