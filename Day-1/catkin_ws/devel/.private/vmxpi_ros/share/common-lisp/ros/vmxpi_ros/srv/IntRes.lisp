; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-srv)


;//! \htmlinclude IntRes-request.msg.html

(cl:defclass <IntRes-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass IntRes-request (<IntRes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntRes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntRes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<IntRes-request> is deprecated: use vmxpi_ros-srv:IntRes-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntRes-request>) ostream)
  "Serializes a message object of type '<IntRes-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntRes-request>) istream)
  "Deserializes a message object of type '<IntRes-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntRes-request>)))
  "Returns string type for a service object of type '<IntRes-request>"
  "vmxpi_ros/IntResRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntRes-request)))
  "Returns string type for a service object of type 'IntRes-request"
  "vmxpi_ros/IntResRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntRes-request>)))
  "Returns md5sum for a message object of type '<IntRes-request>"
  "8524586e34fbd7cb1c08c5f5f1ca0e57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntRes-request)))
  "Returns md5sum for a message object of type 'IntRes-request"
  "8524586e34fbd7cb1c08c5f5f1ca0e57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntRes-request>)))
  "Returns full string definition for message of type '<IntRes-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntRes-request)))
  "Returns full string definition for message of type 'IntRes-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntRes-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntRes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IntRes-request
))
;//! \htmlinclude IntRes-response.msg.html

(cl:defclass <IntRes-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass IntRes-response (<IntRes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntRes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntRes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<IntRes-response> is deprecated: use vmxpi_ros-srv:IntRes-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <IntRes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-srv:data-val is deprecated.  Use vmxpi_ros-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntRes-response>) ostream)
  "Serializes a message object of type '<IntRes-response>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntRes-response>) istream)
  "Deserializes a message object of type '<IntRes-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntRes-response>)))
  "Returns string type for a service object of type '<IntRes-response>"
  "vmxpi_ros/IntResResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntRes-response)))
  "Returns string type for a service object of type 'IntRes-response"
  "vmxpi_ros/IntResResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntRes-response>)))
  "Returns md5sum for a message object of type '<IntRes-response>"
  "8524586e34fbd7cb1c08c5f5f1ca0e57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntRes-response)))
  "Returns md5sum for a message object of type 'IntRes-response"
  "8524586e34fbd7cb1c08c5f5f1ca0e57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntRes-response>)))
  "Returns full string definition for message of type '<IntRes-response>"
  (cl:format cl:nil "int16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntRes-response)))
  "Returns full string definition for message of type 'IntRes-response"
  (cl:format cl:nil "int16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntRes-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntRes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IntRes-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IntRes)))
  'IntRes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IntRes)))
  'IntRes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntRes)))
  "Returns string type for a service object of type '<IntRes>"
  "vmxpi_ros/IntRes")