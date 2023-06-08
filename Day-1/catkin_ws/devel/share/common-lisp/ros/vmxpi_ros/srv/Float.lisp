; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-srv)


;//! \htmlinclude Float-request.msg.html

(cl:defclass <Float-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass Float-request (<Float-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Float-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Float-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<Float-request> is deprecated: use vmxpi_ros-srv:Float-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Float-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-srv:data-val is deprecated.  Use vmxpi_ros-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Float-request>) ostream)
  "Serializes a message object of type '<Float-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Float-request>) istream)
  "Deserializes a message object of type '<Float-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Float-request>)))
  "Returns string type for a service object of type '<Float-request>"
  "vmxpi_ros/FloatRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Float-request)))
  "Returns string type for a service object of type 'Float-request"
  "vmxpi_ros/FloatRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Float-request>)))
  "Returns md5sum for a message object of type '<Float-request>"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Float-request)))
  "Returns md5sum for a message object of type 'Float-request"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Float-request>)))
  "Returns full string definition for message of type '<Float-request>"
  (cl:format cl:nil "float32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Float-request)))
  "Returns full string definition for message of type 'Float-request"
  (cl:format cl:nil "float32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Float-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Float-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Float-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude Float-response.msg.html

(cl:defclass <Float-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Float-response (<Float-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Float-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Float-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<Float-response> is deprecated: use vmxpi_ros-srv:Float-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Float-response>) ostream)
  "Serializes a message object of type '<Float-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Float-response>) istream)
  "Deserializes a message object of type '<Float-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Float-response>)))
  "Returns string type for a service object of type '<Float-response>"
  "vmxpi_ros/FloatResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Float-response)))
  "Returns string type for a service object of type 'Float-response"
  "vmxpi_ros/FloatResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Float-response>)))
  "Returns md5sum for a message object of type '<Float-response>"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Float-response)))
  "Returns md5sum for a message object of type 'Float-response"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Float-response>)))
  "Returns full string definition for message of type '<Float-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Float-response)))
  "Returns full string definition for message of type 'Float-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Float-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Float-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Float-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Float)))
  'Float-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Float)))
  'Float-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Float)))
  "Returns string type for a service object of type '<Float>"
  "vmxpi_ros/Float")