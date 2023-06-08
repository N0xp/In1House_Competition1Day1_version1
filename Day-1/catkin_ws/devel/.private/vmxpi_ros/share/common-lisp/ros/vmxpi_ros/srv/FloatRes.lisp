; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-srv)


;//! \htmlinclude FloatRes-request.msg.html

(cl:defclass <FloatRes-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FloatRes-request (<FloatRes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FloatRes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FloatRes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<FloatRes-request> is deprecated: use vmxpi_ros-srv:FloatRes-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FloatRes-request>) ostream)
  "Serializes a message object of type '<FloatRes-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FloatRes-request>) istream)
  "Deserializes a message object of type '<FloatRes-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FloatRes-request>)))
  "Returns string type for a service object of type '<FloatRes-request>"
  "vmxpi_ros/FloatResRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FloatRes-request)))
  "Returns string type for a service object of type 'FloatRes-request"
  "vmxpi_ros/FloatResRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FloatRes-request>)))
  "Returns md5sum for a message object of type '<FloatRes-request>"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FloatRes-request)))
  "Returns md5sum for a message object of type 'FloatRes-request"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FloatRes-request>)))
  "Returns full string definition for message of type '<FloatRes-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FloatRes-request)))
  "Returns full string definition for message of type 'FloatRes-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FloatRes-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FloatRes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FloatRes-request
))
;//! \htmlinclude FloatRes-response.msg.html

(cl:defclass <FloatRes-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass FloatRes-response (<FloatRes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FloatRes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FloatRes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<FloatRes-response> is deprecated: use vmxpi_ros-srv:FloatRes-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <FloatRes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-srv:data-val is deprecated.  Use vmxpi_ros-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FloatRes-response>) ostream)
  "Serializes a message object of type '<FloatRes-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FloatRes-response>) istream)
  "Deserializes a message object of type '<FloatRes-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FloatRes-response>)))
  "Returns string type for a service object of type '<FloatRes-response>"
  "vmxpi_ros/FloatResResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FloatRes-response)))
  "Returns string type for a service object of type 'FloatRes-response"
  "vmxpi_ros/FloatResResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FloatRes-response>)))
  "Returns md5sum for a message object of type '<FloatRes-response>"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FloatRes-response)))
  "Returns md5sum for a message object of type 'FloatRes-response"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FloatRes-response>)))
  "Returns full string definition for message of type '<FloatRes-response>"
  (cl:format cl:nil "float32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FloatRes-response)))
  "Returns full string definition for message of type 'FloatRes-response"
  (cl:format cl:nil "float32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FloatRes-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FloatRes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FloatRes-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FloatRes)))
  'FloatRes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FloatRes)))
  'FloatRes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FloatRes)))
  "Returns string type for a service object of type '<FloatRes>"
  "vmxpi_ros/FloatRes")