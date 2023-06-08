; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-srv)


;//! \htmlinclude StringRes-request.msg.html

(cl:defclass <StringRes-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StringRes-request (<StringRes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StringRes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StringRes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<StringRes-request> is deprecated: use vmxpi_ros-srv:StringRes-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StringRes-request>) ostream)
  "Serializes a message object of type '<StringRes-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StringRes-request>) istream)
  "Deserializes a message object of type '<StringRes-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StringRes-request>)))
  "Returns string type for a service object of type '<StringRes-request>"
  "vmxpi_ros/StringResRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StringRes-request)))
  "Returns string type for a service object of type 'StringRes-request"
  "vmxpi_ros/StringResRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StringRes-request>)))
  "Returns md5sum for a message object of type '<StringRes-request>"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StringRes-request)))
  "Returns md5sum for a message object of type 'StringRes-request"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StringRes-request>)))
  "Returns full string definition for message of type '<StringRes-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StringRes-request)))
  "Returns full string definition for message of type 'StringRes-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StringRes-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StringRes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StringRes-request
))
;//! \htmlinclude StringRes-response.msg.html

(cl:defclass <StringRes-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass StringRes-response (<StringRes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StringRes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StringRes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-srv:<StringRes-response> is deprecated: use vmxpi_ros-srv:StringRes-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <StringRes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-srv:data-val is deprecated.  Use vmxpi_ros-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StringRes-response>) ostream)
  "Serializes a message object of type '<StringRes-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StringRes-response>) istream)
  "Deserializes a message object of type '<StringRes-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StringRes-response>)))
  "Returns string type for a service object of type '<StringRes-response>"
  "vmxpi_ros/StringResResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StringRes-response)))
  "Returns string type for a service object of type 'StringRes-response"
  "vmxpi_ros/StringResResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StringRes-response>)))
  "Returns md5sum for a message object of type '<StringRes-response>"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StringRes-response)))
  "Returns md5sum for a message object of type 'StringRes-response"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StringRes-response>)))
  "Returns full string definition for message of type '<StringRes-response>"
  (cl:format cl:nil "string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StringRes-response)))
  "Returns full string definition for message of type 'StringRes-response"
  (cl:format cl:nil "string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StringRes-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StringRes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StringRes-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StringRes)))
  'StringRes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StringRes)))
  'StringRes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StringRes)))
  "Returns string type for a service object of type '<StringRes>"
  "vmxpi_ros/StringRes")