; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-msg)


;//! \htmlinclude UniqueID.msg.html

(cl:defclass <UniqueID> (roslisp-msg-protocol:ros-message)
  ((word1
    :reader word1
    :initarg :word1
    :type cl:string
    :initform "")
   (word2
    :reader word2
    :initarg :word2
    :type cl:string
    :initform "")
   (word3
    :reader word3
    :initarg :word3
    :type cl:string
    :initform ""))
)

(cl:defclass UniqueID (<UniqueID>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UniqueID>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UniqueID)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-msg:<UniqueID> is deprecated: use vmxpi_ros-msg:UniqueID instead.")))

(cl:ensure-generic-function 'word1-val :lambda-list '(m))
(cl:defmethod word1-val ((m <UniqueID>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:word1-val is deprecated.  Use vmxpi_ros-msg:word1 instead.")
  (word1 m))

(cl:ensure-generic-function 'word2-val :lambda-list '(m))
(cl:defmethod word2-val ((m <UniqueID>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:word2-val is deprecated.  Use vmxpi_ros-msg:word2 instead.")
  (word2 m))

(cl:ensure-generic-function 'word3-val :lambda-list '(m))
(cl:defmethod word3-val ((m <UniqueID>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:word3-val is deprecated.  Use vmxpi_ros-msg:word3 instead.")
  (word3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UniqueID>) ostream)
  "Serializes a message object of type '<UniqueID>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'word1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'word1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'word2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'word2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'word3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'word3))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UniqueID>) istream)
  "Deserializes a message object of type '<UniqueID>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'word1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'word1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'word2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'word2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'word3) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'word3) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UniqueID>)))
  "Returns string type for a message object of type '<UniqueID>"
  "vmxpi_ros/UniqueID")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UniqueID)))
  "Returns string type for a message object of type 'UniqueID"
  "vmxpi_ros/UniqueID")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UniqueID>)))
  "Returns md5sum for a message object of type '<UniqueID>"
  "d3fe5a40e07321a3fd3e3e35d9b0c93b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UniqueID)))
  "Returns md5sum for a message object of type 'UniqueID"
  "d3fe5a40e07321a3fd3e3e35d9b0c93b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UniqueID>)))
  "Returns full string definition for message of type '<UniqueID>"
  (cl:format cl:nil "string word1~%string word2~%string word3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UniqueID)))
  "Returns full string definition for message of type 'UniqueID"
  (cl:format cl:nil "string word1~%string word2~%string word3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UniqueID>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'word1))
     4 (cl:length (cl:slot-value msg 'word2))
     4 (cl:length (cl:slot-value msg 'word3))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UniqueID>))
  "Converts a ROS message object to a list"
  (cl:list 'UniqueID
    (cl:cons ':word1 (word1 msg))
    (cl:cons ':word2 (word2 msg))
    (cl:cons ':word3 (word3 msg))
))
