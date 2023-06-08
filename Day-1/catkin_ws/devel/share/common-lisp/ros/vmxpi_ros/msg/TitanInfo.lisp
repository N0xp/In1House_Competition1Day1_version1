; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-msg)


;//! \htmlinclude TitanInfo.msg.html

(cl:defclass <TitanInfo> (roslisp-msg-protocol:ros-message)
  ((deviceID
    :reader deviceID
    :initarg :deviceID
    :type cl:integer
    :initform 0)
   (verMaj
    :reader verMaj
    :initarg :verMaj
    :type cl:integer
    :initform 0)
   (verMin
    :reader verMin
    :initarg :verMin
    :type cl:integer
    :initform 0)
   (verBuild
    :reader verBuild
    :initarg :verBuild
    :type cl:integer
    :initform 0)
   (hardware
    :reader hardware
    :initarg :hardware
    :type cl:integer
    :initform 0)
   (hardwareRev
    :reader hardwareRev
    :initarg :hardwareRev
    :type cl:integer
    :initform 0))
)

(cl:defclass TitanInfo (<TitanInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TitanInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TitanInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-msg:<TitanInfo> is deprecated: use vmxpi_ros-msg:TitanInfo instead.")))

(cl:ensure-generic-function 'deviceID-val :lambda-list '(m))
(cl:defmethod deviceID-val ((m <TitanInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:deviceID-val is deprecated.  Use vmxpi_ros-msg:deviceID instead.")
  (deviceID m))

(cl:ensure-generic-function 'verMaj-val :lambda-list '(m))
(cl:defmethod verMaj-val ((m <TitanInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:verMaj-val is deprecated.  Use vmxpi_ros-msg:verMaj instead.")
  (verMaj m))

(cl:ensure-generic-function 'verMin-val :lambda-list '(m))
(cl:defmethod verMin-val ((m <TitanInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:verMin-val is deprecated.  Use vmxpi_ros-msg:verMin instead.")
  (verMin m))

(cl:ensure-generic-function 'verBuild-val :lambda-list '(m))
(cl:defmethod verBuild-val ((m <TitanInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:verBuild-val is deprecated.  Use vmxpi_ros-msg:verBuild instead.")
  (verBuild m))

(cl:ensure-generic-function 'hardware-val :lambda-list '(m))
(cl:defmethod hardware-val ((m <TitanInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:hardware-val is deprecated.  Use vmxpi_ros-msg:hardware instead.")
  (hardware m))

(cl:ensure-generic-function 'hardwareRev-val :lambda-list '(m))
(cl:defmethod hardwareRev-val ((m <TitanInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:hardwareRev-val is deprecated.  Use vmxpi_ros-msg:hardwareRev instead.")
  (hardwareRev m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TitanInfo>) ostream)
  "Serializes a message object of type '<TitanInfo>"
  (cl:let* ((signed (cl:slot-value msg 'deviceID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'verMaj)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'verMin)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'verBuild)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'hardware)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'hardwareRev)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TitanInfo>) istream)
  "Deserializes a message object of type '<TitanInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'deviceID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'verMaj) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'verMin) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'verBuild) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hardware) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hardwareRev) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TitanInfo>)))
  "Returns string type for a message object of type '<TitanInfo>"
  "vmxpi_ros/TitanInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TitanInfo)))
  "Returns string type for a message object of type 'TitanInfo"
  "vmxpi_ros/TitanInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TitanInfo>)))
  "Returns md5sum for a message object of type '<TitanInfo>"
  "5c9aee76b3d5cddac787b5c807fa2b41")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TitanInfo)))
  "Returns md5sum for a message object of type 'TitanInfo"
  "5c9aee76b3d5cddac787b5c807fa2b41")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TitanInfo>)))
  "Returns full string definition for message of type '<TitanInfo>"
  (cl:format cl:nil "int32 deviceID~%int32 verMaj~%int32 verMin~%int32 verBuild~%int32 hardware~%int32 hardwareRev~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TitanInfo)))
  "Returns full string definition for message of type 'TitanInfo"
  (cl:format cl:nil "int32 deviceID~%int32 verMaj~%int32 verMin~%int32 verBuild~%int32 hardware~%int32 hardwareRev~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TitanInfo>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TitanInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'TitanInfo
    (cl:cons ':deviceID (deviceID msg))
    (cl:cons ':verMaj (verMaj msg))
    (cl:cons ':verMin (verMin msg))
    (cl:cons ':verBuild (verBuild msg))
    (cl:cons ':hardware (hardware msg))
    (cl:cons ':hardwareRev (hardwareRev msg))
))
