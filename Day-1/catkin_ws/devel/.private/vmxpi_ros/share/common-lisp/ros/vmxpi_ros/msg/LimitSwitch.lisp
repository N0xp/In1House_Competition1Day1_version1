; Auto-generated. Do not edit!


(cl:in-package vmxpi_ros-msg)


;//! \htmlinclude LimitSwitch.msg.html

(cl:defclass <LimitSwitch> (roslisp-msg-protocol:ros-message)
  ((limitswitch
    :reader limitswitch
    :initarg :limitswitch
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass LimitSwitch (<LimitSwitch>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LimitSwitch>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LimitSwitch)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vmxpi_ros-msg:<LimitSwitch> is deprecated: use vmxpi_ros-msg:LimitSwitch instead.")))

(cl:ensure-generic-function 'limitswitch-val :lambda-list '(m))
(cl:defmethod limitswitch-val ((m <LimitSwitch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vmxpi_ros-msg:limitswitch-val is deprecated.  Use vmxpi_ros-msg:limitswitch instead.")
  (limitswitch m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LimitSwitch>) ostream)
  "Serializes a message object of type '<LimitSwitch>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'limitswitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'limitswitch))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LimitSwitch>) istream)
  "Deserializes a message object of type '<LimitSwitch>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'limitswitch) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'limitswitch)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LimitSwitch>)))
  "Returns string type for a message object of type '<LimitSwitch>"
  "vmxpi_ros/LimitSwitch")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LimitSwitch)))
  "Returns string type for a message object of type 'LimitSwitch"
  "vmxpi_ros/LimitSwitch")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LimitSwitch>)))
  "Returns md5sum for a message object of type '<LimitSwitch>"
  "cabe49bdbfdd4a3cf170660bf17038a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LimitSwitch)))
  "Returns md5sum for a message object of type 'LimitSwitch"
  "cabe49bdbfdd4a3cf170660bf17038a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LimitSwitch>)))
  "Returns full string definition for message of type '<LimitSwitch>"
  (cl:format cl:nil "bool[] limitswitch~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LimitSwitch)))
  "Returns full string definition for message of type 'LimitSwitch"
  (cl:format cl:nil "bool[] limitswitch~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LimitSwitch>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'limitswitch) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LimitSwitch>))
  "Converts a ROS message object to a list"
  (cl:list 'LimitSwitch
    (cl:cons ':limitswitch (limitswitch msg))
))
