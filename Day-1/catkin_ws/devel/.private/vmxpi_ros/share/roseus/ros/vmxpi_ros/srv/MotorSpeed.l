;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::MotorSpeed)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'MotorSpeed (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::MOTORSPEED")
  (make-package "VMXPI_ROS::MOTORSPEED"))
(unless (find-package "VMXPI_ROS::MOTORSPEEDREQUEST")
  (make-package "VMXPI_ROS::MOTORSPEEDREQUEST"))
(unless (find-package "VMXPI_ROS::MOTORSPEEDRESPONSE")
  (make-package "VMXPI_ROS::MOTORSPEEDRESPONSE"))

(in-package "ROS")





(defclass vmxpi_ros::MotorSpeedRequest
  :super ros::object
  :slots (_motor _speed ))

(defmethod vmxpi_ros::MotorSpeedRequest
  (:init
   (&key
    ((:motor __motor) 0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _motor (round __motor))
   (setq _speed (float __speed))
   self)
  (:motor
   (&optional __motor)
   (if __motor (setq _motor __motor)) _motor)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int32 _motor
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _motor
       (write-long _motor s)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _motor
     (setq _motor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass vmxpi_ros::MotorSpeedResponse
  :super ros::object
  :slots ())

(defmethod vmxpi_ros::MotorSpeedResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass vmxpi_ros::MotorSpeed
  :super ros::object
  :slots ())

(setf (get vmxpi_ros::MotorSpeed :md5sum-) "c74fe58695fd9150f0fc1ab8000015b6")
(setf (get vmxpi_ros::MotorSpeed :datatype-) "vmxpi_ros/MotorSpeed")
(setf (get vmxpi_ros::MotorSpeed :request) vmxpi_ros::MotorSpeedRequest)
(setf (get vmxpi_ros::MotorSpeed :response) vmxpi_ros::MotorSpeedResponse)

(defmethod vmxpi_ros::MotorSpeedRequest
  (:response () (instance vmxpi_ros::MotorSpeedResponse :init)))

(setf (get vmxpi_ros::MotorSpeedRequest :md5sum-) "c74fe58695fd9150f0fc1ab8000015b6")
(setf (get vmxpi_ros::MotorSpeedRequest :datatype-) "vmxpi_ros/MotorSpeedRequest")
(setf (get vmxpi_ros::MotorSpeedRequest :definition-)
      "int32 motor
float32 speed
---
")

(setf (get vmxpi_ros::MotorSpeedResponse :md5sum-) "c74fe58695fd9150f0fc1ab8000015b6")
(setf (get vmxpi_ros::MotorSpeedResponse :datatype-) "vmxpi_ros/MotorSpeedResponse")
(setf (get vmxpi_ros::MotorSpeedResponse :definition-)
      "int32 motor
float32 speed
---
")



(provide :vmxpi_ros/MotorSpeed "c74fe58695fd9150f0fc1ab8000015b6")


