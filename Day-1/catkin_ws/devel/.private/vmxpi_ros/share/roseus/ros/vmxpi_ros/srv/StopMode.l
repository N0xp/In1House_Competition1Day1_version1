;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::StopMode)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'StopMode (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::STOPMODE")
  (make-package "VMXPI_ROS::STOPMODE"))
(unless (find-package "VMXPI_ROS::STOPMODEREQUEST")
  (make-package "VMXPI_ROS::STOPMODEREQUEST"))
(unless (find-package "VMXPI_ROS::STOPMODERESPONSE")
  (make-package "VMXPI_ROS::STOPMODERESPONSE"))

(in-package "ROS")





(defclass vmxpi_ros::StopModeRequest
  :super ros::object
  :slots (_motor _mode ))

(defmethod vmxpi_ros::StopModeRequest
  (:init
   (&key
    ((:motor __motor) 0)
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _motor (round __motor))
   (setq _mode (round __mode))
   self)
  (:motor
   (&optional __motor)
   (if __motor (setq _motor __motor)) _motor)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int8 _motor
    1
    ;; int8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _motor
       (write-byte _motor s)
     ;; int8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _motor
     (setq _motor (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _motor 127) (setq _motor (- _motor 256)))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;;
   self)
  )

(defclass vmxpi_ros::StopModeResponse
  :super ros::object
  :slots ())

(defmethod vmxpi_ros::StopModeResponse
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

(defclass vmxpi_ros::StopMode
  :super ros::object
  :slots ())

(setf (get vmxpi_ros::StopMode :md5sum-) "f4be1905b0e869ab5f98286edb652845")
(setf (get vmxpi_ros::StopMode :datatype-) "vmxpi_ros/StopMode")
(setf (get vmxpi_ros::StopMode :request) vmxpi_ros::StopModeRequest)
(setf (get vmxpi_ros::StopMode :response) vmxpi_ros::StopModeResponse)

(defmethod vmxpi_ros::StopModeRequest
  (:response () (instance vmxpi_ros::StopModeResponse :init)))

(setf (get vmxpi_ros::StopModeRequest :md5sum-) "f4be1905b0e869ab5f98286edb652845")
(setf (get vmxpi_ros::StopModeRequest :datatype-) "vmxpi_ros/StopModeRequest")
(setf (get vmxpi_ros::StopModeRequest :definition-)
      "int8 motor
int8 mode
---
")

(setf (get vmxpi_ros::StopModeResponse :md5sum-) "f4be1905b0e869ab5f98286edb652845")
(setf (get vmxpi_ros::StopModeResponse :datatype-) "vmxpi_ros/StopModeResponse")
(setf (get vmxpi_ros::StopModeResponse :definition-)
      "int8 motor
int8 mode
---
")



(provide :vmxpi_ros/StopMode "f4be1905b0e869ab5f98286edb652845")


