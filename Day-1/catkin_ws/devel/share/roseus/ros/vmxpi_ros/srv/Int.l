;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::Int)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'Int (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::INT")
  (make-package "VMXPI_ROS::INT"))
(unless (find-package "VMXPI_ROS::INTREQUEST")
  (make-package "VMXPI_ROS::INTREQUEST"))
(unless (find-package "VMXPI_ROS::INTRESPONSE")
  (make-package "VMXPI_ROS::INTRESPONSE"))

(in-package "ROS")





(defclass vmxpi_ros::IntRequest
  :super ros::object
  :slots (_data ))

(defmethod vmxpi_ros::IntRequest
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _data
       (write-long _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass vmxpi_ros::IntResponse
  :super ros::object
  :slots ())

(defmethod vmxpi_ros::IntResponse
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

(defclass vmxpi_ros::Int
  :super ros::object
  :slots ())

(setf (get vmxpi_ros::Int :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get vmxpi_ros::Int :datatype-) "vmxpi_ros/Int")
(setf (get vmxpi_ros::Int :request) vmxpi_ros::IntRequest)
(setf (get vmxpi_ros::Int :response) vmxpi_ros::IntResponse)

(defmethod vmxpi_ros::IntRequest
  (:response () (instance vmxpi_ros::IntResponse :init)))

(setf (get vmxpi_ros::IntRequest :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get vmxpi_ros::IntRequest :datatype-) "vmxpi_ros/IntRequest")
(setf (get vmxpi_ros::IntRequest :definition-)
      "int32 data
---
")

(setf (get vmxpi_ros::IntResponse :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get vmxpi_ros::IntResponse :datatype-) "vmxpi_ros/IntResponse")
(setf (get vmxpi_ros::IntResponse :definition-)
      "int32 data
---
")



(provide :vmxpi_ros/Int "da5909fbe378aeaf85e547e830cc1bb7")


