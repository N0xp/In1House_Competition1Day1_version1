;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::IntRes)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'IntRes (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::INTRES")
  (make-package "VMXPI_ROS::INTRES"))
(unless (find-package "VMXPI_ROS::INTRESREQUEST")
  (make-package "VMXPI_ROS::INTRESREQUEST"))
(unless (find-package "VMXPI_ROS::INTRESRESPONSE")
  (make-package "VMXPI_ROS::INTRESRESPONSE"))

(in-package "ROS")





(defclass vmxpi_ros::IntResRequest
  :super ros::object
  :slots ())

(defmethod vmxpi_ros::IntResRequest
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

(defclass vmxpi_ros::IntResResponse
  :super ros::object
  :slots (_data ))

(defmethod vmxpi_ros::IntResResponse
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
    ;; int16 _data
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _data
       (write-word _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _data
     (setq _data (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass vmxpi_ros::IntRes
  :super ros::object
  :slots ())

(setf (get vmxpi_ros::IntRes :md5sum-) "8524586e34fbd7cb1c08c5f5f1ca0e57")
(setf (get vmxpi_ros::IntRes :datatype-) "vmxpi_ros/IntRes")
(setf (get vmxpi_ros::IntRes :request) vmxpi_ros::IntResRequest)
(setf (get vmxpi_ros::IntRes :response) vmxpi_ros::IntResResponse)

(defmethod vmxpi_ros::IntResRequest
  (:response () (instance vmxpi_ros::IntResResponse :init)))

(setf (get vmxpi_ros::IntResRequest :md5sum-) "8524586e34fbd7cb1c08c5f5f1ca0e57")
(setf (get vmxpi_ros::IntResRequest :datatype-) "vmxpi_ros/IntResRequest")
(setf (get vmxpi_ros::IntResRequest :definition-)
      "---
int16 data
")

(setf (get vmxpi_ros::IntResResponse :md5sum-) "8524586e34fbd7cb1c08c5f5f1ca0e57")
(setf (get vmxpi_ros::IntResResponse :datatype-) "vmxpi_ros/IntResResponse")
(setf (get vmxpi_ros::IntResResponse :definition-)
      "---
int16 data
")



(provide :vmxpi_ros/IntRes "8524586e34fbd7cb1c08c5f5f1ca0e57")


