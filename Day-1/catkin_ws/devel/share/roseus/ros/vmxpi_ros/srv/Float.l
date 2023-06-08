;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::Float)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'Float (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::FLOAT")
  (make-package "VMXPI_ROS::FLOAT"))
(unless (find-package "VMXPI_ROS::FLOATREQUEST")
  (make-package "VMXPI_ROS::FLOATREQUEST"))
(unless (find-package "VMXPI_ROS::FLOATRESPONSE")
  (make-package "VMXPI_ROS::FLOATRESPONSE"))

(in-package "ROS")





(defclass vmxpi_ros::FloatRequest
  :super ros::object
  :slots (_data ))

(defmethod vmxpi_ros::FloatRequest
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass vmxpi_ros::FloatResponse
  :super ros::object
  :slots ())

(defmethod vmxpi_ros::FloatResponse
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

(defclass vmxpi_ros::Float
  :super ros::object
  :slots ())

(setf (get vmxpi_ros::Float :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get vmxpi_ros::Float :datatype-) "vmxpi_ros/Float")
(setf (get vmxpi_ros::Float :request) vmxpi_ros::FloatRequest)
(setf (get vmxpi_ros::Float :response) vmxpi_ros::FloatResponse)

(defmethod vmxpi_ros::FloatRequest
  (:response () (instance vmxpi_ros::FloatResponse :init)))

(setf (get vmxpi_ros::FloatRequest :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get vmxpi_ros::FloatRequest :datatype-) "vmxpi_ros/FloatRequest")
(setf (get vmxpi_ros::FloatRequest :definition-)
      "float32 data
---
")

(setf (get vmxpi_ros::FloatResponse :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get vmxpi_ros::FloatResponse :datatype-) "vmxpi_ros/FloatResponse")
(setf (get vmxpi_ros::FloatResponse :definition-)
      "float32 data
---
")



(provide :vmxpi_ros/Float "73fcbf46b49191e672908e50842a83d4")


