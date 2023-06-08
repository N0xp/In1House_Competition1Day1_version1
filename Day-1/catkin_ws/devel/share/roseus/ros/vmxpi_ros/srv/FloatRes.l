;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::FloatRes)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'FloatRes (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::FLOATRES")
  (make-package "VMXPI_ROS::FLOATRES"))
(unless (find-package "VMXPI_ROS::FLOATRESREQUEST")
  (make-package "VMXPI_ROS::FLOATRESREQUEST"))
(unless (find-package "VMXPI_ROS::FLOATRESRESPONSE")
  (make-package "VMXPI_ROS::FLOATRESRESPONSE"))

(in-package "ROS")





(defclass vmxpi_ros::FloatResRequest
  :super ros::object
  :slots ())

(defmethod vmxpi_ros::FloatResRequest
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

(defclass vmxpi_ros::FloatResResponse
  :super ros::object
  :slots (_data ))

(defmethod vmxpi_ros::FloatResResponse
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

(defclass vmxpi_ros::FloatRes
  :super ros::object
  :slots ())

(setf (get vmxpi_ros::FloatRes :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get vmxpi_ros::FloatRes :datatype-) "vmxpi_ros/FloatRes")
(setf (get vmxpi_ros::FloatRes :request) vmxpi_ros::FloatResRequest)
(setf (get vmxpi_ros::FloatRes :response) vmxpi_ros::FloatResResponse)

(defmethod vmxpi_ros::FloatResRequest
  (:response () (instance vmxpi_ros::FloatResResponse :init)))

(setf (get vmxpi_ros::FloatResRequest :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get vmxpi_ros::FloatResRequest :datatype-) "vmxpi_ros/FloatResRequest")
(setf (get vmxpi_ros::FloatResRequest :definition-)
      "---
float32 data
")

(setf (get vmxpi_ros::FloatResResponse :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get vmxpi_ros::FloatResResponse :datatype-) "vmxpi_ros/FloatResResponse")
(setf (get vmxpi_ros::FloatResResponse :definition-)
      "---
float32 data
")



(provide :vmxpi_ros/FloatRes "73fcbf46b49191e672908e50842a83d4")


