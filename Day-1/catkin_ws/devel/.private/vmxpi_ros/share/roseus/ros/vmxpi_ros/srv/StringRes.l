;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::StringRes)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'StringRes (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::STRINGRES")
  (make-package "VMXPI_ROS::STRINGRES"))
(unless (find-package "VMXPI_ROS::STRINGRESREQUEST")
  (make-package "VMXPI_ROS::STRINGRESREQUEST"))
(unless (find-package "VMXPI_ROS::STRINGRESRESPONSE")
  (make-package "VMXPI_ROS::STRINGRESRESPONSE"))

(in-package "ROS")





(defclass vmxpi_ros::StringResRequest
  :super ros::object
  :slots ())

(defmethod vmxpi_ros::StringResRequest
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

(defclass vmxpi_ros::StringResResponse
  :super ros::object
  :slots (_data ))

(defmethod vmxpi_ros::StringResResponse
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass vmxpi_ros::StringRes
  :super ros::object
  :slots ())

(setf (get vmxpi_ros::StringRes :md5sum-) "992ce8a1687cec8c8bd883ec73ca41d1")
(setf (get vmxpi_ros::StringRes :datatype-) "vmxpi_ros/StringRes")
(setf (get vmxpi_ros::StringRes :request) vmxpi_ros::StringResRequest)
(setf (get vmxpi_ros::StringRes :response) vmxpi_ros::StringResResponse)

(defmethod vmxpi_ros::StringResRequest
  (:response () (instance vmxpi_ros::StringResResponse :init)))

(setf (get vmxpi_ros::StringResRequest :md5sum-) "992ce8a1687cec8c8bd883ec73ca41d1")
(setf (get vmxpi_ros::StringResRequest :datatype-) "vmxpi_ros/StringResRequest")
(setf (get vmxpi_ros::StringResRequest :definition-)
      "---
string data
")

(setf (get vmxpi_ros::StringResResponse :md5sum-) "992ce8a1687cec8c8bd883ec73ca41d1")
(setf (get vmxpi_ros::StringResResponse :datatype-) "vmxpi_ros/StringResResponse")
(setf (get vmxpi_ros::StringResResponse :definition-)
      "---
string data
")



(provide :vmxpi_ros/StringRes "992ce8a1687cec8c8bd883ec73ca41d1")


