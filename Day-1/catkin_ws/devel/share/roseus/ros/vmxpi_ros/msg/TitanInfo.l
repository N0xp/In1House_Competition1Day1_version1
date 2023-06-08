;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::TitanInfo)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'TitanInfo (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::TITANINFO")
  (make-package "VMXPI_ROS::TITANINFO"))

(in-package "ROS")
;;//! \htmlinclude TitanInfo.msg.html


(defclass vmxpi_ros::TitanInfo
  :super ros::object
  :slots (_deviceID _verMaj _verMin _verBuild _hardware _hardwareRev ))

(defmethod vmxpi_ros::TitanInfo
  (:init
   (&key
    ((:deviceID __deviceID) 0)
    ((:verMaj __verMaj) 0)
    ((:verMin __verMin) 0)
    ((:verBuild __verBuild) 0)
    ((:hardware __hardware) 0)
    ((:hardwareRev __hardwareRev) 0)
    )
   (send-super :init)
   (setq _deviceID (round __deviceID))
   (setq _verMaj (round __verMaj))
   (setq _verMin (round __verMin))
   (setq _verBuild (round __verBuild))
   (setq _hardware (round __hardware))
   (setq _hardwareRev (round __hardwareRev))
   self)
  (:deviceID
   (&optional __deviceID)
   (if __deviceID (setq _deviceID __deviceID)) _deviceID)
  (:verMaj
   (&optional __verMaj)
   (if __verMaj (setq _verMaj __verMaj)) _verMaj)
  (:verMin
   (&optional __verMin)
   (if __verMin (setq _verMin __verMin)) _verMin)
  (:verBuild
   (&optional __verBuild)
   (if __verBuild (setq _verBuild __verBuild)) _verBuild)
  (:hardware
   (&optional __hardware)
   (if __hardware (setq _hardware __hardware)) _hardware)
  (:hardwareRev
   (&optional __hardwareRev)
   (if __hardwareRev (setq _hardwareRev __hardwareRev)) _hardwareRev)
  (:serialization-length
   ()
   (+
    ;; int32 _deviceID
    4
    ;; int32 _verMaj
    4
    ;; int32 _verMin
    4
    ;; int32 _verBuild
    4
    ;; int32 _hardware
    4
    ;; int32 _hardwareRev
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _deviceID
       (write-long _deviceID s)
     ;; int32 _verMaj
       (write-long _verMaj s)
     ;; int32 _verMin
       (write-long _verMin s)
     ;; int32 _verBuild
       (write-long _verBuild s)
     ;; int32 _hardware
       (write-long _hardware s)
     ;; int32 _hardwareRev
       (write-long _hardwareRev s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _deviceID
     (setq _deviceID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _verMaj
     (setq _verMaj (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _verMin
     (setq _verMin (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _verBuild
     (setq _verBuild (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _hardware
     (setq _hardware (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _hardwareRev
     (setq _hardwareRev (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get vmxpi_ros::TitanInfo :md5sum-) "5c9aee76b3d5cddac787b5c807fa2b41")
(setf (get vmxpi_ros::TitanInfo :datatype-) "vmxpi_ros/TitanInfo")
(setf (get vmxpi_ros::TitanInfo :definition-)
      "int32 deviceID
int32 verMaj
int32 verMin
int32 verBuild
int32 hardware
int32 hardwareRev
")



(provide :vmxpi_ros/TitanInfo "5c9aee76b3d5cddac787b5c807fa2b41")


