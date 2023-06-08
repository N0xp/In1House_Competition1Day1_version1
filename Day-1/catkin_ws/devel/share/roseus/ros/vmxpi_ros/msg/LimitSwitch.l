;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::LimitSwitch)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'LimitSwitch (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::LIMITSWITCH")
  (make-package "VMXPI_ROS::LIMITSWITCH"))

(in-package "ROS")
;;//! \htmlinclude LimitSwitch.msg.html


(defclass vmxpi_ros::LimitSwitch
  :super ros::object
  :slots (_limitswitch ))

(defmethod vmxpi_ros::LimitSwitch
  (:init
   (&key
    ((:limitswitch __limitswitch) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _limitswitch __limitswitch)
   self)
  (:limitswitch
   (&optional __limitswitch)
   (if __limitswitch (setq _limitswitch __limitswitch)) _limitswitch)
  (:serialization-length
   ()
   (+
    ;; bool[] _limitswitch
    (* 1    (length _limitswitch)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool[] _limitswitch
     (write-long (length _limitswitch) s)
     (dotimes (i (length _limitswitch))
       (if (elt _limitswitch i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool[] _limitswitch
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _limitswitch (make-list n))
     (dotimes (i n)
     (setf (elt _limitswitch i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(setf (get vmxpi_ros::LimitSwitch :md5sum-) "cabe49bdbfdd4a3cf170660bf17038a4")
(setf (get vmxpi_ros::LimitSwitch :datatype-) "vmxpi_ros/LimitSwitch")
(setf (get vmxpi_ros::LimitSwitch :definition-)
      "bool[] limitswitch

")



(provide :vmxpi_ros/LimitSwitch "cabe49bdbfdd4a3cf170660bf17038a4")


