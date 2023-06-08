;; Auto-generated. Do not edit!


(when (boundp 'vmxpi_ros::UniqueID)
  (if (not (find-package "VMXPI_ROS"))
    (make-package "VMXPI_ROS"))
  (shadow 'UniqueID (find-package "VMXPI_ROS")))
(unless (find-package "VMXPI_ROS::UNIQUEID")
  (make-package "VMXPI_ROS::UNIQUEID"))

(in-package "ROS")
;;//! \htmlinclude UniqueID.msg.html


(defclass vmxpi_ros::UniqueID
  :super ros::object
  :slots (_word1 _word2 _word3 ))

(defmethod vmxpi_ros::UniqueID
  (:init
   (&key
    ((:word1 __word1) "")
    ((:word2 __word2) "")
    ((:word3 __word3) "")
    )
   (send-super :init)
   (setq _word1 (string __word1))
   (setq _word2 (string __word2))
   (setq _word3 (string __word3))
   self)
  (:word1
   (&optional __word1)
   (if __word1 (setq _word1 __word1)) _word1)
  (:word2
   (&optional __word2)
   (if __word2 (setq _word2 __word2)) _word2)
  (:word3
   (&optional __word3)
   (if __word3 (setq _word3 __word3)) _word3)
  (:serialization-length
   ()
   (+
    ;; string _word1
    4 (length _word1)
    ;; string _word2
    4 (length _word2)
    ;; string _word3
    4 (length _word3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _word1
       (write-long (length _word1) s) (princ _word1 s)
     ;; string _word2
       (write-long (length _word2) s) (princ _word2 s)
     ;; string _word3
       (write-long (length _word3) s) (princ _word3 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _word1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _word1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _word2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _word2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _word3
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _word3 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get vmxpi_ros::UniqueID :md5sum-) "d3fe5a40e07321a3fd3e3e35d9b0c93b")
(setf (get vmxpi_ros::UniqueID :datatype-) "vmxpi_ros/UniqueID")
(setf (get vmxpi_ros::UniqueID :definition-)
      "string word1
string word2
string word3
")



(provide :vmxpi_ros/UniqueID "d3fe5a40e07321a3fd3e3e35d9b0c93b")


