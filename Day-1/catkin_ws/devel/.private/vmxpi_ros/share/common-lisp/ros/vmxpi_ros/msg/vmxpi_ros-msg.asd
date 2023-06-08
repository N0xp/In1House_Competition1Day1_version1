
(cl:in-package :asdf)

(defsystem "vmxpi_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LimitSwitch" :depends-on ("_package_LimitSwitch"))
    (:file "_package_LimitSwitch" :depends-on ("_package"))
    (:file "TitanInfo" :depends-on ("_package_TitanInfo"))
    (:file "_package_TitanInfo" :depends-on ("_package"))
    (:file "UniqueID" :depends-on ("_package_UniqueID"))
    (:file "_package_UniqueID" :depends-on ("_package"))
  ))