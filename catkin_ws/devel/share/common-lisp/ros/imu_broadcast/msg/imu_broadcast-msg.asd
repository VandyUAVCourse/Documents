
(cl:in-package :asdf)

(defsystem "imu_broadcast-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "raw_imu" :depends-on ("_package_raw_imu"))
    (:file "_package_raw_imu" :depends-on ("_package"))
    (:file "attitude" :depends-on ("_package_attitude"))
    (:file "_package_attitude" :depends-on ("_package"))
  ))