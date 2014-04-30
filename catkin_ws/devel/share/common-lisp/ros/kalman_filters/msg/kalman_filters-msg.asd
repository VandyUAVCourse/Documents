
(cl:in-package :asdf)

(defsystem "kalman_filters-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pose_ukf_msg" :depends-on ("_package_pose_ukf_msg"))
    (:file "_package_pose_ukf_msg" :depends-on ("_package"))
    (:file "state_ukf_msg" :depends-on ("_package_state_ukf_msg"))
    (:file "_package_state_ukf_msg" :depends-on ("_package"))
  ))