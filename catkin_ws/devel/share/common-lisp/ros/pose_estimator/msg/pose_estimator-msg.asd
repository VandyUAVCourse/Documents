
(cl:in-package :asdf)

(defsystem "pose_estimator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "proj_2_5d_msg" :depends-on ("_package_proj_2_5d_msg"))
    (:file "_package_proj_2_5d_msg" :depends-on ("_package"))
    (:file "pose_estimator_msg" :depends-on ("_package_pose_estimator_msg"))
    (:file "_package_pose_estimator_msg" :depends-on ("_package"))
    (:file "point2d_t" :depends-on ("_package_point2d_t"))
    (:file "_package_point2d_t" :depends-on ("_package"))
  ))