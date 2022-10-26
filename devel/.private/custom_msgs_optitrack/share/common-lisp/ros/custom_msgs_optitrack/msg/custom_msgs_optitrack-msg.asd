
(cl:in-package :asdf)

(defsystem "custom_msgs_optitrack-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "custom_opti_pose_stamped_msg" :depends-on ("_package_custom_opti_pose_stamped_msg"))
    (:file "_package_custom_opti_pose_stamped_msg" :depends-on ("_package"))
  ))