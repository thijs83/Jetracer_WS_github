;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs_optitrack::custom_opti_pose_stamped_msg)
  (if (not (find-package "CUSTOM_MSGS_OPTITRACK"))
    (make-package "CUSTOM_MSGS_OPTITRACK"))
  (shadow 'custom_opti_pose_stamped_msg (find-package "CUSTOM_MSGS_OPTITRACK")))
(unless (find-package "CUSTOM_MSGS_OPTITRACK::CUSTOM_OPTI_POSE_STAMPED_MSG")
  (make-package "CUSTOM_MSGS_OPTITRACK::CUSTOM_OPTI_POSE_STAMPED_MSG"))

(in-package "ROS")
;;//! \htmlinclude custom_opti_pose_stamped_msg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_msgs_optitrack::custom_opti_pose_stamped_msg
  :super ros::object
  :slots (_header _x _y _rotation ))

(defmethod custom_msgs_optitrack::custom_opti_pose_stamped_msg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:rotation __rotation) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _rotation (float __rotation))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:rotation
   (&optional __rotation)
   (if __rotation (setq _rotation __rotation)) _rotation)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _rotation
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotation
       (sys::poke _rotation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotation
     (setq _rotation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msgs_optitrack::custom_opti_pose_stamped_msg :md5sum-) "70bfefdfc7d1f29f125789ac694c643e")
(setf (get custom_msgs_optitrack::custom_opti_pose_stamped_msg :datatype-) "custom_msgs_optitrack/custom_opti_pose_stamped_msg")
(setf (get custom_msgs_optitrack::custom_opti_pose_stamped_msg :definition-)
      "std_msgs/Header header
float32 x
float32 y
float32 rotation

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :custom_msgs_optitrack/custom_opti_pose_stamped_msg "70bfefdfc7d1f29f125789ac694c643e")


