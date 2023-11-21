; Auto-generated. Do not edit!


(cl:in-package custom_msgs_optitrack-msg)


;//! \htmlinclude custom_opti_pose_stamped_msg.msg.html

(cl:defclass <custom_opti_pose_stamped_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (rotation
    :reader rotation
    :initarg :rotation
    :type cl:float
    :initform 0.0)
   (car_number
    :reader car_number
    :initarg :car_number
    :type cl:float
    :initform 0.0))
)

(cl:defclass custom_opti_pose_stamped_msg (<custom_opti_pose_stamped_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <custom_opti_pose_stamped_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'custom_opti_pose_stamped_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs_optitrack-msg:<custom_opti_pose_stamped_msg> is deprecated: use custom_msgs_optitrack-msg:custom_opti_pose_stamped_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <custom_opti_pose_stamped_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs_optitrack-msg:header-val is deprecated.  Use custom_msgs_optitrack-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <custom_opti_pose_stamped_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs_optitrack-msg:x-val is deprecated.  Use custom_msgs_optitrack-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <custom_opti_pose_stamped_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs_optitrack-msg:y-val is deprecated.  Use custom_msgs_optitrack-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <custom_opti_pose_stamped_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs_optitrack-msg:rotation-val is deprecated.  Use custom_msgs_optitrack-msg:rotation instead.")
  (rotation m))

(cl:ensure-generic-function 'car_number-val :lambda-list '(m))
(cl:defmethod car_number-val ((m <custom_opti_pose_stamped_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs_optitrack-msg:car_number-val is deprecated.  Use custom_msgs_optitrack-msg:car_number instead.")
  (car_number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <custom_opti_pose_stamped_msg>) ostream)
  "Serializes a message object of type '<custom_opti_pose_stamped_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'car_number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <custom_opti_pose_stamped_msg>) istream)
  "Deserializes a message object of type '<custom_opti_pose_stamped_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'car_number) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<custom_opti_pose_stamped_msg>)))
  "Returns string type for a message object of type '<custom_opti_pose_stamped_msg>"
  "custom_msgs_optitrack/custom_opti_pose_stamped_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'custom_opti_pose_stamped_msg)))
  "Returns string type for a message object of type 'custom_opti_pose_stamped_msg"
  "custom_msgs_optitrack/custom_opti_pose_stamped_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<custom_opti_pose_stamped_msg>)))
  "Returns md5sum for a message object of type '<custom_opti_pose_stamped_msg>"
  "43beb3a48f3877dec12e819cc3cd4001")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'custom_opti_pose_stamped_msg)))
  "Returns md5sum for a message object of type 'custom_opti_pose_stamped_msg"
  "43beb3a48f3877dec12e819cc3cd4001")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<custom_opti_pose_stamped_msg>)))
  "Returns full string definition for message of type '<custom_opti_pose_stamped_msg>"
  (cl:format cl:nil "std_msgs/Header header~%float32 x~%float32 y~%float32 rotation~%float32 car_number~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'custom_opti_pose_stamped_msg)))
  "Returns full string definition for message of type 'custom_opti_pose_stamped_msg"
  (cl:format cl:nil "std_msgs/Header header~%float32 x~%float32 y~%float32 rotation~%float32 car_number~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <custom_opti_pose_stamped_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <custom_opti_pose_stamped_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'custom_opti_pose_stamped_msg
    (cl:cons ':header (header msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':rotation (rotation msg))
    (cl:cons ':car_number (car_number msg))
))
