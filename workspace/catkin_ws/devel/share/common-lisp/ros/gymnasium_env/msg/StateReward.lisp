; Auto-generated. Do not edit!


(cl:in-package gymnasium_env-msg)


;//! \htmlinclude StateReward.msg.html

(cl:defclass <StateReward> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (info
    :reader info
    :initarg :info
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (reward
    :reader reward
    :initarg :reward
    :type cl:float
    :initform 0.0)
   (terminal
    :reader terminal
    :initarg :terminal
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StateReward (<StateReward>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateReward>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateReward)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gymnasium_env-msg:<StateReward> is deprecated: use gymnasium_env-msg:StateReward instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <StateReward>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gymnasium_env-msg:state-val is deprecated.  Use gymnasium_env-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <StateReward>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gymnasium_env-msg:info-val is deprecated.  Use gymnasium_env-msg:info instead.")
  (info m))

(cl:ensure-generic-function 'reward-val :lambda-list '(m))
(cl:defmethod reward-val ((m <StateReward>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gymnasium_env-msg:reward-val is deprecated.  Use gymnasium_env-msg:reward instead.")
  (reward m))

(cl:ensure-generic-function 'terminal-val :lambda-list '(m))
(cl:defmethod terminal-val ((m <StateReward>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gymnasium_env-msg:terminal-val is deprecated.  Use gymnasium_env-msg:terminal instead.")
  (terminal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateReward>) ostream)
  "Serializes a message object of type '<StateReward>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'state))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'info))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reward))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'terminal) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateReward>) istream)
  "Deserializes a message object of type '<StateReward>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'info) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'info)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reward) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'terminal) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateReward>)))
  "Returns string type for a message object of type '<StateReward>"
  "gymnasium_env/StateReward")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateReward)))
  "Returns string type for a message object of type 'StateReward"
  "gymnasium_env/StateReward")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateReward>)))
  "Returns md5sum for a message object of type '<StateReward>"
  "df77cdff2c07bc1682dbb96fddaaf25c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateReward)))
  "Returns md5sum for a message object of type 'StateReward"
  "df77cdff2c07bc1682dbb96fddaaf25c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateReward>)))
  "Returns full string definition for message of type '<StateReward>"
  (cl:format cl:nil "#Credits: RL ROS package~%# Message for returning the current sensation vector ~%# (i.e. state or observation or sensor readings) and a~%# reward from an  environment~%~%float32[] state~%float32[] info~%float32 reward~%bool terminal~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateReward)))
  "Returns full string definition for message of type 'StateReward"
  (cl:format cl:nil "#Credits: RL ROS package~%# Message for returning the current sensation vector ~%# (i.e. state or observation or sensor readings) and a~%# reward from an  environment~%~%float32[] state~%float32[] info~%float32 reward~%bool terminal~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateReward>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateReward>))
  "Converts a ROS message object to a list"
  (cl:list 'StateReward
    (cl:cons ':state (state msg))
    (cl:cons ':info (info msg))
    (cl:cons ':reward (reward msg))
    (cl:cons ':terminal (terminal msg))
))
