; Auto-generated. Do not edit!


(cl:in-package open_challenge-msg)


;//! \htmlinclude ultraInfo.msg.html

(cl:defclass <ultraInfo> (roslisp-msg-protocol:ros-message)
  ((nums
    :reader nums
    :initarg :nums
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ultraInfo (<ultraInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ultraInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ultraInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_challenge-msg:<ultraInfo> is deprecated: use open_challenge-msg:ultraInfo instead.")))

(cl:ensure-generic-function 'nums-val :lambda-list '(m))
(cl:defmethod nums-val ((m <ultraInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_challenge-msg:nums-val is deprecated.  Use open_challenge-msg:nums instead.")
  (nums m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ultraInfo>) ostream)
  "Serializes a message object of type '<ultraInfo>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nums))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'nums))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ultraInfo>) istream)
  "Deserializes a message object of type '<ultraInfo>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nums) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nums)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ultraInfo>)))
  "Returns string type for a message object of type '<ultraInfo>"
  "open_challenge/ultraInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ultraInfo)))
  "Returns string type for a message object of type 'ultraInfo"
  "open_challenge/ultraInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ultraInfo>)))
  "Returns md5sum for a message object of type '<ultraInfo>"
  "8a327f765b70d8bd267a8d96beb763ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ultraInfo)))
  "Returns md5sum for a message object of type 'ultraInfo"
  "8a327f765b70d8bd267a8d96beb763ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ultraInfo>)))
  "Returns full string definition for message of type '<ultraInfo>"
  (cl:format cl:nil "float32[] nums ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ultraInfo)))
  "Returns full string definition for message of type 'ultraInfo"
  (cl:format cl:nil "float32[] nums ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ultraInfo>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nums) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ultraInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ultraInfo
    (cl:cons ':nums (nums msg))
))
