; Auto-generated. Do not edit!


(cl:in-package obstacle_challenge-msg)


;//! \htmlinclude Pillar.msg.html

(cl:defclass <Pillar> (roslisp-msg-protocol:ros-message)
  ((centroid_x
    :reader centroid_x
    :initarg :centroid_x
    :type cl:float
    :initform 0.0)
   (centroid_y
    :reader centroid_y
    :initarg :centroid_y
    :type cl:float
    :initform 0.0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (color
    :reader color
    :initarg :color
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Pillar (<Pillar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pillar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pillar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name obstacle_challenge-msg:<Pillar> is deprecated: use obstacle_challenge-msg:Pillar instead.")))

(cl:ensure-generic-function 'centroid_x-val :lambda-list '(m))
(cl:defmethod centroid_x-val ((m <Pillar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader obstacle_challenge-msg:centroid_x-val is deprecated.  Use obstacle_challenge-msg:centroid_x instead.")
  (centroid_x m))

(cl:ensure-generic-function 'centroid_y-val :lambda-list '(m))
(cl:defmethod centroid_y-val ((m <Pillar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader obstacle_challenge-msg:centroid_y-val is deprecated.  Use obstacle_challenge-msg:centroid_y instead.")
  (centroid_y m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <Pillar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader obstacle_challenge-msg:distance-val is deprecated.  Use obstacle_challenge-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <Pillar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader obstacle_challenge-msg:color-val is deprecated.  Use obstacle_challenge-msg:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pillar>) ostream)
  "Serializes a message object of type '<Pillar>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'centroid_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'centroid_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'color)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pillar>) istream)
  "Deserializes a message object of type '<Pillar>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'centroid_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'centroid_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pillar>)))
  "Returns string type for a message object of type '<Pillar>"
  "obstacle_challenge/Pillar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pillar)))
  "Returns string type for a message object of type 'Pillar"
  "obstacle_challenge/Pillar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pillar>)))
  "Returns md5sum for a message object of type '<Pillar>"
  "577a76ead12b846a3fe1693414bc0bdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pillar)))
  "Returns md5sum for a message object of type 'Pillar"
  "577a76ead12b846a3fe1693414bc0bdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pillar>)))
  "Returns full string definition for message of type '<Pillar>"
  (cl:format cl:nil "float32 centroid_x~%float32 centroid_y~%float32 distance~%int8 color~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pillar)))
  "Returns full string definition for message of type 'Pillar"
  (cl:format cl:nil "float32 centroid_x~%float32 centroid_y~%float32 distance~%int8 color~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pillar>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pillar>))
  "Converts a ROS message object to a list"
  (cl:list 'Pillar
    (cl:cons ':centroid_x (centroid_x msg))
    (cl:cons ':centroid_y (centroid_y msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':color (color msg))
))
