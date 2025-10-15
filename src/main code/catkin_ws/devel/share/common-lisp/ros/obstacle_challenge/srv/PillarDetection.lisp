; Auto-generated. Do not edit!


(cl:in-package obstacle_challenge-srv)


;//! \htmlinclude PillarDetection-request.msg.html

(cl:defclass <PillarDetection-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PillarDetection-request (<PillarDetection-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PillarDetection-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PillarDetection-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name obstacle_challenge-srv:<PillarDetection-request> is deprecated: use obstacle_challenge-srv:PillarDetection-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PillarDetection-request>) ostream)
  "Serializes a message object of type '<PillarDetection-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PillarDetection-request>) istream)
  "Deserializes a message object of type '<PillarDetection-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PillarDetection-request>)))
  "Returns string type for a service object of type '<PillarDetection-request>"
  "obstacle_challenge/PillarDetectionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PillarDetection-request)))
  "Returns string type for a service object of type 'PillarDetection-request"
  "obstacle_challenge/PillarDetectionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PillarDetection-request>)))
  "Returns md5sum for a message object of type '<PillarDetection-request>"
  "05c01fe013cbbc393a81b8e5fc3bc0a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PillarDetection-request)))
  "Returns md5sum for a message object of type 'PillarDetection-request"
  "05c01fe013cbbc393a81b8e5fc3bc0a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PillarDetection-request>)))
  "Returns full string definition for message of type '<PillarDetection-request>"
  (cl:format cl:nil "# Request (empty since client just requests data)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PillarDetection-request)))
  "Returns full string definition for message of type 'PillarDetection-request"
  (cl:format cl:nil "# Request (empty since client just requests data)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PillarDetection-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PillarDetection-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PillarDetection-request
))
;//! \htmlinclude PillarDetection-response.msg.html

(cl:defclass <PillarDetection-response> (roslisp-msg-protocol:ros-message)
  ((pillars
    :reader pillars
    :initarg :pillars
    :type (cl:vector obstacle_challenge-msg:Pillar)
   :initform (cl:make-array 0 :element-type 'obstacle_challenge-msg:Pillar :initial-element (cl:make-instance 'obstacle_challenge-msg:Pillar))))
)

(cl:defclass PillarDetection-response (<PillarDetection-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PillarDetection-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PillarDetection-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name obstacle_challenge-srv:<PillarDetection-response> is deprecated: use obstacle_challenge-srv:PillarDetection-response instead.")))

(cl:ensure-generic-function 'pillars-val :lambda-list '(m))
(cl:defmethod pillars-val ((m <PillarDetection-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader obstacle_challenge-srv:pillars-val is deprecated.  Use obstacle_challenge-srv:pillars instead.")
  (pillars m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PillarDetection-response>) ostream)
  "Serializes a message object of type '<PillarDetection-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pillars))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pillars))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PillarDetection-response>) istream)
  "Deserializes a message object of type '<PillarDetection-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pillars) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pillars)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'obstacle_challenge-msg:Pillar))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PillarDetection-response>)))
  "Returns string type for a service object of type '<PillarDetection-response>"
  "obstacle_challenge/PillarDetectionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PillarDetection-response)))
  "Returns string type for a service object of type 'PillarDetection-response"
  "obstacle_challenge/PillarDetectionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PillarDetection-response>)))
  "Returns md5sum for a message object of type '<PillarDetection-response>"
  "05c01fe013cbbc393a81b8e5fc3bc0a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PillarDetection-response)))
  "Returns md5sum for a message object of type 'PillarDetection-response"
  "05c01fe013cbbc393a81b8e5fc3bc0a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PillarDetection-response>)))
  "Returns full string definition for message of type '<PillarDetection-response>"
  (cl:format cl:nil "# Response~%Pillar[] pillars~%~%~%~%~%================================================================================~%MSG: obstacle_challenge/Pillar~%float32 centroid_x~%float32 centroid_y~%float32 distance~%int8 color~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PillarDetection-response)))
  "Returns full string definition for message of type 'PillarDetection-response"
  (cl:format cl:nil "# Response~%Pillar[] pillars~%~%~%~%~%================================================================================~%MSG: obstacle_challenge/Pillar~%float32 centroid_x~%float32 centroid_y~%float32 distance~%int8 color~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PillarDetection-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pillars) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PillarDetection-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PillarDetection-response
    (cl:cons ':pillars (pillars msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PillarDetection)))
  'PillarDetection-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PillarDetection)))
  'PillarDetection-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PillarDetection)))
  "Returns string type for a service object of type '<PillarDetection>"
  "obstacle_challenge/PillarDetection")