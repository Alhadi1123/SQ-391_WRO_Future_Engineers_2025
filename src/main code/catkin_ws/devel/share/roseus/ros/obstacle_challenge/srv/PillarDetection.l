;; Auto-generated. Do not edit!


(when (boundp 'obstacle_challenge::PillarDetection)
  (if (not (find-package "OBSTACLE_CHALLENGE"))
    (make-package "OBSTACLE_CHALLENGE"))
  (shadow 'PillarDetection (find-package "OBSTACLE_CHALLENGE")))
(unless (find-package "OBSTACLE_CHALLENGE::PILLARDETECTION")
  (make-package "OBSTACLE_CHALLENGE::PILLARDETECTION"))
(unless (find-package "OBSTACLE_CHALLENGE::PILLARDETECTIONREQUEST")
  (make-package "OBSTACLE_CHALLENGE::PILLARDETECTIONREQUEST"))
(unless (find-package "OBSTACLE_CHALLENGE::PILLARDETECTIONRESPONSE")
  (make-package "OBSTACLE_CHALLENGE::PILLARDETECTIONRESPONSE"))

(in-package "ROS")





(defclass obstacle_challenge::PillarDetectionRequest
  :super ros::object
  :slots ())

(defmethod obstacle_challenge::PillarDetectionRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass obstacle_challenge::PillarDetectionResponse
  :super ros::object
  :slots (_pillars ))

(defmethod obstacle_challenge::PillarDetectionResponse
  (:init
   (&key
    ((:pillars __pillars) ())
    )
   (send-super :init)
   (setq _pillars __pillars)
   self)
  (:pillars
   (&rest __pillars)
   (if (keywordp (car __pillars))
       (send* _pillars __pillars)
     (progn
       (if __pillars (setq _pillars (car __pillars)))
       _pillars)))
  (:serialization-length
   ()
   (+
    ;; obstacle_challenge/Pillar[] _pillars
    (apply #'+ (send-all _pillars :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; obstacle_challenge/Pillar[] _pillars
     (write-long (length _pillars) s)
     (dolist (elem _pillars)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; obstacle_challenge/Pillar[] _pillars
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pillars (let (r) (dotimes (i n) (push (instance obstacle_challenge::Pillar :init) r)) r))
     (dolist (elem- _pillars)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass obstacle_challenge::PillarDetection
  :super ros::object
  :slots ())

(setf (get obstacle_challenge::PillarDetection :md5sum-) "05c01fe013cbbc393a81b8e5fc3bc0a8")
(setf (get obstacle_challenge::PillarDetection :datatype-) "obstacle_challenge/PillarDetection")
(setf (get obstacle_challenge::PillarDetection :request) obstacle_challenge::PillarDetectionRequest)
(setf (get obstacle_challenge::PillarDetection :response) obstacle_challenge::PillarDetectionResponse)

(defmethod obstacle_challenge::PillarDetectionRequest
  (:response () (instance obstacle_challenge::PillarDetectionResponse :init)))

(setf (get obstacle_challenge::PillarDetectionRequest :md5sum-) "05c01fe013cbbc393a81b8e5fc3bc0a8")
(setf (get obstacle_challenge::PillarDetectionRequest :datatype-) "obstacle_challenge/PillarDetectionRequest")
(setf (get obstacle_challenge::PillarDetectionRequest :definition-)
      "# Request (empty since client just requests data)
---
# Response
Pillar[] pillars




================================================================================
MSG: obstacle_challenge/Pillar
float32 centroid_x
float32 centroid_y
float32 distance
")

(setf (get obstacle_challenge::PillarDetectionResponse :md5sum-) "05c01fe013cbbc393a81b8e5fc3bc0a8")
(setf (get obstacle_challenge::PillarDetectionResponse :datatype-) "obstacle_challenge/PillarDetectionResponse")
(setf (get obstacle_challenge::PillarDetectionResponse :definition-)
      "# Request (empty since client just requests data)
---
# Response
Pillar[] pillars




================================================================================
MSG: obstacle_challenge/Pillar
float32 centroid_x
float32 centroid_y
float32 distance
")



(provide :obstacle_challenge/PillarDetection "05c01fe013cbbc393a81b8e5fc3bc0a8")


