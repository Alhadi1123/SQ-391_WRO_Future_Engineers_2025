;; Auto-generated. Do not edit!


(when (boundp 'obstacle_challenge::Pillar)
  (if (not (find-package "OBSTACLE_CHALLENGE"))
    (make-package "OBSTACLE_CHALLENGE"))
  (shadow 'Pillar (find-package "OBSTACLE_CHALLENGE")))
(unless (find-package "OBSTACLE_CHALLENGE::PILLAR")
  (make-package "OBSTACLE_CHALLENGE::PILLAR"))

(in-package "ROS")
;;//! \htmlinclude Pillar.msg.html


(defclass obstacle_challenge::Pillar
  :super ros::object
  :slots (_centroid_x _centroid_y _distance _color ))

(defmethod obstacle_challenge::Pillar
  (:init
   (&key
    ((:centroid_x __centroid_x) 0.0)
    ((:centroid_y __centroid_y) 0.0)
    ((:distance __distance) 0.0)
    ((:color __color) 0)
    )
   (send-super :init)
   (setq _centroid_x (float __centroid_x))
   (setq _centroid_y (float __centroid_y))
   (setq _distance (float __distance))
   (setq _color (round __color))
   self)
  (:centroid_x
   (&optional __centroid_x)
   (if __centroid_x (setq _centroid_x __centroid_x)) _centroid_x)
  (:centroid_y
   (&optional __centroid_y)
   (if __centroid_y (setq _centroid_y __centroid_y)) _centroid_y)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:serialization-length
   ()
   (+
    ;; float32 _centroid_x
    4
    ;; float32 _centroid_y
    4
    ;; float32 _distance
    4
    ;; int8 _color
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _centroid_x
       (sys::poke _centroid_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _centroid_y
       (sys::poke _centroid_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _color
       (write-byte _color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _centroid_x
     (setq _centroid_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _centroid_y
     (setq _centroid_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _color
     (setq _color (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _color 127) (setq _color (- _color 256)))
   ;;
   self)
  )

(setf (get obstacle_challenge::Pillar :md5sum-) "577a76ead12b846a3fe1693414bc0bdc")
(setf (get obstacle_challenge::Pillar :datatype-) "obstacle_challenge/Pillar")
(setf (get obstacle_challenge::Pillar :definition-)
      "float32 centroid_x
float32 centroid_y
float32 distance
int8 color
")



(provide :obstacle_challenge/Pillar "577a76ead12b846a3fe1693414bc0bdc")


