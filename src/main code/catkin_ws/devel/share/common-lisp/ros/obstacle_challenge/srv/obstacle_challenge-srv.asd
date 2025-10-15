
(cl:in-package :asdf)

(defsystem "obstacle_challenge-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :obstacle_challenge-msg
)
  :components ((:file "_package")
    (:file "PillarDetection" :depends-on ("_package_PillarDetection"))
    (:file "_package_PillarDetection" :depends-on ("_package"))
  ))