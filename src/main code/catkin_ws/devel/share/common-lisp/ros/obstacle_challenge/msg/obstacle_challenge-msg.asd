
(cl:in-package :asdf)

(defsystem "obstacle_challenge-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Pillar" :depends-on ("_package_Pillar"))
    (:file "_package_Pillar" :depends-on ("_package"))
    (:file "ultraInfo" :depends-on ("_package_ultraInfo"))
    (:file "_package_ultraInfo" :depends-on ("_package"))
  ))