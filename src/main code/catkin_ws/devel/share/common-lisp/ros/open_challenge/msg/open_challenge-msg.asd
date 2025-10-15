
(cl:in-package :asdf)

(defsystem "open_challenge-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ultraInfo" :depends-on ("_package_ultraInfo"))
    (:file "_package_ultraInfo" :depends-on ("_package"))
  ))