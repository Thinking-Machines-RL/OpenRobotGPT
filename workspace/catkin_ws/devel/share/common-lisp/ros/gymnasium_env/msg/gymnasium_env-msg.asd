
(cl:in-package :asdf)

(defsystem "gymnasium_env-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Action" :depends-on ("_package_Action"))
    (:file "_package_Action" :depends-on ("_package"))
    (:file "StateReward" :depends-on ("_package_StateReward"))
    (:file "_package_StateReward" :depends-on ("_package"))
  ))