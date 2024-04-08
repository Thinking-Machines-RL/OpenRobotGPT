;; Auto-generated. Do not edit!


(when (boundp 'gymnasium_env::Action)
  (if (not (find-package "GYMNASIUM_ENV"))
    (make-package "GYMNASIUM_ENV"))
  (shadow 'Action (find-package "GYMNASIUM_ENV")))
(unless (find-package "GYMNASIUM_ENV::ACTION")
  (make-package "GYMNASIUM_ENV::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html


(defclass gymnasium_env::Action
  :super ros::object
  :slots (_action ))

(defmethod gymnasium_env::Action
  (:init
   (&key
    ((:action __action) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _action __action)
   self)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:serialization-length
   ()
   (+
    ;; float32[] _action
    (* 4    (length _action)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _action
     (write-long (length _action) s)
     (dotimes (i (length _action))
       (sys::poke (elt _action i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _action
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _action (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _action i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get gymnasium_env::Action :md5sum-) "a70a7d92e0376dcb967914076f276ea6")
(setf (get gymnasium_env::Action :datatype-) "gymnasium_env/Action")
(setf (get gymnasium_env::Action :definition-)
      "# Message sending action to env

float32[] action
")



(provide :gymnasium_env/Action "a70a7d92e0376dcb967914076f276ea6")


