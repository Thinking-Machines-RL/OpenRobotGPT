;; Auto-generated. Do not edit!


(when (boundp 'gymnasium_env::StateReward)
  (if (not (find-package "GYMNASIUM_ENV"))
    (make-package "GYMNASIUM_ENV"))
  (shadow 'StateReward (find-package "GYMNASIUM_ENV")))
(unless (find-package "GYMNASIUM_ENV::STATEREWARD")
  (make-package "GYMNASIUM_ENV::STATEREWARD"))

(in-package "ROS")
;;//! \htmlinclude StateReward.msg.html


(defclass gymnasium_env::StateReward
  :super ros::object
  :slots (_state _info _reward _terminal ))

(defmethod gymnasium_env::StateReward
  (:init
   (&key
    ((:state __state) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:info __info) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:reward __reward) 0.0)
    ((:terminal __terminal) nil)
    )
   (send-super :init)
   (setq _state __state)
   (setq _info __info)
   (setq _reward (float __reward))
   (setq _terminal __terminal)
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:reward
   (&optional __reward)
   (if __reward (setq _reward __reward)) _reward)
  (:terminal
   (&optional (__terminal :null))
   (if (not (eq __terminal :null)) (setq _terminal __terminal)) _terminal)
  (:serialization-length
   ()
   (+
    ;; float32[] _state
    (* 4    (length _state)) 4
    ;; float32[] _info
    (* 4    (length _info)) 4
    ;; float32 _reward
    4
    ;; bool _terminal
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _state
     (write-long (length _state) s)
     (dotimes (i (length _state))
       (sys::poke (elt _state i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _info
     (write-long (length _info) s)
     (dotimes (i (length _info))
       (sys::poke (elt _info i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _reward
       (sys::poke _reward (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _terminal
       (if _terminal (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _state i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _info
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _info (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _info i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _reward
     (setq _reward (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _terminal
     (setq _terminal (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get gymnasium_env::StateReward :md5sum-) "df77cdff2c07bc1682dbb96fddaaf25c")
(setf (get gymnasium_env::StateReward :datatype-) "gymnasium_env/StateReward")
(setf (get gymnasium_env::StateReward :definition-)
      "#Credits: RL ROS package
# Message for returning the current sensation vector 
# (i.e. state or observation or sensor readings) and a
# reward from an  environment

float32[] state
float32[] info
float32 reward
bool terminal
")



(provide :gymnasium_env/StateReward "df77cdff2c07bc1682dbb96fddaaf25c")


