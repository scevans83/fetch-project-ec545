;; Auto-generated. Do not edit!


(when (boundp 'fetch_controller::controller_state)
  (if (not (find-package "FETCH_CONTROLLER"))
    (make-package "FETCH_CONTROLLER"))
  (shadow 'controller_state (find-package "FETCH_CONTROLLER")))
(unless (find-package "FETCH_CONTROLLER::CONTROLLER_STATE")
  (make-package "FETCH_CONTROLLER::CONTROLLER_STATE"))

(in-package "ROS")
;;//! \htmlinclude controller_state.msg.html


(defclass fetch_controller::controller_state
  :super ros::object
  :slots (_x_position _y_position _angle _test_bool ))

(defmethod fetch_controller::controller_state
  (:init
   (&key
    ((:x_position __x_position) 0.0)
    ((:y_position __y_position) 0.0)
    ((:angle __angle) 0.0)
    ((:test_bool __test_bool) nil)
    )
   (send-super :init)
   (setq _x_position (float __x_position))
   (setq _y_position (float __y_position))
   (setq _angle (float __angle))
   (setq _test_bool __test_bool)
   self)
  (:x_position
   (&optional __x_position)
   (if __x_position (setq _x_position __x_position)) _x_position)
  (:y_position
   (&optional __y_position)
   (if __y_position (setq _y_position __y_position)) _y_position)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:test_bool
   (&optional (__test_bool :null))
   (if (not (eq __test_bool :null)) (setq _test_bool __test_bool)) _test_bool)
  (:serialization-length
   ()
   (+
    ;; float64 _x_position
    8
    ;; float64 _y_position
    8
    ;; float64 _angle
    8
    ;; bool _test_bool
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_position
       (sys::poke _x_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_position
       (sys::poke _y_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _test_bool
       (if _test_bool (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_position
     (setq _x_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_position
     (setq _y_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _test_bool
     (setq _test_bool (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fetch_controller::controller_state :md5sum-) "d57738613aa2d1a1b42d20f2a0694ccd")
(setf (get fetch_controller::controller_state :datatype-) "fetch_controller/controller_state")
(setf (get fetch_controller::controller_state :definition-)
      "#robot state measurement
float64 x_position #inches?
float64 y_position #inches?
float64 angle #degrees

#status variables
bool test_bool #We can add more of these
")



(provide :fetch_controller/controller_state "d57738613aa2d1a1b42d20f2a0694ccd")


