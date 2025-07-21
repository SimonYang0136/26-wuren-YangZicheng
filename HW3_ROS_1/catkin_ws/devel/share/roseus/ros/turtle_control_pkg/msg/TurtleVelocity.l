;; Auto-generated. Do not edit!


(when (boundp 'turtle_control_pkg::TurtleVelocity)
  (if (not (find-package "TURTLE_CONTROL_PKG"))
    (make-package "TURTLE_CONTROL_PKG"))
  (shadow 'TurtleVelocity (find-package "TURTLE_CONTROL_PKG")))
(unless (find-package "TURTLE_CONTROL_PKG::TURTLEVELOCITY")
  (make-package "TURTLE_CONTROL_PKG::TURTLEVELOCITY"))

(in-package "ROS")
;;//! \htmlinclude TurtleVelocity.msg.html


(defclass turtle_control_pkg::TurtleVelocity
  :super ros::object
  :slots (_linear_x ))

(defmethod turtle_control_pkg::TurtleVelocity
  (:init
   (&key
    ((:linear_x __linear_x) 0.0)
    )
   (send-super :init)
   (setq _linear_x (float __linear_x))
   self)
  (:linear_x
   (&optional __linear_x)
   (if __linear_x (setq _linear_x __linear_x)) _linear_x)
  (:serialization-length
   ()
   (+
    ;; float64 _linear_x
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _linear_x
       (sys::poke _linear_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _linear_x
     (setq _linear_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get turtle_control_pkg::TurtleVelocity :md5sum-) "d25d5302321f823414be97fb10f18cdd")
(setf (get turtle_control_pkg::TurtleVelocity :datatype-) "turtle_control_pkg/TurtleVelocity")
(setf (get turtle_control_pkg::TurtleVelocity :definition-)
      "float64 linear_x

")



(provide :turtle_control_pkg/TurtleVelocity "d25d5302321f823414be97fb10f18cdd")


