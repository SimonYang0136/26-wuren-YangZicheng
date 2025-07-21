; Auto-generated. Do not edit!


(cl:in-package turtle_control_pkg-msg)


;//! \htmlinclude TurtleVelocity.msg.html

(cl:defclass <TurtleVelocity> (roslisp-msg-protocol:ros-message)
  ((linear_x
    :reader linear_x
    :initarg :linear_x
    :type cl:float
    :initform 0.0))
)

(cl:defclass TurtleVelocity (<TurtleVelocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TurtleVelocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TurtleVelocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name turtle_control_pkg-msg:<TurtleVelocity> is deprecated: use turtle_control_pkg-msg:TurtleVelocity instead.")))

(cl:ensure-generic-function 'linear_x-val :lambda-list '(m))
(cl:defmethod linear_x-val ((m <TurtleVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtle_control_pkg-msg:linear_x-val is deprecated.  Use turtle_control_pkg-msg:linear_x instead.")
  (linear_x m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TurtleVelocity>) ostream)
  "Serializes a message object of type '<TurtleVelocity>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'linear_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TurtleVelocity>) istream)
  "Deserializes a message object of type '<TurtleVelocity>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_x) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TurtleVelocity>)))
  "Returns string type for a message object of type '<TurtleVelocity>"
  "turtle_control_pkg/TurtleVelocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TurtleVelocity)))
  "Returns string type for a message object of type 'TurtleVelocity"
  "turtle_control_pkg/TurtleVelocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TurtleVelocity>)))
  "Returns md5sum for a message object of type '<TurtleVelocity>"
  "d25d5302321f823414be97fb10f18cdd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TurtleVelocity)))
  "Returns md5sum for a message object of type 'TurtleVelocity"
  "d25d5302321f823414be97fb10f18cdd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TurtleVelocity>)))
  "Returns full string definition for message of type '<TurtleVelocity>"
  (cl:format cl:nil "float64 linear_x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TurtleVelocity)))
  "Returns full string definition for message of type 'TurtleVelocity"
  (cl:format cl:nil "float64 linear_x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TurtleVelocity>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TurtleVelocity>))
  "Converts a ROS message object to a list"
  (cl:list 'TurtleVelocity
    (cl:cons ':linear_x (linear_x msg))
))
