; Auto-generated. Do not edit!


(cl:in-package fetch_controller-msg)


;//! \htmlinclude controller_state.msg.html

(cl:defclass <controller_state> (roslisp-msg-protocol:ros-message)
  ((x_position
    :reader x_position
    :initarg :x_position
    :type cl:float
    :initform 0.0)
   (y_position
    :reader y_position
    :initarg :y_position
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (test_bool
    :reader test_bool
    :initarg :test_bool
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass controller_state (<controller_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controller_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controller_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fetch_controller-msg:<controller_state> is deprecated: use fetch_controller-msg:controller_state instead.")))

(cl:ensure-generic-function 'x_position-val :lambda-list '(m))
(cl:defmethod x_position-val ((m <controller_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fetch_controller-msg:x_position-val is deprecated.  Use fetch_controller-msg:x_position instead.")
  (x_position m))

(cl:ensure-generic-function 'y_position-val :lambda-list '(m))
(cl:defmethod y_position-val ((m <controller_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fetch_controller-msg:y_position-val is deprecated.  Use fetch_controller-msg:y_position instead.")
  (y_position m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <controller_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fetch_controller-msg:angle-val is deprecated.  Use fetch_controller-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'test_bool-val :lambda-list '(m))
(cl:defmethod test_bool-val ((m <controller_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fetch_controller-msg:test_bool-val is deprecated.  Use fetch_controller-msg:test_bool instead.")
  (test_bool m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controller_state>) ostream)
  "Serializes a message object of type '<controller_state>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'test_bool) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controller_state>) istream)
  "Deserializes a message object of type '<controller_state>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'test_bool) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controller_state>)))
  "Returns string type for a message object of type '<controller_state>"
  "fetch_controller/controller_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controller_state)))
  "Returns string type for a message object of type 'controller_state"
  "fetch_controller/controller_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controller_state>)))
  "Returns md5sum for a message object of type '<controller_state>"
  "d57738613aa2d1a1b42d20f2a0694ccd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controller_state)))
  "Returns md5sum for a message object of type 'controller_state"
  "d57738613aa2d1a1b42d20f2a0694ccd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controller_state>)))
  "Returns full string definition for message of type '<controller_state>"
  (cl:format cl:nil "#robot state measurement~%float64 x_position #inches?~%float64 y_position #inches?~%float64 angle #degrees~%~%#status variables~%bool test_bool #We can add more of these~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controller_state)))
  "Returns full string definition for message of type 'controller_state"
  (cl:format cl:nil "#robot state measurement~%float64 x_position #inches?~%float64 y_position #inches?~%float64 angle #degrees~%~%#status variables~%bool test_bool #We can add more of these~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controller_state>))
  (cl:+ 0
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controller_state>))
  "Converts a ROS message object to a list"
  (cl:list 'controller_state
    (cl:cons ':x_position (x_position msg))
    (cl:cons ':y_position (y_position msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':test_bool (test_bool msg))
))
