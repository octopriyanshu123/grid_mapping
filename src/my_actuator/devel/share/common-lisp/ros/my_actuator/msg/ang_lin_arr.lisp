; Auto-generated. Do not edit!


(cl:in-package my_actuator-msg)


;//! \htmlinclude ang_lin_arr.msg.html

(cl:defclass <ang_lin_arr> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ang_lin_arr (<ang_lin_arr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ang_lin_arr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ang_lin_arr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_actuator-msg:<ang_lin_arr> is deprecated: use my_actuator-msg:ang_lin_arr instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ang_lin_arr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_actuator-msg:data-val is deprecated.  Use my_actuator-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ang_lin_arr>) ostream)
  "Serializes a message object of type '<ang_lin_arr>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ang_lin_arr>) istream)
  "Deserializes a message object of type '<ang_lin_arr>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ang_lin_arr>)))
  "Returns string type for a message object of type '<ang_lin_arr>"
  "my_actuator/ang_lin_arr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ang_lin_arr)))
  "Returns string type for a message object of type 'ang_lin_arr"
  "my_actuator/ang_lin_arr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ang_lin_arr>)))
  "Returns md5sum for a message object of type '<ang_lin_arr>"
  "f75759a84c93e4f482e59ca4ad2da4b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ang_lin_arr)))
  "Returns md5sum for a message object of type 'ang_lin_arr"
  "f75759a84c93e4f482e59ca4ad2da4b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ang_lin_arr>)))
  "Returns full string definition for message of type '<ang_lin_arr>"
  (cl:format cl:nil "float32[4] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ang_lin_arr)))
  "Returns full string definition for message of type 'ang_lin_arr"
  (cl:format cl:nil "float32[4] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ang_lin_arr>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ang_lin_arr>))
  "Converts a ROS message object to a list"
  (cl:list 'ang_lin_arr
    (cl:cons ':data (data msg))
))
