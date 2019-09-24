; Auto-generated. Do not edit!


(cl:in-package oars_pkg-msg)


;//! \htmlinclude ObjFrame.msg.html

(cl:defclass <ObjFrame> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (left
    :reader left
    :initarg :left
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (right
    :reader right
    :initarg :right
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (top
    :reader top
    :initarg :top
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (bottom
    :reader bottom
    :initarg :bottom
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32)))
)

(cl:defclass ObjFrame (<ObjFrame>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjFrame>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjFrame)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name oars_pkg-msg:<ObjFrame> is deprecated: use oars_pkg-msg:ObjFrame instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ObjFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader oars_pkg-msg:name-val is deprecated.  Use oars_pkg-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <ObjFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader oars_pkg-msg:left-val is deprecated.  Use oars_pkg-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <ObjFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader oars_pkg-msg:right-val is deprecated.  Use oars_pkg-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'top-val :lambda-list '(m))
(cl:defmethod top-val ((m <ObjFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader oars_pkg-msg:top-val is deprecated.  Use oars_pkg-msg:top instead.")
  (top m))

(cl:ensure-generic-function 'bottom-val :lambda-list '(m))
(cl:defmethod bottom-val ((m <ObjFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader oars_pkg-msg:bottom-val is deprecated.  Use oars_pkg-msg:bottom instead.")
  (bottom m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjFrame>) ostream)
  "Serializes a message object of type '<ObjFrame>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'name) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'top) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bottom) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjFrame>) istream)
  "Deserializes a message object of type '<ObjFrame>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'name) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'top) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bottom) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjFrame>)))
  "Returns string type for a message object of type '<ObjFrame>"
  "oars_pkg/ObjFrame")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjFrame)))
  "Returns string type for a message object of type 'ObjFrame"
  "oars_pkg/ObjFrame")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjFrame>)))
  "Returns md5sum for a message object of type '<ObjFrame>"
  "87318109dd924e4711aff92aa4479132")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjFrame)))
  "Returns md5sum for a message object of type 'ObjFrame"
  "87318109dd924e4711aff92aa4479132")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjFrame>)))
  "Returns full string definition for message of type '<ObjFrame>"
  (cl:format cl:nil "std_msgs/String name~%std_msgs/Float32 left~%std_msgs/Float32 right~%std_msgs/Float32 top~%std_msgs/Float32 bottom~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjFrame)))
  "Returns full string definition for message of type 'ObjFrame"
  (cl:format cl:nil "std_msgs/String name~%std_msgs/Float32 left~%std_msgs/Float32 right~%std_msgs/Float32 top~%std_msgs/Float32 bottom~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjFrame>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'top))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bottom))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjFrame>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjFrame
    (cl:cons ':name (name msg))
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
    (cl:cons ':top (top msg))
    (cl:cons ':bottom (bottom msg))
))
