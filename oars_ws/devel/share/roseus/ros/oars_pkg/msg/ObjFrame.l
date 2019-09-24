;; Auto-generated. Do not edit!


(when (boundp 'oars_pkg::ObjFrame)
  (if (not (find-package "OARS_PKG"))
    (make-package "OARS_PKG"))
  (shadow 'ObjFrame (find-package "OARS_PKG")))
(unless (find-package "OARS_PKG::OBJFRAME")
  (make-package "OARS_PKG::OBJFRAME"))

(in-package "ROS")
;;//! \htmlinclude ObjFrame.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass oars_pkg::ObjFrame
  :super ros::object
  :slots (_name _left _right _top _bottom ))

(defmethod oars_pkg::ObjFrame
  (:init
   (&key
    ((:name __name) (instance std_msgs::String :init))
    ((:left __left) (instance std_msgs::Float32 :init))
    ((:right __right) (instance std_msgs::Float32 :init))
    ((:top __top) (instance std_msgs::Float32 :init))
    ((:bottom __bottom) (instance std_msgs::Float32 :init))
    )
   (send-super :init)
   (setq _name __name)
   (setq _left __left)
   (setq _right __right)
   (setq _top __top)
   (setq _bottom __bottom)
   self)
  (:name
   (&rest __name)
   (if (keywordp (car __name))
       (send* _name __name)
     (progn
       (if __name (setq _name (car __name)))
       _name)))
  (:left
   (&rest __left)
   (if (keywordp (car __left))
       (send* _left __left)
     (progn
       (if __left (setq _left (car __left)))
       _left)))
  (:right
   (&rest __right)
   (if (keywordp (car __right))
       (send* _right __right)
     (progn
       (if __right (setq _right (car __right)))
       _right)))
  (:top
   (&rest __top)
   (if (keywordp (car __top))
       (send* _top __top)
     (progn
       (if __top (setq _top (car __top)))
       _top)))
  (:bottom
   (&rest __bottom)
   (if (keywordp (car __bottom))
       (send* _bottom __bottom)
     (progn
       (if __bottom (setq _bottom (car __bottom)))
       _bottom)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _name
    (send _name :serialization-length)
    ;; std_msgs/Float32 _left
    (send _left :serialization-length)
    ;; std_msgs/Float32 _right
    (send _right :serialization-length)
    ;; std_msgs/Float32 _top
    (send _top :serialization-length)
    ;; std_msgs/Float32 _bottom
    (send _bottom :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _name
       (send _name :serialize s)
     ;; std_msgs/Float32 _left
       (send _left :serialize s)
     ;; std_msgs/Float32 _right
       (send _right :serialize s)
     ;; std_msgs/Float32 _top
       (send _top :serialize s)
     ;; std_msgs/Float32 _bottom
       (send _bottom :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _name
     (send _name :deserialize buf ptr-) (incf ptr- (send _name :serialization-length))
   ;; std_msgs/Float32 _left
     (send _left :deserialize buf ptr-) (incf ptr- (send _left :serialization-length))
   ;; std_msgs/Float32 _right
     (send _right :deserialize buf ptr-) (incf ptr- (send _right :serialization-length))
   ;; std_msgs/Float32 _top
     (send _top :deserialize buf ptr-) (incf ptr- (send _top :serialization-length))
   ;; std_msgs/Float32 _bottom
     (send _bottom :deserialize buf ptr-) (incf ptr- (send _bottom :serialization-length))
   ;;
   self)
  )

(setf (get oars_pkg::ObjFrame :md5sum-) "87318109dd924e4711aff92aa4479132")
(setf (get oars_pkg::ObjFrame :datatype-) "oars_pkg/ObjFrame")
(setf (get oars_pkg::ObjFrame :definition-)
      "std_msgs/String name
std_msgs/Float32 left
std_msgs/Float32 right
std_msgs/Float32 top
std_msgs/Float32 bottom

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :oars_pkg/ObjFrame "87318109dd924e4711aff92aa4479132")


