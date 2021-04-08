;; Auto-generated. Do not edit!


(when (boundp 'offb_posctl::controlstate)
  (if (not (find-package "OFFB_POSCTL"))
    (make-package "OFFB_POSCTL"))
  (shadow 'controlstate (find-package "OFFB_POSCTL")))
(unless (find-package "OFFB_POSCTL::CONTROLSTATE")
  (make-package "OFFB_POSCTL::CONTROLSTATE"))

(in-package "ROS")
;;//! \htmlinclude controlstate.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass offb_posctl::controlstate
  :super ros::object
  :slots (_header _discrepointpersecond _inicounter _arraylength _wall_z _wall_y _parabolictime _thrustarray _tauarray _phiarray _thetaarray _stateXarray _stateYarray _stateZarray _stateVXarray _stateVYarray _stateVZarray _stateAXarray _stateAYarray _stateAZarray ))

(defmethod offb_posctl::controlstate
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:discrepointpersecond __discrepointpersecond) 0)
    ((:inicounter __inicounter) 0)
    ((:arraylength __arraylength) 0)
    ((:wall_z __wall_z) 0.0)
    ((:wall_y __wall_y) 0.0)
    ((:parabolictime __parabolictime) 0.0)
    ((:thrustarray __thrustarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:tauarray __tauarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:phiarray __phiarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:thetaarray __thetaarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateXarray __stateXarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateYarray __stateYarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateZarray __stateZarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateVXarray __stateVXarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateVYarray __stateVYarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateVZarray __stateVZarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateAXarray __stateAXarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateAYarray __stateAYarray) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:stateAZarray __stateAZarray) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _discrepointpersecond (round __discrepointpersecond))
   (setq _inicounter (round __inicounter))
   (setq _arraylength (round __arraylength))
   (setq _wall_z (float __wall_z))
   (setq _wall_y (float __wall_y))
   (setq _parabolictime (float __parabolictime))
   (setq _thrustarray __thrustarray)
   (setq _tauarray __tauarray)
   (setq _phiarray __phiarray)
   (setq _thetaarray __thetaarray)
   (setq _stateXarray __stateXarray)
   (setq _stateYarray __stateYarray)
   (setq _stateZarray __stateZarray)
   (setq _stateVXarray __stateVXarray)
   (setq _stateVYarray __stateVYarray)
   (setq _stateVZarray __stateVZarray)
   (setq _stateAXarray __stateAXarray)
   (setq _stateAYarray __stateAYarray)
   (setq _stateAZarray __stateAZarray)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:discrepointpersecond
   (&optional __discrepointpersecond)
   (if __discrepointpersecond (setq _discrepointpersecond __discrepointpersecond)) _discrepointpersecond)
  (:inicounter
   (&optional __inicounter)
   (if __inicounter (setq _inicounter __inicounter)) _inicounter)
  (:arraylength
   (&optional __arraylength)
   (if __arraylength (setq _arraylength __arraylength)) _arraylength)
  (:wall_z
   (&optional __wall_z)
   (if __wall_z (setq _wall_z __wall_z)) _wall_z)
  (:wall_y
   (&optional __wall_y)
   (if __wall_y (setq _wall_y __wall_y)) _wall_y)
  (:parabolictime
   (&optional __parabolictime)
   (if __parabolictime (setq _parabolictime __parabolictime)) _parabolictime)
  (:thrustarray
   (&optional __thrustarray)
   (if __thrustarray (setq _thrustarray __thrustarray)) _thrustarray)
  (:tauarray
   (&optional __tauarray)
   (if __tauarray (setq _tauarray __tauarray)) _tauarray)
  (:phiarray
   (&optional __phiarray)
   (if __phiarray (setq _phiarray __phiarray)) _phiarray)
  (:thetaarray
   (&optional __thetaarray)
   (if __thetaarray (setq _thetaarray __thetaarray)) _thetaarray)
  (:stateXarray
   (&optional __stateXarray)
   (if __stateXarray (setq _stateXarray __stateXarray)) _stateXarray)
  (:stateYarray
   (&optional __stateYarray)
   (if __stateYarray (setq _stateYarray __stateYarray)) _stateYarray)
  (:stateZarray
   (&optional __stateZarray)
   (if __stateZarray (setq _stateZarray __stateZarray)) _stateZarray)
  (:stateVXarray
   (&optional __stateVXarray)
   (if __stateVXarray (setq _stateVXarray __stateVXarray)) _stateVXarray)
  (:stateVYarray
   (&optional __stateVYarray)
   (if __stateVYarray (setq _stateVYarray __stateVYarray)) _stateVYarray)
  (:stateVZarray
   (&optional __stateVZarray)
   (if __stateVZarray (setq _stateVZarray __stateVZarray)) _stateVZarray)
  (:stateAXarray
   (&optional __stateAXarray)
   (if __stateAXarray (setq _stateAXarray __stateAXarray)) _stateAXarray)
  (:stateAYarray
   (&optional __stateAYarray)
   (if __stateAYarray (setq _stateAYarray __stateAYarray)) _stateAYarray)
  (:stateAZarray
   (&optional __stateAZarray)
   (if __stateAZarray (setq _stateAZarray __stateAZarray)) _stateAZarray)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _discrepointpersecond
    2
    ;; int16 _inicounter
    2
    ;; int16 _arraylength
    2
    ;; float32 _wall_z
    4
    ;; float32 _wall_y
    4
    ;; float32 _parabolictime
    4
    ;; float32[] _thrustarray
    (* 4    (length _thrustarray)) 4
    ;; float32[] _tauarray
    (* 4    (length _tauarray)) 4
    ;; float32[] _phiarray
    (* 4    (length _phiarray)) 4
    ;; float32[] _thetaarray
    (* 4    (length _thetaarray)) 4
    ;; float32[] _stateXarray
    (* 4    (length _stateXarray)) 4
    ;; float32[] _stateYarray
    (* 4    (length _stateYarray)) 4
    ;; float32[] _stateZarray
    (* 4    (length _stateZarray)) 4
    ;; float32[] _stateVXarray
    (* 4    (length _stateVXarray)) 4
    ;; float32[] _stateVYarray
    (* 4    (length _stateVYarray)) 4
    ;; float32[] _stateVZarray
    (* 4    (length _stateVZarray)) 4
    ;; float32[] _stateAXarray
    (* 4    (length _stateAXarray)) 4
    ;; float32[] _stateAYarray
    (* 4    (length _stateAYarray)) 4
    ;; float32[] _stateAZarray
    (* 4    (length _stateAZarray)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _discrepointpersecond
       (write-word _discrepointpersecond s)
     ;; int16 _inicounter
       (write-word _inicounter s)
     ;; int16 _arraylength
       (write-word _arraylength s)
     ;; float32 _wall_z
       (sys::poke _wall_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _wall_y
       (sys::poke _wall_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _parabolictime
       (sys::poke _parabolictime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _thrustarray
     (write-long (length _thrustarray) s)
     (dotimes (i (length _thrustarray))
       (sys::poke (elt _thrustarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _tauarray
     (write-long (length _tauarray) s)
     (dotimes (i (length _tauarray))
       (sys::poke (elt _tauarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _phiarray
     (write-long (length _phiarray) s)
     (dotimes (i (length _phiarray))
       (sys::poke (elt _phiarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _thetaarray
     (write-long (length _thetaarray) s)
     (dotimes (i (length _thetaarray))
       (sys::poke (elt _thetaarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateXarray
     (write-long (length _stateXarray) s)
     (dotimes (i (length _stateXarray))
       (sys::poke (elt _stateXarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateYarray
     (write-long (length _stateYarray) s)
     (dotimes (i (length _stateYarray))
       (sys::poke (elt _stateYarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateZarray
     (write-long (length _stateZarray) s)
     (dotimes (i (length _stateZarray))
       (sys::poke (elt _stateZarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateVXarray
     (write-long (length _stateVXarray) s)
     (dotimes (i (length _stateVXarray))
       (sys::poke (elt _stateVXarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateVYarray
     (write-long (length _stateVYarray) s)
     (dotimes (i (length _stateVYarray))
       (sys::poke (elt _stateVYarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateVZarray
     (write-long (length _stateVZarray) s)
     (dotimes (i (length _stateVZarray))
       (sys::poke (elt _stateVZarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateAXarray
     (write-long (length _stateAXarray) s)
     (dotimes (i (length _stateAXarray))
       (sys::poke (elt _stateAXarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateAYarray
     (write-long (length _stateAYarray) s)
     (dotimes (i (length _stateAYarray))
       (sys::poke (elt _stateAYarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _stateAZarray
     (write-long (length _stateAZarray) s)
     (dotimes (i (length _stateAZarray))
       (sys::poke (elt _stateAZarray i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _discrepointpersecond
     (setq _discrepointpersecond (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _inicounter
     (setq _inicounter (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _arraylength
     (setq _arraylength (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _wall_z
     (setq _wall_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _wall_y
     (setq _wall_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _parabolictime
     (setq _parabolictime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _thrustarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _thrustarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _thrustarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _tauarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tauarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tauarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _phiarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _phiarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _phiarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _thetaarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _thetaarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _thetaarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateXarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateXarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateXarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateYarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateYarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateYarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateZarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateZarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateZarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateVXarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateVXarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateVXarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateVYarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateVYarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateVYarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateVZarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateVZarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateVZarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateAXarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateAXarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateAXarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateAYarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateAYarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateAYarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _stateAZarray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stateAZarray (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _stateAZarray i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get offb_posctl::controlstate :md5sum-) "fa885756731fe76aac6c7868ca11c7f6")
(setf (get offb_posctl::controlstate :datatype-) "offb_posctl/controlstate")
(setf (get offb_posctl::controlstate :definition-)
      "std_msgs/Header header
int16 discrepointpersecond
int16 inicounter
int16 arraylength
float32 wall_z
float32 wall_y
float32 parabolictime
float32[] thrustarray
float32[] tauarray
float32[] phiarray
float32[] thetaarray
float32[] stateXarray
float32[] stateYarray
float32[] stateZarray
float32[] stateVXarray
float32[] stateVYarray
float32[] stateVZarray
float32[] stateAXarray
float32[] stateAYarray
float32[] stateAZarray
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :offb_posctl/controlstate "fa885756731fe76aac6c7868ca11c7f6")


