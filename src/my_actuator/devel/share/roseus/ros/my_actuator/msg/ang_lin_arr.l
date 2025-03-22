;; Auto-generated. Do not edit!


(when (boundp 'my_actuator::ang_lin_arr)
  (if (not (find-package "MY_ACTUATOR"))
    (make-package "MY_ACTUATOR"))
  (shadow 'ang_lin_arr (find-package "MY_ACTUATOR")))
(unless (find-package "MY_ACTUATOR::ANG_LIN_ARR")
  (make-package "MY_ACTUATOR::ANG_LIN_ARR"))

(in-package "ROS")
;;//! \htmlinclude ang_lin_arr.msg.html


(defclass my_actuator::ang_lin_arr
  :super ros::object
  :slots (_data ))

(defmethod my_actuator::ang_lin_arr
  (:init
   (&key
    ((:data __data) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32[4] _data
    (* 4    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[4] _data
     (dotimes (i 4)
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[4] _data
   (dotimes (i (length _data))
     (setf (elt _data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get my_actuator::ang_lin_arr :md5sum-) "f75759a84c93e4f482e59ca4ad2da4b7")
(setf (get my_actuator::ang_lin_arr :datatype-) "my_actuator/ang_lin_arr")
(setf (get my_actuator::ang_lin_arr :definition-)
      "float32[4] data
")



(provide :my_actuator/ang_lin_arr "f75759a84c93e4f482e59ca4ad2da4b7")


