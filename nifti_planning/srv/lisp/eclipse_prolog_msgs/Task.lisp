; Auto-generated. Do not edit!


(in-package eclipse_prolog_msgs-srv)


;//! \htmlinclude Task-request.msg.html

(defclass <Task-request> (ros-message)
  ((id
    :reader id-val
    :initarg :id
    :type integer
    :initform 0)
   (task_name
    :reader task_name-val
    :initarg :task_name
    :type string
    :initform ""))
)
(defmethod serialize ((msg <Task-request>) ostream)
  "Serializes a message object of type '<Task-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'id)) ostream)
  (let ((__ros_str_len (length (slot-value msg 'task_name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'task_name))
)
(defmethod deserialize ((msg <Task-request>) istream)
  "Deserializes a message object of type '<Task-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'id)) (read-byte istream))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'task_name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'task_name) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Task-request>)))
  "Returns string type for a service object of type '<Task-request>"
  "eclipse_prolog_msgs/TaskRequest")
(defmethod md5sum ((type (eql '<Task-request>)))
  "Returns md5sum for a message object of type '<Task-request>"
  "ce2d5654f0ad4f49ee44a2765058e811")
(defmethod message-definition ((type (eql '<Task-request>)))
  "Returns full string definition for message of type '<Task-request>"
  (format nil "int32 id~%string task_name~%~%"))
(defmethod serialization-length ((msg <Task-request>))
  (+ 0
     4
     4 (length (slot-value msg 'task_name))
))
(defmethod ros-message-to-list ((msg <Task-request>))
  "Converts a ROS message object to a list"
  (list '<Task-request>
    (cons ':id (id-val msg))
    (cons ':task_name (task_name-val msg))
))
;//! \htmlinclude Task-response.msg.html

(defclass <Task-response> (ros-message)
  ((feedback
    :reader feedback-val
    :initarg :feedback
    :type string
    :initform ""))
)
(defmethod serialize ((msg <Task-response>) ostream)
  "Serializes a message object of type '<Task-response>"
  (let ((__ros_str_len (length (slot-value msg 'feedback))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'feedback))
)
(defmethod deserialize ((msg <Task-response>) istream)
  "Deserializes a message object of type '<Task-response>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'feedback) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'feedback) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Task-response>)))
  "Returns string type for a service object of type '<Task-response>"
  "eclipse_prolog_msgs/TaskResponse")
(defmethod md5sum ((type (eql '<Task-response>)))
  "Returns md5sum for a message object of type '<Task-response>"
  "ce2d5654f0ad4f49ee44a2765058e811")
(defmethod message-definition ((type (eql '<Task-response>)))
  "Returns full string definition for message of type '<Task-response>"
  (format nil "string feedback~%~%~%~%"))
(defmethod serialization-length ((msg <Task-response>))
  (+ 0
     4 (length (slot-value msg 'feedback))
))
(defmethod ros-message-to-list ((msg <Task-response>))
  "Converts a ROS message object to a list"
  (list '<Task-response>
    (cons ':feedback (feedback-val msg))
))
(defmethod service-request-type ((msg (eql 'Task)))
  '<Task-request>)
(defmethod service-response-type ((msg (eql 'Task)))
  '<Task-response>)
(defmethod ros-datatype ((msg (eql 'Task)))
  "Returns string type for a service object of type '<Task>"
  "eclipse_prolog_msgs/Task")
