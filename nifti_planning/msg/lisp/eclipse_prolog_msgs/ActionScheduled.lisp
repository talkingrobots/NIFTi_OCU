; Auto-generated. Do not edit!


(in-package eclipse_prolog_msgs-msg)


;//! \htmlinclude ActionScheduled.msg.html

(defclass <ActionScheduled> (ros-message)
  ((task_name
    :reader task_name-val
    :initarg :task_name
    :type string
    :initform "")
   (id
    :reader id-val
    :initarg :id
    :type integer
    :initform 0)
   (action
    :reader action-val
    :initarg :action
    :type string
    :initform "")
   (status
    :reader status-val
    :initarg :status
    :type string
    :initform ""))
)
(defmethod serialize ((msg <ActionScheduled>) ostream)
  "Serializes a message object of type '<ActionScheduled>"
  (let ((__ros_str_len (length (slot-value msg 'task_name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'task_name))
    (write-byte (ldb (byte 8 0) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'id)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'id)) ostream)
  (let ((__ros_str_len (length (slot-value msg 'action))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'action))
  (let ((__ros_str_len (length (slot-value msg 'status))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'status))
)
(defmethod deserialize ((msg <ActionScheduled>) istream)
  "Deserializes a message object of type '<ActionScheduled>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'task_name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'task_name) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'id)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'id)) (read-byte istream))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'action) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'action) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'status) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'status) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<ActionScheduled>)))
  "Returns string type for a message object of type '<ActionScheduled>"
  "eclipse_prolog_msgs/ActionScheduled")
(defmethod md5sum ((type (eql '<ActionScheduled>)))
  "Returns md5sum for a message object of type '<ActionScheduled>"
  "36dbf383a366755ed2ee19f9834f4b14")
(defmethod message-definition ((type (eql '<ActionScheduled>)))
  "Returns full string definition for message of type '<ActionScheduled>"
  (format nil "string task_name~%int32 id~%string action~%string status~%~%~%~%"))
(defmethod serialization-length ((msg <ActionScheduled>))
  (+ 0
     4 (length (slot-value msg 'task_name))
     4
     4 (length (slot-value msg 'action))
     4 (length (slot-value msg 'status))
))
(defmethod ros-message-to-list ((msg <ActionScheduled>))
  "Converts a ROS message object to a list"
  (list '<ActionScheduled>
    (cons ':task_name (task_name-val msg))
    (cons ':id (id-val msg))
    (cons ':action (action-val msg))
    (cons ':status (status-val msg))
))
