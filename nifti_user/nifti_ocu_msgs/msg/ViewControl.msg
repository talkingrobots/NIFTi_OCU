# Published when the user changes views (number, types, content, etc.)

# Note: These constants should be used on the publisher side and on the receiver side

# Modifiers used with MOVE
int8 START =	1
int8 STOP =	0

# Used in combination with START/STOP
int8 MOVE =	10

# These below are punctual events (no START/STOP)
int8 RESET =	20
int8 ZOOM =	30

int8 CHANGE_VIEW_TYPE =	40
int8 CHANGE_VIEW_MODE =	50


time stamp
string userID
int8 action

