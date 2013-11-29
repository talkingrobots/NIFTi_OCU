# NIFTi GUI Robot Control Widget is published when a user clicks on a widget to control the robot (flippers, brake, reset)

# Note: These constants should be used on the publisher side and on the receiver side

# Constants used in only combination with others
int8 START =			1
int8 STOP =			0

# Constants used in only combination with others
int8 OPEN = 			0
int8 CLOSE =			1

# Signifies that the user started or stopped clicking on the navigation dial
# Always used in combination with START and STOP
int8 WIDGET_MOTION =		30

# Signifies that the user clicked a button for flipper positions
int8 POSITION_DRIVING =		1
int8 POSITION_BANANA =		2
int8 POSITION_FLAT =		3
int8 POSITION_CLIMB_START =	4
int8 POSITION_CLIMB_END =	5

# Signifies that the user clicked on the button to reset the flippers
int8 BTN_RESET_FLIPPERS = 	10

# Signifies that the user used the widget to lock or unlock the differential
# Always used in combination with OPEN and CLOSE
int8 SLIDER_BRAKE = 		20

# Signifies that the user opened or closed the panel for flipper autonomy
# Always used in combination with OPEN and CLOSE
int8 PANEL_FLIPPER_AUTONOMY = 	40

time stamp
string userID
int8 widgetUsed

