# NIFTi GUI Status geared towards multiple recipients for various purposes

Header header			# This should contain a timestamp

# Visualization mode

# Note: These numbers match the ones in QuadVizPanel.h, enum MultiVizMode

int8 VIZ_MODE_SINGLE =  	0        # One full-screen view
int8 VIZ_MODE_DUAL_VERTICAL =	1        # Two views, top and bottom
int8 VIZ_MODE_DUAL_HORIZONTAL = 2        # Two views, side-by-side
int8 VIZ_MODE_QUAD = 		3        # Four views

int8 vizMode			# Visualization mode currently used


# Type of view

# Note: These numbers match the ones in NIFTiViewsUtil.h, enum ViewType

int8 VIEW_TYPE_OMNI_CAM = 		0
int8 VIEW_TYPE_VIRTUAL_PTZ_CAM = 	1
int8 VIEW_TYPE_VIRTUAL_PTZ_CAM_OVERLAY= 2
int8 VIEW_TYPE_ARM_CAM =		3
int8 VIEW_TYPE_UAV_CAM_DOWN =		4
int8 VIEW_TYPE_UAV_CAM_FRONT =		5
int8 VIEW_TYPE_PTZ_CAM =		6
int8 VIEW_TYPE_KINECT_CAM =		7
int8 VIEW_TYPE_MAP_2D =			8
int8 VIEW_TYPE_MAP_3D =			9
int8 VIEW_TYPE_CHASE_2D =		10
int8 VIEW_TYPE_CHASE_3D =		11
int8 VIEW_TYPE_FIRST_PERSON =		12
int8 VIEW_TYPE_PHOTO =		        13
int8 VIEW_TYPE_UNINITIALIZED =		14

int8[] views			# This is an array of one to four values, representing the different views currently used

NIFTiViewParams[] viewParams	# This arrays represents the parameters of each view that the GUI has open

# Position of the cursor in a certain view, the view number, and the time of the last movement 
# (if the cursor is not in a view, then contains the last value)
int16 cursor_x
int16 cursor_y
int8 cursor_view
#time cursor_time

# Position of the last single left click in a certain view, the view number, and the time
#int16 last_click_x
#int16 last_click_y
#int8 last_click_view
#time last_click_time

# Note: These are in the same order as VisualizationFrame::initialize, where the tools get added
#int8 TOOL_CAM = 	0
#int8 TOOL_NAV_GOAL =	1
#int8 TOOL_SELECT = 	2
# Note: There could be debugging tools, but I'm pretty sure that everyone can ignore them

#int8 currentTool		# The tool that the user is currently using

#string[] selectedMarkersNames	# The name of the currently selected objects (e.g. car123, debris6)
