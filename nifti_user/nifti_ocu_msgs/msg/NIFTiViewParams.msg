# Represents the parameters that indicate what a user is viewing (in the map, or camera)


# Note these names and values are the same as OGRE
int8 PT_ORTHOGRAPHIC =	0	# 2D projection
int8 PT_PERSPECTIVE =	1	# 3D projection

int8 projectionType 		# Projection type (whether the camera looks from above (2D) or is a 3D perspective)
				# Has no meaning when the view is a video feed

# Frame of reference for which the following coordinates apply
#string frameOfReference


geometry_msgs/Pose cameraPose   #Position and orientation of the camera


float32 fieldOfViewHorizontal	# Opening of the "camera lens" in radians
float32 fieldOfViewVertical	# Opening of the "camera lens" in radians


# These represent the size of the view in pixels on the user's screen
int16 viewWidth
int16 viewHeight
