# Represents the most abstract class for elements in the environment.
# Sub-classes are LocationOfInterest, AreaOfInterest, and ObjectOfInterest

int8 TYPE_ELEMENT =  	0	# The object is of type element, but we don't know more
int8 TYPE_LOCATION =  	1	# The object is a location
int8 TYPE_AREA =	2	# The object is an area
int8 TYPE_OBJECT = 	3	# The object is an actual real-life object, but we don't know more
int8 TYPE_CAR = 	4	# The object is a car
int8 TYPE_SIGN = 	5	# The object is a sign
int8 TYPE_VICTIM = 	6	# The object is a victim

int8 SOURCE_TYPE_VISION = 0	# Vision detected this object
int8 SOURCE_TYPE_USER =	  1	# The user detected this object

int8 STATUS_UNCONFIRMED = 0 # User hasn't yet confirmed/rejected this object 
int8 STATUS_CONFIRMED = 1	# User has confirmed the object
int8 STATUS_REJECTED = 2    # User has rejected the object

int16     uuid
string    name
int8      type
int8      sourceType
int8 	  status
float64   confidence # Confidence of detection (Range 0 -> 1)
