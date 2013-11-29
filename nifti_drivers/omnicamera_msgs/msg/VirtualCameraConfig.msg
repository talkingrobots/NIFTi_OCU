# Viewport width, in pixels.
int32 viewportWidth
# Viewport height, in pixels.
int32 viewportHeight
# Horizontal pan (i.e. azimuth) of the camera, in degrees.
# Angular displacement measured from the positive x axis of the /omnicam frame.
# Positive pan corresponds to looking left.
float64 pan
# Vertical tilt (i.e. elevation) of the camera, in degrees.
# Angular displacement measured from the x-y plane of the /omnicam frame.
# Positive tilt corresponds to looking up.
float64 tilt
# Field of view in horizontal direction, fills the viewport in horizontal direction.
float64 horizontalFov
# Field of view in vertical direction, fills the viewport in vertical direction.
float64 verticalFov
# Interval for updating the source image for rendering, in seconds.
float64 imageUpdateInterval
