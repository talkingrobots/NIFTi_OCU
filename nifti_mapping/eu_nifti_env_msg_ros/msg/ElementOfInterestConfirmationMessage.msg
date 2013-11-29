# Benoit 2012-02-28

# Sets a new confidence level for an element of interest
   
Header header

int16 uuid

# These constants are put here as reference only, but they are defined in eu_nifti_env::ElementOfInterest
#int8 STATUS_UNCONFIRMED = 0
#int8 STATUS_CONFIRMED = 1
#int8 STATUS_REJECTED = 2

int8 status

# This extra field is provided as convenience so that CAST can retrieve elements of interest quickly once they are selected. This field should be populated by the CAST system, and read by the GUI. Others can ignore it.
eu_nifti_cast/WorkingMemoryPointer castWorkingMemoryPointer
