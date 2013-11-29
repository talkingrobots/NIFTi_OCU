# Benoit 2011-05-13

# Message that tells the system about an element of interest
# This construction is not ideal. Due to the lack of hierarchy, composition is used, which increases significantly the message size

int8 TYPE_ADD = 0
int8 TYPE_MODIFY = 1
int8 TYPE_REMOVE = 2
    
Header header

int8 type   #See ElementOfInterest.msg for the possibilities


# TODO: Investigate if arrays
eu_nifti_env/LocationOfInterest location
eu_nifti_env/AreaOfInterest area
eu_nifti_env/CarObjectOfInterest car
eu_nifti_env/VictimObjectOfInterest victim
eu_nifti_env/SignObjectOfInterest sign

int8 action

# This extra field is provided as convenience so that CAST can retrieve elements of interest quickly once they are selected. This field should be populated by the CAST system, and read by the GUI. Others can ignore it.
eu_nifti_cast/WorkingMemoryPointer castWorkingMemoryPointer
