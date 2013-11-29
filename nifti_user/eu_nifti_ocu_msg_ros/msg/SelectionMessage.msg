# SelectionMessage: tells when an element is selected

int8 SELECT = 0
int8 DESELECT = 1

Header  header
string  userID
int16   elementUUID	#Probably not necessary
int8    action

# This extra field is provided as convenience so that CAST can retrieve elements of interest quickly. It cannot be guaranteed to be populated, so the element's UUID should be the ultimate look-up key.
eu_nifti_cast/WorkingMemoryPointer castWorkingMemoryPointer
