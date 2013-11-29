Header header

#System wide status of the battery:
# -Full = battery is fully charged
# -Charging = battery is charging/ AC is attached
# -Discharging = battery is discharging/AC is not attached
# -Unknown = battery is nearly charged but the final state of Full is not reached jet (only given for a few types of batteries and OS)
string 	status

#System wide percentage of the current battery level [scaled in between 1 and 0]
float32 currentBatteryLevel

#Estimated time for discharging/charging the battery [in sec]
#Note that the estimation is done by the OS and quality depends on the age of the battery and so on.
int32 batteryTimeLeft