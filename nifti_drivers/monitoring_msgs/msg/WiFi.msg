Header header

#Interface name (e.g. wlan0, ath1)
string  interfaceName

#Used network protocol (e.g. IEEE 802.11abgn)
string  networkProtocol

#SSID of the connected Router/Access Point/Adhoc-partner 
string  essid

#Network operation mode
# -Managed
# -Unmanaged
string  mode

#Used frequency band (e.g. 2.422 GHz, 5.41 GHz)
string  frequency

#MAC-address of the connected Router/Access Point/Adhoc-partner 
string  apMac

#Current bit-Rate used by the network card
#Note that this value is not directly linked to the signal quality
uint32  bitRate

#Current Decibel level of the transmitter
int32  txdBm

#Current power of the transmitter [in Watt]
float32  txPower

#Quality of the network link in percentage [scaled in between 1 and 0]
float32  linkQuality

#Current Decibel level of the received signal  
int32  signalLevel

#Current power of the received signal [in Watt]
float32 signalPower

#Current Decibel  noise level of the received signal  
int32  noiseLevel

#Current noise power of the received signal [in Watt]
float32 noisePower