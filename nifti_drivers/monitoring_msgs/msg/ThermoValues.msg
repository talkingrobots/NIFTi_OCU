#ID of the thermal data
string key

#Thermal status 
# - 0 == OK
# - 1 == warm
# - 2 == hot
string status

#Temperature, given in scale described by "unit" (bellow, usually degree Celsius) 
float32 temperature

#Unit used for "temperature", usually degree Celsius
# C == Celsius
# F == Fahrenheit
# K == Kelvin 
string unit