Header header

#System wide CPU load in percentage [scaled in between 1 and 0]
float32 load

#Number of physical CPU cores available
int32 cpu_cores

#Number of virtual CPU cores/siblings available
int32 cpu_siblings

#Details of each available core/sibling
CPUValues[] siblings