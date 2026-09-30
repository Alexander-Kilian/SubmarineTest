# The granted operating mode, published by system_monitor_node on jit/mode.
#
# This is what the system has GRANTED, not what was requested. The switch (or,
# later, the autonomy layer) requests a mode via ModeRequest; the monitor grants
# one after applying the health gate.

builtin_interfaces/Time stamp
uint8 mode

uint8 SAFE          = 0   # nothing may arm or command motion
uint8 MANUAL        = 1   # sticks -> MANUAL_CONTROL
uint8 LOCAL_GUIDED  = 2   # waypoints against /mavros/local_position/pose
uint8 GLOBAL_GUIDED = 3   # waypoints against a GPS fix (not implemented)
