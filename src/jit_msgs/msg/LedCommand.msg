# Indicator-panel command, published by system_monitor_node on led/command.
#
# The panel's drive mechanism is not yet known, so this carries a pattern name
# rather than anything hardware-specific. led_driver_node is a stub that logs
# the string; expanding the message is expected once the panel is specified.

builtin_interfaces/Time stamp
string pattern

string ESTOP  = ESTOP    # red    - motor rail is dead (relay open)
string MANUAL = MANUAL   # yellow - rail live, manual control
string AUTO   = AUTO     # green  - rail live, a guided mode is active
