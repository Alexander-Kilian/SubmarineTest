# A horizontal-only mission, published by waypoint_listener_node on
# jit/nav/local/mission. header.frame_id is the sub's local frame
# (local_origin); every waypoint has already been transformed into it.
#
# No z anywhere, by design - see WaypointXY.msg.

std_msgs/Header header
WaypointXY[] waypoints
