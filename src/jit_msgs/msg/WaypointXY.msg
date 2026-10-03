# One horizontal waypoint, absolute in the sub's local_origin frame (ENU).
#
# There is deliberately NO z field. This type carries topic-sourced missions
# from waypoint_listener_node to local_guided_node, and a topic-sourced mission
# must never command a dive: the depth is filled in by local_guided_node from
# the depth captured at mode entry. Dives come only from the waypoint file.

float64 x   # metres East in local_origin
float64 y   # metres North in local_origin
