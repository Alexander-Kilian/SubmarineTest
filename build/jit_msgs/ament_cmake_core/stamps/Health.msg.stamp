# The single ground-truth health verdict for the vehicle, published by
# system_monitor_node at a fixed rate on jit/health.
#
# Consumers must apply their own staleness check against `stamp` in addition to
# reading `ok` - this message being old is itself a fault condition, and the bus
# is not a substitute for a freshness timeout.

builtin_interfaces/Time stamp

# True when `faults` is zero. Nothing may arm or command motion when false.
bool ok

# Bitfield of every currently-asserted fault. Rebuilt from scratch on every
# evaluation tick, so a fault clears the moment its cause clears.
uint32 faults

# Human-readable summary of the asserted faults, for logs and diagnosis.
string detail

# ---------------------------------------------------------------------------
# Fault bits
# ---------------------------------------------------------------------------
uint32 RC_LINK_LOST     = 1     # crsf/link_ok is false
uint32 CRSF_NODE_DEAD   = 2     # crsf/link_ok has gone stale (that node publishes
                                # every poll cycle, so silence means it died)
uint32 MAVROS_LOST      = 4     # /mavros/state.connected false, or stale
uint32 ESTOP_NODE_DEAD  = 8     # estop/status has gone stale
uint32 ESTOP_OPEN       = 16    # relay feedback says the motor rail is dead
uint32 RELAY_NOT_CLOSED = 32    # commanded closed, feedback says open
uint32 RELAY_WELDED     = 64    # commanded open, feedback says CLOSED - the
                                # hardware e-stop is not working
uint32 MISSION_COMPLETE = 128   # guided mission finished and held long enough

# ---------------------------------------------------------------------------
# FAULT_SAFE_MASK - the faults that open the motor relay
# ---------------------------------------------------------------------------
# system_monitor_node publishes sys/health_ok = ((faults & FAULT_SAFE_MASK) == 0),
# and gpio_estop_node requires that to be true before it will close the relay.
#
# Membership of this mask is NOT a severity judgement. Two rules decide it:
#
#  1. A fault derived from the relay's own feedback must NEVER be in the mask.
#     Otherwise: relay open -> feedback low -> ESTOP_OPEN -> sys/health_ok false
#     -> relay commanded open -> feedback stays low. That is a deadlock the
#     system cannot leave, and since the vehicle boots with the relay open it
#     would never become drivable at all. ESTOP_OPEN, RELAY_NOT_CLOSED and
#     RELAY_WELDED are therefore excluded.
#
#  2. MISSION_COMPLETE is excluded because a successful mission must not cycle
#     the hardware e-stop. It is a "mission-safe": disarm only, relay untouched.
#
# Every excluded fault still sets `ok` false and still forces
# vehicle_interface_node to disarm. They just do not gate the relay.
uint32 FAULT_SAFE_MASK  = 15    # RC_LINK_LOST | CRSF_NODE_DEAD | MAVROS_LOST | ESTOP_NODE_DEAD
