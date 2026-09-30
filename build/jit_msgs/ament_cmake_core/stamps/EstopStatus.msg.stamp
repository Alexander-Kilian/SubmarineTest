# Relay state as reported by gpio_estop_node on estop/status.
#
# Two independent facts, because they can disagree and the disagreement is the
# most safety-relevant signal in the system:
#
#   commanded - what we drove the output pin to
#   feedback  - what the LDO on the switched battery rail says is actually
#               happening (HIGH = relay closed = rail live)
#
# Published with transient-local ("latched") QoS plus a heartbeat, so a late
# subscriber gets the state immediately and a live one can apply a staleness
# timeout.

builtin_interfaces/Time stamp

bool commanded    # true = we asked for the rail to be live
bool feedback     # true = the rail actually is live
bool mismatch     # true = settled and disagreeing (see relay_settle_ms)

# Set once the boot-time self-test has run. Before that, `mismatch` is not
# meaningful because the relay has not had time to settle.
bool settled
