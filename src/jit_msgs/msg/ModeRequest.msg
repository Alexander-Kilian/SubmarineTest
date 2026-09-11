# A request for an operating mode.
#
# NOTE-2: there is exactly ONE publisher of this message today
# (crsf_channel_node, from the ch8 detent), so system_monitor_node grants
# whatever it is asked for. Before an autonomy layer publishes here as well,
# the monitor needs source priority (RC must outrank autonomy) and request
# expiry (a source that stops publishing loses its claim). The `source` field
# is reserved for that and is currently ignored.

builtin_interfaces/Time stamp
uint8 mode      # values are Mode.msg's constants - Mode::SAFE, Mode::MANUAL,
                # Mode::LOCAL_GUIDED, Mode::GLOBAL_GUIDED. They are deliberately
                # NOT redeclared here: one definition, in Mode.msg.
uint8 source    # reserved - see NOTE-2

uint8 SOURCE_RC       = 0
uint8 SOURCE_AUTONOMY = 1
