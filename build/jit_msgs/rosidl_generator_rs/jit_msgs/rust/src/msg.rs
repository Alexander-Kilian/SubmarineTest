#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to jit_msgs__msg__Health
/// The single ground-truth health verdict for the vehicle, published by
/// system_monitor_node at a fixed rate on jit/health.
///
/// Consumers must apply their own staleness check against `stamp` in addition to
/// reading `ok` - this message being old is itself a fault condition, and the bus
/// is not a substitute for a freshness timeout.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Health {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

    /// True when `faults` is zero. Nothing may arm or command motion when false.
    pub ok: bool,

    /// Bitfield of every currently-asserted fault. Rebuilt from scratch on every
    /// evaluation tick, so a fault clears the moment its cause clears.
    pub faults: u32,

    /// Human-readable summary of the asserted faults, for logs and diagnosis.
    pub detail: std::string::String,

}

impl Health {
    /// ---------------------------------------------------------------------------
    /// Fault bits
    /// ---------------------------------------------------------------------------
    /// crsf/link_ok is false
    pub const RC_LINK_LOST: u32 = 1;

    /// crsf/link_ok has gone stale (that node publishes
    /// every poll cycle, so silence means it died)
    pub const CRSF_NODE_DEAD: u32 = 2;

    /// /mavros/state.connected false, or stale
    pub const MAVROS_LOST: u32 = 4;

    /// estop/status has gone stale
    pub const ESTOP_NODE_DEAD: u32 = 8;

    /// relay feedback says the motor rail is dead
    pub const ESTOP_OPEN: u32 = 16;

    /// commanded closed, feedback says open
    pub const RELAY_NOT_CLOSED: u32 = 32;

    /// commanded open, feedback says CLOSED - the
    /// hardware e-stop is not working
    pub const RELAY_WELDED: u32 = 64;

    /// guided mission finished and held long enough
    pub const MISSION_COMPLETE: u32 = 128;

    /// ---------------------------------------------------------------------------
    /// FAULT_SAFE_MASK - the faults that open the motor relay
    /// ---------------------------------------------------------------------------
    /// system_monitor_node publishes sys/health_ok = ((faults & FAULT_SAFE_MASK) == 0),
    /// and gpio_estop_node requires that to be true before it will close the relay.
    ///
    /// Membership of this mask is NOT a severity judgement. Two rules decide it:
    ///
    ///  1. A fault derived from the relay's own feedback must NEVER be in the mask.
    ///     Otherwise: relay open -> feedback low -> ESTOP_OPEN -> sys/health_ok false
    ///     -> relay commanded open -> feedback stays low. That is a deadlock the
    ///     system cannot leave, and since the vehicle boots with the relay open it
    ///     would never become drivable at all. ESTOP_OPEN, RELAY_NOT_CLOSED and
    ///     RELAY_WELDED are therefore excluded.
    ///
    ///  2. MISSION_COMPLETE is excluded because a successful mission must not cycle
    ///     the hardware e-stop. It is a "mission-safe": disarm only, relay untouched.
    ///
    /// Every excluded fault still sets `ok` false and still forces
    /// vehicle_interface_node to disarm. They just do not gate the relay.
    /// RC_LINK_LOST | CRSF_NODE_DEAD | MAVROS_LOST | ESTOP_NODE_DEAD
    pub const FAULT_SAFE_MASK: u32 = 15;

}


impl Default for Health {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Health::default())
  }
}

impl rosidl_runtime_rs::Message for Health {
  type RmwMsg = super::msg::rmw::Health;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        ok: msg.ok,
        faults: msg.faults,
        detail: msg.detail.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      ok: msg.ok,
      faults: msg.faults,
        detail: msg.detail.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      ok: msg.ok,
      faults: msg.faults,
      detail: msg.detail.to_string(),
    }
  }
}


// Corresponds to jit_msgs__msg__Mode
/// The granted operating mode, published by system_monitor_node on jit/mode.
///
/// This is what the system has GRANTED, not what was requested. The switch (or,
/// later, the autonomy layer) requests a mode via ModeRequest; the monitor grants
/// one after applying the health gate.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Mode {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: u8,

}

impl Mode {
    /// nothing may arm or command motion
    pub const SAFE: u8 = 0;

    /// sticks -> MANUAL_CONTROL
    pub const MANUAL: u8 = 1;

    /// waypoints against /mavros/local_position/pose
    pub const LOCAL_GUIDED: u8 = 2;

    /// waypoints against a GPS fix (not implemented)
    pub const GLOBAL_GUIDED: u8 = 3;

}


impl Default for Mode {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Mode::default())
  }
}

impl rosidl_runtime_rs::Message for Mode {
  type RmwMsg = super::msg::rmw::Mode;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        mode: msg.mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      mode: msg.mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      mode: msg.mode,
    }
  }
}


// Corresponds to jit_msgs__msg__ModeRequest
/// A request for an operating mode.
///
/// NOTE-2: there is exactly ONE publisher of this message today
/// (crsf_channel_node, from the ch8 detent), so system_monitor_node grants
/// whatever it is asked for. Before an autonomy layer publishes here as well,
/// the monitor needs source priority (RC must outrank autonomy) and request
/// expiry (a source that stops publishing loses its claim). The `source` field
/// is reserved for that and is currently ignored.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ModeRequest {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

    /// values are Mode.msg's constants - Mode::SAFE, Mode::MANUAL,
    /// Mode::LOCAL_GUIDED, Mode::GLOBAL_GUIDED. They are deliberately
    /// NOT redeclared here: one definition, in Mode.msg.
    pub mode: u8,

    /// reserved - see NOTE-2
    pub source: u8,

}

impl ModeRequest {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SOURCE_RC: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SOURCE_AUTONOMY: u8 = 1;

}


impl Default for ModeRequest {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ModeRequest::default())
  }
}

impl rosidl_runtime_rs::Message for ModeRequest {
  type RmwMsg = super::msg::rmw::ModeRequest;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        mode: msg.mode,
        source: msg.source,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      mode: msg.mode,
      source: msg.source,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      mode: msg.mode,
      source: msg.source,
    }
  }
}


// Corresponds to jit_msgs__msg__LedCommand
/// Indicator-panel command, published by system_monitor_node on led/command.
///
/// The panel's drive mechanism is not yet known, so this carries a pattern name
/// rather than anything hardware-specific. led_driver_node is a stub that logs
/// the string; expanding the message is expected once the panel is specified.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LedCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pattern: std::string::String,

}

impl LedCommand {
    /// red    - motor rail is dead (relay open)
    pub const ESTOP: &'static str = "ESTOP";

    /// yellow - rail live, manual control
    pub const MANUAL: &'static str = "MANUAL";

    /// green  - rail live, a guided mode is active
    pub const AUTO: &'static str = "AUTO";

}


impl Default for LedCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LedCommand::default())
  }
}

impl rosidl_runtime_rs::Message for LedCommand {
  type RmwMsg = super::msg::rmw::LedCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        pattern: msg.pattern.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        pattern: msg.pattern.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      pattern: msg.pattern.to_string(),
    }
  }
}


// Corresponds to jit_msgs__msg__EstopStatus
/// Relay state as reported by gpio_estop_node on estop/status.
///
/// Two independent facts, because they can disagree and the disagreement is the
/// most safety-relevant signal in the system:
///
///   commanded - what we drove the output pin to
///   feedback  - what the LDO on the switched battery rail says is actually
///               happening (HIGH = relay closed = rail live)
///
/// Published with transient-local ("latched") QoS plus a heartbeat, so a late
/// subscriber gets the state immediately and a live one can apply a staleness
/// timeout.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EstopStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

    /// true = we asked for the rail to be live
    pub commanded: bool,

    /// true = the rail actually is live
    pub feedback: bool,

    /// true = settled and disagreeing (see relay_settle_ms)
    pub mismatch: bool,

    /// Set once the boot-time self-test has run. Before that, `mismatch` is not
    /// meaningful because the relay has not had time to settle.
    pub settled: bool,

}



impl Default for EstopStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EstopStatus::default())
  }
}

impl rosidl_runtime_rs::Message for EstopStatus {
  type RmwMsg = super::msg::rmw::EstopStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        commanded: msg.commanded,
        feedback: msg.feedback,
        mismatch: msg.mismatch,
        settled: msg.settled,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      commanded: msg.commanded,
      feedback: msg.feedback,
      mismatch: msg.mismatch,
      settled: msg.settled,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      commanded: msg.commanded,
      feedback: msg.feedback,
      mismatch: msg.mismatch,
      settled: msg.settled,
    }
  }
}


