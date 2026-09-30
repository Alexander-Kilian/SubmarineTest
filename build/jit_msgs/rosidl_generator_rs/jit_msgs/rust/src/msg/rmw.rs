#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "jit_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__Health() -> *const std::ffi::c_void;
}

#[link(name = "jit_msgs__rosidl_generator_c")]
extern "C" {
    fn jit_msgs__msg__Health__init(msg: *mut Health) -> bool;
    fn jit_msgs__msg__Health__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Health>, size: usize) -> bool;
    fn jit_msgs__msg__Health__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Health>);
    fn jit_msgs__msg__Health__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Health>, out_seq: *mut rosidl_runtime_rs::Sequence<Health>) -> bool;
}

// Corresponds to jit_msgs__msg__Health
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// The single ground-truth health verdict for the vehicle, published by
/// system_monitor_node at a fixed rate on jit/health.
///
/// Consumers must apply their own staleness check against `stamp` in addition to
/// reading `ok` - this message being old is itself a fault condition, and the bus
/// is not a substitute for a freshness timeout.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Health {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

    /// True when `faults` is zero. Nothing may arm or command motion when false.
    pub ok: bool,

    /// Bitfield of every currently-asserted fault. Rebuilt from scratch on every
    /// evaluation tick, so a fault clears the moment its cause clears.
    pub faults: u32,

    /// Human-readable summary of the asserted faults, for logs and diagnosis.
    pub detail: rosidl_runtime_rs::String,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jit_msgs__msg__Health__init(&mut msg as *mut _) {
        panic!("Call to jit_msgs__msg__Health__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Health {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__Health__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__Health__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__Health__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Health {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Health where Self: Sized {
  const TYPE_NAME: &'static str = "jit_msgs/msg/Health";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__Health() }
  }
}


#[link(name = "jit_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__Mode() -> *const std::ffi::c_void;
}

#[link(name = "jit_msgs__rosidl_generator_c")]
extern "C" {
    fn jit_msgs__msg__Mode__init(msg: *mut Mode) -> bool;
    fn jit_msgs__msg__Mode__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Mode>, size: usize) -> bool;
    fn jit_msgs__msg__Mode__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Mode>);
    fn jit_msgs__msg__Mode__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Mode>, out_seq: *mut rosidl_runtime_rs::Sequence<Mode>) -> bool;
}

// Corresponds to jit_msgs__msg__Mode
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// The granted operating mode, published by system_monitor_node on jit/mode.
///
/// This is what the system has GRANTED, not what was requested. The switch (or,
/// later, the autonomy layer) requests a mode via ModeRequest; the monitor grants
/// one after applying the health gate.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Mode {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jit_msgs__msg__Mode__init(&mut msg as *mut _) {
        panic!("Call to jit_msgs__msg__Mode__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Mode {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__Mode__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__Mode__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__Mode__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Mode {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Mode where Self: Sized {
  const TYPE_NAME: &'static str = "jit_msgs/msg/Mode";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__Mode() }
  }
}


#[link(name = "jit_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__ModeRequest() -> *const std::ffi::c_void;
}

#[link(name = "jit_msgs__rosidl_generator_c")]
extern "C" {
    fn jit_msgs__msg__ModeRequest__init(msg: *mut ModeRequest) -> bool;
    fn jit_msgs__msg__ModeRequest__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ModeRequest>, size: usize) -> bool;
    fn jit_msgs__msg__ModeRequest__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ModeRequest>);
    fn jit_msgs__msg__ModeRequest__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ModeRequest>, out_seq: *mut rosidl_runtime_rs::Sequence<ModeRequest>) -> bool;
}

// Corresponds to jit_msgs__msg__ModeRequest
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// A request for an operating mode.
///
/// NOTE-2: there is exactly ONE publisher of this message today
/// (crsf_channel_node, from the ch8 detent), so system_monitor_node grants
/// whatever it is asked for. Before an autonomy layer publishes here as well,
/// the monitor needs source priority (RC must outrank autonomy) and request
/// expiry (a source that stops publishing loses its claim). The `source` field
/// is reserved for that and is currently ignored.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ModeRequest {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jit_msgs__msg__ModeRequest__init(&mut msg as *mut _) {
        panic!("Call to jit_msgs__msg__ModeRequest__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ModeRequest {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__ModeRequest__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__ModeRequest__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__ModeRequest__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ModeRequest {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ModeRequest where Self: Sized {
  const TYPE_NAME: &'static str = "jit_msgs/msg/ModeRequest";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__ModeRequest() }
  }
}


#[link(name = "jit_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__LedCommand() -> *const std::ffi::c_void;
}

#[link(name = "jit_msgs__rosidl_generator_c")]
extern "C" {
    fn jit_msgs__msg__LedCommand__init(msg: *mut LedCommand) -> bool;
    fn jit_msgs__msg__LedCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LedCommand>, size: usize) -> bool;
    fn jit_msgs__msg__LedCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LedCommand>);
    fn jit_msgs__msg__LedCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LedCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<LedCommand>) -> bool;
}

// Corresponds to jit_msgs__msg__LedCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Indicator-panel command, published by system_monitor_node on led/command.
///
/// The panel's drive mechanism is not yet known, so this carries a pattern name
/// rather than anything hardware-specific. led_driver_node is a stub that logs
/// the string; expanding the message is expected once the panel is specified.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LedCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pattern: rosidl_runtime_rs::String,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jit_msgs__msg__LedCommand__init(&mut msg as *mut _) {
        panic!("Call to jit_msgs__msg__LedCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LedCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__LedCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__LedCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__LedCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LedCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LedCommand where Self: Sized {
  const TYPE_NAME: &'static str = "jit_msgs/msg/LedCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__LedCommand() }
  }
}


#[link(name = "jit_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__EstopStatus() -> *const std::ffi::c_void;
}

#[link(name = "jit_msgs__rosidl_generator_c")]
extern "C" {
    fn jit_msgs__msg__EstopStatus__init(msg: *mut EstopStatus) -> bool;
    fn jit_msgs__msg__EstopStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EstopStatus>, size: usize) -> bool;
    fn jit_msgs__msg__EstopStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EstopStatus>);
    fn jit_msgs__msg__EstopStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EstopStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<EstopStatus>) -> bool;
}

// Corresponds to jit_msgs__msg__EstopStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

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

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EstopStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !jit_msgs__msg__EstopStatus__init(&mut msg as *mut _) {
        panic!("Call to jit_msgs__msg__EstopStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EstopStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__EstopStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__EstopStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { jit_msgs__msg__EstopStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EstopStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EstopStatus where Self: Sized {
  const TYPE_NAME: &'static str = "jit_msgs/msg/EstopStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__jit_msgs__msg__EstopStatus() }
  }
}


