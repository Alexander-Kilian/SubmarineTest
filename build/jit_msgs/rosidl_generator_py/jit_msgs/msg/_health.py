# generated from rosidl_generator_py/resource/_idl.py.em
# with input from jit_msgs:msg/Health.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Health(type):
    """Metaclass of message 'Health'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'RC_LINK_LOST': 1,
        'CRSF_NODE_DEAD': 2,
        'MAVROS_LOST': 4,
        'ESTOP_NODE_DEAD': 8,
        'ESTOP_OPEN': 16,
        'RELAY_NOT_CLOSED': 32,
        'RELAY_WELDED': 64,
        'MISSION_COMPLETE': 128,
        'FAULT_SAFE_MASK': 15,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('jit_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'jit_msgs.msg.Health')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__health
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__health
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__health
            cls._TYPE_SUPPORT = module.type_support_msg__msg__health
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__health

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'RC_LINK_LOST': cls.__constants['RC_LINK_LOST'],
            'CRSF_NODE_DEAD': cls.__constants['CRSF_NODE_DEAD'],
            'MAVROS_LOST': cls.__constants['MAVROS_LOST'],
            'ESTOP_NODE_DEAD': cls.__constants['ESTOP_NODE_DEAD'],
            'ESTOP_OPEN': cls.__constants['ESTOP_OPEN'],
            'RELAY_NOT_CLOSED': cls.__constants['RELAY_NOT_CLOSED'],
            'RELAY_WELDED': cls.__constants['RELAY_WELDED'],
            'MISSION_COMPLETE': cls.__constants['MISSION_COMPLETE'],
            'FAULT_SAFE_MASK': cls.__constants['FAULT_SAFE_MASK'],
        }

    @property
    def RC_LINK_LOST(self):
        """Message constant 'RC_LINK_LOST'."""
        return Metaclass_Health.__constants['RC_LINK_LOST']

    @property
    def CRSF_NODE_DEAD(self):
        """Message constant 'CRSF_NODE_DEAD'."""
        return Metaclass_Health.__constants['CRSF_NODE_DEAD']

    @property
    def MAVROS_LOST(self):
        """Message constant 'MAVROS_LOST'."""
        return Metaclass_Health.__constants['MAVROS_LOST']

    @property
    def ESTOP_NODE_DEAD(self):
        """Message constant 'ESTOP_NODE_DEAD'."""
        return Metaclass_Health.__constants['ESTOP_NODE_DEAD']

    @property
    def ESTOP_OPEN(self):
        """Message constant 'ESTOP_OPEN'."""
        return Metaclass_Health.__constants['ESTOP_OPEN']

    @property
    def RELAY_NOT_CLOSED(self):
        """Message constant 'RELAY_NOT_CLOSED'."""
        return Metaclass_Health.__constants['RELAY_NOT_CLOSED']

    @property
    def RELAY_WELDED(self):
        """Message constant 'RELAY_WELDED'."""
        return Metaclass_Health.__constants['RELAY_WELDED']

    @property
    def MISSION_COMPLETE(self):
        """Message constant 'MISSION_COMPLETE'."""
        return Metaclass_Health.__constants['MISSION_COMPLETE']

    @property
    def FAULT_SAFE_MASK(self):
        """Message constant 'FAULT_SAFE_MASK'."""
        return Metaclass_Health.__constants['FAULT_SAFE_MASK']


class Health(metaclass=Metaclass_Health):
    """
    Message class 'Health'.

    Constants:
      RC_LINK_LOST
      CRSF_NODE_DEAD
      MAVROS_LOST
      ESTOP_NODE_DEAD
      ESTOP_OPEN
      RELAY_NOT_CLOSED
      RELAY_WELDED
      MISSION_COMPLETE
      FAULT_SAFE_MASK
    """

    __slots__ = [
        '_stamp',
        '_ok',
        '_faults',
        '_detail',
    ]

    _fields_and_field_types = {
        'stamp': 'builtin_interfaces/Time',
        'ok': 'boolean',
        'faults': 'uint32',
        'detail': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())
        self.ok = kwargs.get('ok', bool())
        self.faults = kwargs.get('faults', int())
        self.detail = kwargs.get('detail', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.stamp != other.stamp:
            return False
        if self.ok != other.ok:
            return False
        if self.faults != other.faults:
            return False
        if self.detail != other.detail:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value

    @builtins.property
    def ok(self):
        """Message field 'ok'."""
        return self._ok

    @ok.setter
    def ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ok' field must be of type 'bool'"
        self._ok = value

    @builtins.property
    def faults(self):
        """Message field 'faults'."""
        return self._faults

    @faults.setter
    def faults(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'faults' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'faults' field must be an unsigned integer in [0, 4294967295]"
        self._faults = value

    @builtins.property
    def detail(self):
        """Message field 'detail'."""
        return self._detail

    @detail.setter
    def detail(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'detail' field must be of type 'str'"
        self._detail = value
