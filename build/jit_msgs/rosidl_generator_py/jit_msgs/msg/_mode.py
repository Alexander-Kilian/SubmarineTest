# generated from rosidl_generator_py/resource/_idl.py.em
# with input from jit_msgs:msg/Mode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Mode(type):
    """Metaclass of message 'Mode'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SAFE': 0,
        'MANUAL': 1,
        'LOCAL_GUIDED': 2,
        'GLOBAL_GUIDED': 3,
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
                'jit_msgs.msg.Mode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mode
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mode
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mode
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mode
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mode

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SAFE': cls.__constants['SAFE'],
            'MANUAL': cls.__constants['MANUAL'],
            'LOCAL_GUIDED': cls.__constants['LOCAL_GUIDED'],
            'GLOBAL_GUIDED': cls.__constants['GLOBAL_GUIDED'],
        }

    @property
    def SAFE(self):
        """Message constant 'SAFE'."""
        return Metaclass_Mode.__constants['SAFE']

    @property
    def MANUAL(self):
        """Message constant 'MANUAL'."""
        return Metaclass_Mode.__constants['MANUAL']

    @property
    def LOCAL_GUIDED(self):
        """Message constant 'LOCAL_GUIDED'."""
        return Metaclass_Mode.__constants['LOCAL_GUIDED']

    @property
    def GLOBAL_GUIDED(self):
        """Message constant 'GLOBAL_GUIDED'."""
        return Metaclass_Mode.__constants['GLOBAL_GUIDED']


class Mode(metaclass=Metaclass_Mode):
    """
    Message class 'Mode'.

    Constants:
      SAFE
      MANUAL
      LOCAL_GUIDED
      GLOBAL_GUIDED
    """

    __slots__ = [
        '_stamp',
        '_mode',
    ]

    _fields_and_field_types = {
        'stamp': 'builtin_interfaces/Time',
        'mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())
        self.mode = kwargs.get('mode', int())

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
        if self.mode != other.mode:
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
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value
