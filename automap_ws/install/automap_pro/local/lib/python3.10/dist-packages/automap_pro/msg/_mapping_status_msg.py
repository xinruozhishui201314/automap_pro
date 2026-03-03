# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:msg/MappingStatusMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MappingStatusMsg(type):
    """Metaclass of message 'MappingStatusMsg'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('automap_pro')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'automap_pro.msg.MappingStatusMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mapping_status_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mapping_status_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mapping_status_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mapping_status_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mapping_status_msg

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MappingStatusMsg(metaclass=Metaclass_MappingStatusMsg):
    """Message class 'MappingStatusMsg'."""

    __slots__ = [
        '_header',
        '_state',
        '_session_id',
        '_keyframe_count',
        '_submap_count',
        '_loop_count',
        '_gps_aligned',
        '_gps_alignment_score',
        '_map_entropy',
        '_total_distance_m',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'state': 'string',
        'session_id': 'uint64',
        'keyframe_count': 'uint32',
        'submap_count': 'uint32',
        'loop_count': 'uint32',
        'gps_aligned': 'boolean',
        'gps_alignment_score': 'float',
        'map_entropy': 'double',
        'total_distance_m': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.state = kwargs.get('state', str())
        self.session_id = kwargs.get('session_id', int())
        self.keyframe_count = kwargs.get('keyframe_count', int())
        self.submap_count = kwargs.get('submap_count', int())
        self.loop_count = kwargs.get('loop_count', int())
        self.gps_aligned = kwargs.get('gps_aligned', bool())
        self.gps_alignment_score = kwargs.get('gps_alignment_score', float())
        self.map_entropy = kwargs.get('map_entropy', float())
        self.total_distance_m = kwargs.get('total_distance_m', float())

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
        if self.header != other.header:
            return False
        if self.state != other.state:
            return False
        if self.session_id != other.session_id:
            return False
        if self.keyframe_count != other.keyframe_count:
            return False
        if self.submap_count != other.submap_count:
            return False
        if self.loop_count != other.loop_count:
            return False
        if self.gps_aligned != other.gps_aligned:
            return False
        if self.gps_alignment_score != other.gps_alignment_score:
            return False
        if self.map_entropy != other.map_entropy:
            return False
        if self.total_distance_m != other.total_distance_m:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'state' field must be of type 'str'"
        self._state = value

    @builtins.property
    def session_id(self):
        """Message field 'session_id'."""
        return self._session_id

    @session_id.setter
    def session_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'session_id' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'session_id' field must be an unsigned integer in [0, 18446744073709551615]"
        self._session_id = value

    @builtins.property
    def keyframe_count(self):
        """Message field 'keyframe_count'."""
        return self._keyframe_count

    @keyframe_count.setter
    def keyframe_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'keyframe_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'keyframe_count' field must be an unsigned integer in [0, 4294967295]"
        self._keyframe_count = value

    @builtins.property
    def submap_count(self):
        """Message field 'submap_count'."""
        return self._submap_count

    @submap_count.setter
    def submap_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'submap_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'submap_count' field must be an unsigned integer in [0, 4294967295]"
        self._submap_count = value

    @builtins.property
    def loop_count(self):
        """Message field 'loop_count'."""
        return self._loop_count

    @loop_count.setter
    def loop_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'loop_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'loop_count' field must be an unsigned integer in [0, 4294967295]"
        self._loop_count = value

    @builtins.property
    def gps_aligned(self):
        """Message field 'gps_aligned'."""
        return self._gps_aligned

    @gps_aligned.setter
    def gps_aligned(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'gps_aligned' field must be of type 'bool'"
        self._gps_aligned = value

    @builtins.property
    def gps_alignment_score(self):
        """Message field 'gps_alignment_score'."""
        return self._gps_alignment_score

    @gps_alignment_score.setter
    def gps_alignment_score(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gps_alignment_score' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'gps_alignment_score' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._gps_alignment_score = value

    @builtins.property
    def map_entropy(self):
        """Message field 'map_entropy'."""
        return self._map_entropy

    @map_entropy.setter
    def map_entropy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'map_entropy' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'map_entropy' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._map_entropy = value

    @builtins.property
    def total_distance_m(self):
        """Message field 'total_distance_m'."""
        return self._total_distance_m

    @total_distance_m.setter
    def total_distance_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total_distance_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'total_distance_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._total_distance_m = value
