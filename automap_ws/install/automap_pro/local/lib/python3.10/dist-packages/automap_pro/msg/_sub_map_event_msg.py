# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:msg/SubMapEventMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SubMapEventMsg(type):
    """Metaclass of message 'SubMapEventMsg'."""

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
                'automap_pro.msg.SubMapEventMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sub_map_event_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sub_map_event_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sub_map_event_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sub_map_event_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sub_map_event_msg

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

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


class SubMapEventMsg(metaclass=Metaclass_SubMapEventMsg):
    """Message class 'SubMapEventMsg'."""

    __slots__ = [
        '_header',
        '_submap_id',
        '_session_id',
        '_event_type',
        '_keyframe_count',
        '_spatial_extent_m',
        '_anchor_pose',
        '_has_valid_gps',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'submap_id': 'int32',
        'session_id': 'uint64',
        'event_type': 'string',
        'keyframe_count': 'int32',
        'spatial_extent_m': 'double',
        'anchor_pose': 'geometry_msgs/Pose',
        'has_valid_gps': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.submap_id = kwargs.get('submap_id', int())
        self.session_id = kwargs.get('session_id', int())
        self.event_type = kwargs.get('event_type', str())
        self.keyframe_count = kwargs.get('keyframe_count', int())
        self.spatial_extent_m = kwargs.get('spatial_extent_m', float())
        from geometry_msgs.msg import Pose
        self.anchor_pose = kwargs.get('anchor_pose', Pose())
        self.has_valid_gps = kwargs.get('has_valid_gps', bool())

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
        if self.submap_id != other.submap_id:
            return False
        if self.session_id != other.session_id:
            return False
        if self.event_type != other.event_type:
            return False
        if self.keyframe_count != other.keyframe_count:
            return False
        if self.spatial_extent_m != other.spatial_extent_m:
            return False
        if self.anchor_pose != other.anchor_pose:
            return False
        if self.has_valid_gps != other.has_valid_gps:
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
    def submap_id(self):
        """Message field 'submap_id'."""
        return self._submap_id

    @submap_id.setter
    def submap_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'submap_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'submap_id' field must be an integer in [-2147483648, 2147483647]"
        self._submap_id = value

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
    def event_type(self):
        """Message field 'event_type'."""
        return self._event_type

    @event_type.setter
    def event_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'event_type' field must be of type 'str'"
        self._event_type = value

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
            assert value >= -2147483648 and value < 2147483648, \
                "The 'keyframe_count' field must be an integer in [-2147483648, 2147483647]"
        self._keyframe_count = value

    @builtins.property
    def spatial_extent_m(self):
        """Message field 'spatial_extent_m'."""
        return self._spatial_extent_m

    @spatial_extent_m.setter
    def spatial_extent_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'spatial_extent_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'spatial_extent_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._spatial_extent_m = value

    @builtins.property
    def anchor_pose(self):
        """Message field 'anchor_pose'."""
        return self._anchor_pose

    @anchor_pose.setter
    def anchor_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'anchor_pose' field must be a sub message of type 'Pose'"
        self._anchor_pose = value

    @builtins.property
    def has_valid_gps(self):
        """Message field 'has_valid_gps'."""
        return self._has_valid_gps

    @has_valid_gps.setter
    def has_valid_gps(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'has_valid_gps' field must be of type 'bool'"
        self._has_valid_gps = value
