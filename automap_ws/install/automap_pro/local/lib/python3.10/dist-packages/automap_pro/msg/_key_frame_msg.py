# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:msg/KeyFrameMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_KeyFrameMsg(type):
    """Metaclass of message 'KeyFrameMsg'."""

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
                'automap_pro.msg.KeyFrameMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__key_frame_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__key_frame_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__key_frame_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__key_frame_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__key_frame_msg

            from automap_pro.msg import GPSMeasurementMsg
            if GPSMeasurementMsg.__class__._TYPE_SUPPORT is None:
                GPSMeasurementMsg.__class__.__import_type_support__()

            from geometry_msgs.msg import PoseWithCovariance
            if PoseWithCovariance.__class__._TYPE_SUPPORT is None:
                PoseWithCovariance.__class__.__import_type_support__()

            from sensor_msgs.msg import PointCloud2
            if PointCloud2.__class__._TYPE_SUPPORT is None:
                PointCloud2.__class__.__import_type_support__()

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


class KeyFrameMsg(metaclass=Metaclass_KeyFrameMsg):
    """Message class 'KeyFrameMsg'."""

    __slots__ = [
        '_header',
        '_id',
        '_session_id',
        '_submap_id',
        '_pose',
        '_cloud',
        '_has_gps',
        '_gps',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'id': 'uint64',
        'session_id': 'uint64',
        'submap_id': 'int32',
        'pose': 'geometry_msgs/PoseWithCovariance',
        'cloud': 'sensor_msgs/PointCloud2',
        'has_gps': 'boolean',
        'gps': 'automap_pro/GPSMeasurementMsg',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseWithCovariance'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['automap_pro', 'msg'], 'GPSMeasurementMsg'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.id = kwargs.get('id', int())
        self.session_id = kwargs.get('session_id', int())
        self.submap_id = kwargs.get('submap_id', int())
        from geometry_msgs.msg import PoseWithCovariance
        self.pose = kwargs.get('pose', PoseWithCovariance())
        from sensor_msgs.msg import PointCloud2
        self.cloud = kwargs.get('cloud', PointCloud2())
        self.has_gps = kwargs.get('has_gps', bool())
        from automap_pro.msg import GPSMeasurementMsg
        self.gps = kwargs.get('gps', GPSMeasurementMsg())

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
        if self.id != other.id:
            return False
        if self.session_id != other.session_id:
            return False
        if self.submap_id != other.submap_id:
            return False
        if self.pose != other.pose:
            return False
        if self.cloud != other.cloud:
            return False
        if self.has_gps != other.has_gps:
            return False
        if self.gps != other.gps:
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

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'id' field must be an unsigned integer in [0, 18446744073709551615]"
        self._id = value

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
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseWithCovariance
            assert \
                isinstance(value, PoseWithCovariance), \
                "The 'pose' field must be a sub message of type 'PoseWithCovariance'"
        self._pose = value

    @builtins.property
    def cloud(self):
        """Message field 'cloud'."""
        return self._cloud

    @cloud.setter
    def cloud(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'cloud' field must be a sub message of type 'PointCloud2'"
        self._cloud = value

    @builtins.property
    def has_gps(self):
        """Message field 'has_gps'."""
        return self._has_gps

    @has_gps.setter
    def has_gps(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'has_gps' field must be of type 'bool'"
        self._has_gps = value

    @builtins.property
    def gps(self):
        """Message field 'gps'."""
        return self._gps

    @gps.setter
    def gps(self, value):
        if __debug__:
            from automap_pro.msg import GPSMeasurementMsg
            assert \
                isinstance(value, GPSMeasurementMsg), \
                "The 'gps' field must be a sub message of type 'GPSMeasurementMsg'"
        self._gps = value
