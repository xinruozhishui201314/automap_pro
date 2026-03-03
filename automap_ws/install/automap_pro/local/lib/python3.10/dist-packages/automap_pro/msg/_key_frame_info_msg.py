# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:msg/KeyFrameInfoMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'gyro_bias'
# Member 'accel_bias'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_KeyFrameInfoMsg(type):
    """Metaclass of message 'KeyFrameInfoMsg'."""

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
                'automap_pro.msg.KeyFrameInfoMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__key_frame_info_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__key_frame_info_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__key_frame_info_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__key_frame_info_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__key_frame_info_msg

            from geometry_msgs.msg import PoseWithCovariance
            if PoseWithCovariance.__class__._TYPE_SUPPORT is None:
                PoseWithCovariance.__class__.__import_type_support__()

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


class KeyFrameInfoMsg(metaclass=Metaclass_KeyFrameInfoMsg):
    """Message class 'KeyFrameInfoMsg'."""

    __slots__ = [
        '_header',
        '_keyframe_id',
        '_timestamp',
        '_esikf_covariance_norm',
        '_is_degenerate',
        '_map_update_rate',
        '_gyro_bias',
        '_accel_bias',
        '_cloud_point_count',
        '_cloud_valid',
        '_pose',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'keyframe_id': 'uint64',
        'timestamp': 'double',
        'esikf_covariance_norm': 'double',
        'is_degenerate': 'boolean',
        'map_update_rate': 'float',
        'gyro_bias': 'double[3]',
        'accel_bias': 'double[3]',
        'cloud_point_count': 'uint32',
        'cloud_valid': 'boolean',
        'pose': 'geometry_msgs/PoseWithCovariance',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseWithCovariance'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.keyframe_id = kwargs.get('keyframe_id', int())
        self.timestamp = kwargs.get('timestamp', float())
        self.esikf_covariance_norm = kwargs.get('esikf_covariance_norm', float())
        self.is_degenerate = kwargs.get('is_degenerate', bool())
        self.map_update_rate = kwargs.get('map_update_rate', float())
        if 'gyro_bias' not in kwargs:
            self.gyro_bias = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.gyro_bias = kwargs.get('gyro_bias')
        if 'accel_bias' not in kwargs:
            self.accel_bias = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.accel_bias = kwargs.get('accel_bias')
        self.cloud_point_count = kwargs.get('cloud_point_count', int())
        self.cloud_valid = kwargs.get('cloud_valid', bool())
        from geometry_msgs.msg import PoseWithCovariance
        self.pose = kwargs.get('pose', PoseWithCovariance())

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
        if self.keyframe_id != other.keyframe_id:
            return False
        if self.timestamp != other.timestamp:
            return False
        if self.esikf_covariance_norm != other.esikf_covariance_norm:
            return False
        if self.is_degenerate != other.is_degenerate:
            return False
        if self.map_update_rate != other.map_update_rate:
            return False
        if any(self.gyro_bias != other.gyro_bias):
            return False
        if any(self.accel_bias != other.accel_bias):
            return False
        if self.cloud_point_count != other.cloud_point_count:
            return False
        if self.cloud_valid != other.cloud_valid:
            return False
        if self.pose != other.pose:
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
    def keyframe_id(self):
        """Message field 'keyframe_id'."""
        return self._keyframe_id

    @keyframe_id.setter
    def keyframe_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'keyframe_id' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'keyframe_id' field must be an unsigned integer in [0, 18446744073709551615]"
        self._keyframe_id = value

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'timestamp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'timestamp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._timestamp = value

    @builtins.property
    def esikf_covariance_norm(self):
        """Message field 'esikf_covariance_norm'."""
        return self._esikf_covariance_norm

    @esikf_covariance_norm.setter
    def esikf_covariance_norm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'esikf_covariance_norm' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'esikf_covariance_norm' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._esikf_covariance_norm = value

    @builtins.property
    def is_degenerate(self):
        """Message field 'is_degenerate'."""
        return self._is_degenerate

    @is_degenerate.setter
    def is_degenerate(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_degenerate' field must be of type 'bool'"
        self._is_degenerate = value

    @builtins.property
    def map_update_rate(self):
        """Message field 'map_update_rate'."""
        return self._map_update_rate

    @map_update_rate.setter
    def map_update_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'map_update_rate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'map_update_rate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._map_update_rate = value

    @builtins.property
    def gyro_bias(self):
        """Message field 'gyro_bias'."""
        return self._gyro_bias

    @gyro_bias.setter
    def gyro_bias(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'gyro_bias' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'gyro_bias' numpy.ndarray() must have a size of 3"
            self._gyro_bias = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'gyro_bias' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._gyro_bias = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def accel_bias(self):
        """Message field 'accel_bias'."""
        return self._accel_bias

    @accel_bias.setter
    def accel_bias(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'accel_bias' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'accel_bias' numpy.ndarray() must have a size of 3"
            self._accel_bias = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'accel_bias' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._accel_bias = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def cloud_point_count(self):
        """Message field 'cloud_point_count'."""
        return self._cloud_point_count

    @cloud_point_count.setter
    def cloud_point_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cloud_point_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'cloud_point_count' field must be an unsigned integer in [0, 4294967295]"
        self._cloud_point_count = value

    @builtins.property
    def cloud_valid(self):
        """Message field 'cloud_valid'."""
        return self._cloud_valid

    @cloud_valid.setter
    def cloud_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cloud_valid' field must be of type 'bool'"
        self._cloud_valid = value

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
