# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:msg/LoopConstraintMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'information_matrix'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LoopConstraintMsg(type):
    """Metaclass of message 'LoopConstraintMsg'."""

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
                'automap_pro.msg.LoopConstraintMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__loop_constraint_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__loop_constraint_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__loop_constraint_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__loop_constraint_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__loop_constraint_msg

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


class LoopConstraintMsg(metaclass=Metaclass_LoopConstraintMsg):
    """Message class 'LoopConstraintMsg'."""

    __slots__ = [
        '_header',
        '_submap_i',
        '_submap_j',
        '_session_i',
        '_session_j',
        '_overlap_score',
        '_inlier_ratio',
        '_rmse',
        '_is_inter_session',
        '_information_matrix',
        '_delta_pose',
        '_status',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'submap_i': 'int32',
        'submap_j': 'int32',
        'session_i': 'uint64',
        'session_j': 'uint64',
        'overlap_score': 'float',
        'inlier_ratio': 'float',
        'rmse': 'float',
        'is_inter_session': 'boolean',
        'information_matrix': 'double[36]',
        'delta_pose': 'geometry_msgs/Pose',
        'status': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 36),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.submap_i = kwargs.get('submap_i', int())
        self.submap_j = kwargs.get('submap_j', int())
        self.session_i = kwargs.get('session_i', int())
        self.session_j = kwargs.get('session_j', int())
        self.overlap_score = kwargs.get('overlap_score', float())
        self.inlier_ratio = kwargs.get('inlier_ratio', float())
        self.rmse = kwargs.get('rmse', float())
        self.is_inter_session = kwargs.get('is_inter_session', bool())
        if 'information_matrix' not in kwargs:
            self.information_matrix = numpy.zeros(36, dtype=numpy.float64)
        else:
            self.information_matrix = kwargs.get('information_matrix')
        from geometry_msgs.msg import Pose
        self.delta_pose = kwargs.get('delta_pose', Pose())
        self.status = kwargs.get('status', str())

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
        if self.submap_i != other.submap_i:
            return False
        if self.submap_j != other.submap_j:
            return False
        if self.session_i != other.session_i:
            return False
        if self.session_j != other.session_j:
            return False
        if self.overlap_score != other.overlap_score:
            return False
        if self.inlier_ratio != other.inlier_ratio:
            return False
        if self.rmse != other.rmse:
            return False
        if self.is_inter_session != other.is_inter_session:
            return False
        if any(self.information_matrix != other.information_matrix):
            return False
        if self.delta_pose != other.delta_pose:
            return False
        if self.status != other.status:
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
    def submap_i(self):
        """Message field 'submap_i'."""
        return self._submap_i

    @submap_i.setter
    def submap_i(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'submap_i' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'submap_i' field must be an integer in [-2147483648, 2147483647]"
        self._submap_i = value

    @builtins.property
    def submap_j(self):
        """Message field 'submap_j'."""
        return self._submap_j

    @submap_j.setter
    def submap_j(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'submap_j' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'submap_j' field must be an integer in [-2147483648, 2147483647]"
        self._submap_j = value

    @builtins.property
    def session_i(self):
        """Message field 'session_i'."""
        return self._session_i

    @session_i.setter
    def session_i(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'session_i' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'session_i' field must be an unsigned integer in [0, 18446744073709551615]"
        self._session_i = value

    @builtins.property
    def session_j(self):
        """Message field 'session_j'."""
        return self._session_j

    @session_j.setter
    def session_j(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'session_j' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'session_j' field must be an unsigned integer in [0, 18446744073709551615]"
        self._session_j = value

    @builtins.property
    def overlap_score(self):
        """Message field 'overlap_score'."""
        return self._overlap_score

    @overlap_score.setter
    def overlap_score(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'overlap_score' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'overlap_score' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._overlap_score = value

    @builtins.property
    def inlier_ratio(self):
        """Message field 'inlier_ratio'."""
        return self._inlier_ratio

    @inlier_ratio.setter
    def inlier_ratio(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'inlier_ratio' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'inlier_ratio' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._inlier_ratio = value

    @builtins.property
    def rmse(self):
        """Message field 'rmse'."""
        return self._rmse

    @rmse.setter
    def rmse(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rmse' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rmse' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rmse = value

    @builtins.property
    def is_inter_session(self):
        """Message field 'is_inter_session'."""
        return self._is_inter_session

    @is_inter_session.setter
    def is_inter_session(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_inter_session' field must be of type 'bool'"
        self._is_inter_session = value

    @builtins.property
    def information_matrix(self):
        """Message field 'information_matrix'."""
        return self._information_matrix

    @information_matrix.setter
    def information_matrix(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'information_matrix' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 36, \
                "The 'information_matrix' numpy.ndarray() must have a size of 36"
            self._information_matrix = value
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
                 len(value) == 36 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'information_matrix' field must be a set or sequence with length 36 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._information_matrix = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def delta_pose(self):
        """Message field 'delta_pose'."""
        return self._delta_pose

    @delta_pose.setter
    def delta_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'delta_pose' field must be a sub message of type 'Pose'"
        self._delta_pose = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status' field must be of type 'str'"
        self._status = value
