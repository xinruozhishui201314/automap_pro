# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:srv/TriggerGpsAlign.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TriggerGpsAlign_Request(type):
    """Metaclass of message 'TriggerGpsAlign_Request'."""

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
                'automap_pro.srv.TriggerGpsAlign_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__trigger_gps_align__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__trigger_gps_align__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__trigger_gps_align__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__trigger_gps_align__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__trigger_gps_align__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TriggerGpsAlign_Request(metaclass=Metaclass_TriggerGpsAlign_Request):
    """Message class 'TriggerGpsAlign_Request'."""

    __slots__ = [
        '_force',
    ]

    _fields_and_field_types = {
        'force': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.force = kwargs.get('force', bool())

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
        if self.force != other.force:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def force(self):
        """Message field 'force'."""
        return self._force

    @force.setter
    def force(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'force' field must be of type 'bool'"
        self._force = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# Member 'r_gps_lidar'
# Member 't_gps_lidar'
import numpy  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_TriggerGpsAlign_Response(type):
    """Metaclass of message 'TriggerGpsAlign_Response'."""

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
                'automap_pro.srv.TriggerGpsAlign_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__trigger_gps_align__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__trigger_gps_align__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__trigger_gps_align__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__trigger_gps_align__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__trigger_gps_align__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TriggerGpsAlign_Response(metaclass=Metaclass_TriggerGpsAlign_Response):
    """Message class 'TriggerGpsAlign_Response'."""

    __slots__ = [
        '_success',
        '_alignment_rmse_m',
        '_r_gps_lidar',
        '_t_gps_lidar',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'alignment_rmse_m': 'double',
        'r_gps_lidar': 'double[9]',
        't_gps_lidar': 'double[3]',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.alignment_rmse_m = kwargs.get('alignment_rmse_m', float())
        if 'r_gps_lidar' not in kwargs:
            self.r_gps_lidar = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.r_gps_lidar = kwargs.get('r_gps_lidar')
        if 't_gps_lidar' not in kwargs:
            self.t_gps_lidar = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.t_gps_lidar = kwargs.get('t_gps_lidar')
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.alignment_rmse_m != other.alignment_rmse_m:
            return False
        if any(self.r_gps_lidar != other.r_gps_lidar):
            return False
        if any(self.t_gps_lidar != other.t_gps_lidar):
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def alignment_rmse_m(self):
        """Message field 'alignment_rmse_m'."""
        return self._alignment_rmse_m

    @alignment_rmse_m.setter
    def alignment_rmse_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'alignment_rmse_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'alignment_rmse_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._alignment_rmse_m = value

    @builtins.property
    def r_gps_lidar(self):
        """Message field 'r_gps_lidar'."""
        return self._r_gps_lidar

    @r_gps_lidar.setter
    def r_gps_lidar(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'r_gps_lidar' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'r_gps_lidar' numpy.ndarray() must have a size of 9"
            self._r_gps_lidar = value
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
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'r_gps_lidar' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._r_gps_lidar = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def t_gps_lidar(self):
        """Message field 't_gps_lidar'."""
        return self._t_gps_lidar

    @t_gps_lidar.setter
    def t_gps_lidar(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 't_gps_lidar' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 't_gps_lidar' numpy.ndarray() must have a size of 3"
            self._t_gps_lidar = value
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
                "The 't_gps_lidar' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._t_gps_lidar = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_TriggerGpsAlign(type):
    """Metaclass of service 'TriggerGpsAlign'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('automap_pro')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'automap_pro.srv.TriggerGpsAlign')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__trigger_gps_align

            from automap_pro.srv import _trigger_gps_align
            if _trigger_gps_align.Metaclass_TriggerGpsAlign_Request._TYPE_SUPPORT is None:
                _trigger_gps_align.Metaclass_TriggerGpsAlign_Request.__import_type_support__()
            if _trigger_gps_align.Metaclass_TriggerGpsAlign_Response._TYPE_SUPPORT is None:
                _trigger_gps_align.Metaclass_TriggerGpsAlign_Response.__import_type_support__()


class TriggerGpsAlign(metaclass=Metaclass_TriggerGpsAlign):
    from automap_pro.srv._trigger_gps_align import TriggerGpsAlign_Request as Request
    from automap_pro.srv._trigger_gps_align import TriggerGpsAlign_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
