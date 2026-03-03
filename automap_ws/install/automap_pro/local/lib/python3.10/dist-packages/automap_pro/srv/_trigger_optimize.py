# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:srv/TriggerOptimize.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TriggerOptimize_Request(type):
    """Metaclass of message 'TriggerOptimize_Request'."""

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
                'automap_pro.srv.TriggerOptimize_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__trigger_optimize__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__trigger_optimize__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__trigger_optimize__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__trigger_optimize__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__trigger_optimize__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TriggerOptimize_Request(metaclass=Metaclass_TriggerOptimize_Request):
    """Message class 'TriggerOptimize_Request'."""

    __slots__ = [
        '_full_optimization',
        '_max_iterations',
    ]

    _fields_and_field_types = {
        'full_optimization': 'boolean',
        'max_iterations': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.full_optimization = kwargs.get('full_optimization', bool())
        self.max_iterations = kwargs.get('max_iterations', int())

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
        if self.full_optimization != other.full_optimization:
            return False
        if self.max_iterations != other.max_iterations:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def full_optimization(self):
        """Message field 'full_optimization'."""
        return self._full_optimization

    @full_optimization.setter
    def full_optimization(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'full_optimization' field must be of type 'bool'"
        self._full_optimization = value

    @builtins.property
    def max_iterations(self):
        """Message field 'max_iterations'."""
        return self._max_iterations

    @max_iterations.setter
    def max_iterations(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'max_iterations' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'max_iterations' field must be an integer in [-2147483648, 2147483647]"
        self._max_iterations = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_TriggerOptimize_Response(type):
    """Metaclass of message 'TriggerOptimize_Response'."""

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
                'automap_pro.srv.TriggerOptimize_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__trigger_optimize__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__trigger_optimize__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__trigger_optimize__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__trigger_optimize__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__trigger_optimize__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TriggerOptimize_Response(metaclass=Metaclass_TriggerOptimize_Response):
    """Message class 'TriggerOptimize_Response'."""

    __slots__ = [
        '_success',
        '_elapsed_seconds',
        '_nodes_updated',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'elapsed_seconds': 'double',
        'nodes_updated': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.elapsed_seconds = kwargs.get('elapsed_seconds', float())
        self.nodes_updated = kwargs.get('nodes_updated', int())

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
        if self.elapsed_seconds != other.elapsed_seconds:
            return False
        if self.nodes_updated != other.nodes_updated:
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
    def elapsed_seconds(self):
        """Message field 'elapsed_seconds'."""
        return self._elapsed_seconds

    @elapsed_seconds.setter
    def elapsed_seconds(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'elapsed_seconds' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'elapsed_seconds' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._elapsed_seconds = value

    @builtins.property
    def nodes_updated(self):
        """Message field 'nodes_updated'."""
        return self._nodes_updated

    @nodes_updated.setter
    def nodes_updated(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'nodes_updated' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'nodes_updated' field must be an integer in [-2147483648, 2147483647]"
        self._nodes_updated = value


class Metaclass_TriggerOptimize(type):
    """Metaclass of service 'TriggerOptimize'."""

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
                'automap_pro.srv.TriggerOptimize')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__trigger_optimize

            from automap_pro.srv import _trigger_optimize
            if _trigger_optimize.Metaclass_TriggerOptimize_Request._TYPE_SUPPORT is None:
                _trigger_optimize.Metaclass_TriggerOptimize_Request.__import_type_support__()
            if _trigger_optimize.Metaclass_TriggerOptimize_Response._TYPE_SUPPORT is None:
                _trigger_optimize.Metaclass_TriggerOptimize_Response.__import_type_support__()


class TriggerOptimize(metaclass=Metaclass_TriggerOptimize):
    from automap_pro.srv._trigger_optimize import TriggerOptimize_Request as Request
    from automap_pro.srv._trigger_optimize import TriggerOptimize_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
