# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:srv/TriggerHBA.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TriggerHBA_Request(type):
    """Metaclass of message 'TriggerHBA_Request'."""

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
                'automap_pro.srv.TriggerHBA_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__trigger_hba__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__trigger_hba__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__trigger_hba__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__trigger_hba__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__trigger_hba__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TriggerHBA_Request(metaclass=Metaclass_TriggerHBA_Request):
    """Message class 'TriggerHBA_Request'."""

    __slots__ = [
        '_wait_for_result',
    ]

    _fields_and_field_types = {
        'wait_for_result': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.wait_for_result = kwargs.get('wait_for_result', bool())

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
        if self.wait_for_result != other.wait_for_result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def wait_for_result(self):
        """Message field 'wait_for_result'."""
        return self._wait_for_result

    @wait_for_result.setter
    def wait_for_result(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'wait_for_result' field must be of type 'bool'"
        self._wait_for_result = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_TriggerHBA_Response(type):
    """Metaclass of message 'TriggerHBA_Response'."""

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
                'automap_pro.srv.TriggerHBA_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__trigger_hba__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__trigger_hba__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__trigger_hba__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__trigger_hba__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__trigger_hba__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TriggerHBA_Response(metaclass=Metaclass_TriggerHBA_Response):
    """Message class 'TriggerHBA_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_elapsed_seconds',
        '_final_mme',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'elapsed_seconds': 'double',
        'final_mme': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.elapsed_seconds = kwargs.get('elapsed_seconds', float())
        self.final_mme = kwargs.get('final_mme', float())

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
        if self.message != other.message:
            return False
        if self.elapsed_seconds != other.elapsed_seconds:
            return False
        if self.final_mme != other.final_mme:
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
    def final_mme(self):
        """Message field 'final_mme'."""
        return self._final_mme

    @final_mme.setter
    def final_mme(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_mme' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_mme' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_mme = value


class Metaclass_TriggerHBA(type):
    """Metaclass of service 'TriggerHBA'."""

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
                'automap_pro.srv.TriggerHBA')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__trigger_hba

            from automap_pro.srv import _trigger_hba
            if _trigger_hba.Metaclass_TriggerHBA_Request._TYPE_SUPPORT is None:
                _trigger_hba.Metaclass_TriggerHBA_Request.__import_type_support__()
            if _trigger_hba.Metaclass_TriggerHBA_Response._TYPE_SUPPORT is None:
                _trigger_hba.Metaclass_TriggerHBA_Response.__import_type_support__()


class TriggerHBA(metaclass=Metaclass_TriggerHBA):
    from automap_pro.srv._trigger_hba import TriggerHBA_Request as Request
    from automap_pro.srv._trigger_hba import TriggerHBA_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
