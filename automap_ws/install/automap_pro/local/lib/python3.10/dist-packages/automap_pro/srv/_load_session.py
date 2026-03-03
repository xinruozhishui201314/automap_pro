# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:srv/LoadSession.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LoadSession_Request(type):
    """Metaclass of message 'LoadSession_Request'."""

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
                'automap_pro.srv.LoadSession_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__load_session__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__load_session__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__load_session__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__load_session__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__load_session__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LoadSession_Request(metaclass=Metaclass_LoadSession_Request):
    """Message class 'LoadSession_Request'."""

    __slots__ = [
        '_session_dir',
        '_session_id',
    ]

    _fields_and_field_types = {
        'session_dir': 'string',
        'session_id': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.session_dir = kwargs.get('session_dir', str())
        self.session_id = kwargs.get('session_id', int())

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
        if self.session_dir != other.session_dir:
            return False
        if self.session_id != other.session_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def session_dir(self):
        """Message field 'session_dir'."""
        return self._session_dir

    @session_dir.setter
    def session_dir(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'session_dir' field must be of type 'str'"
        self._session_dir = value

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LoadSession_Response(type):
    """Metaclass of message 'LoadSession_Response'."""

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
                'automap_pro.srv.LoadSession_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__load_session__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__load_session__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__load_session__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__load_session__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__load_session__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LoadSession_Response(metaclass=Metaclass_LoadSession_Response):
    """Message class 'LoadSession_Response'."""

    __slots__ = [
        '_success',
        '_submaps_loaded',
        '_descriptors_loaded',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'submaps_loaded': 'uint32',
        'descriptors_loaded': 'uint32',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.submaps_loaded = kwargs.get('submaps_loaded', int())
        self.descriptors_loaded = kwargs.get('descriptors_loaded', int())
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
        if self.submaps_loaded != other.submaps_loaded:
            return False
        if self.descriptors_loaded != other.descriptors_loaded:
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
    def submaps_loaded(self):
        """Message field 'submaps_loaded'."""
        return self._submaps_loaded

    @submaps_loaded.setter
    def submaps_loaded(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'submaps_loaded' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'submaps_loaded' field must be an unsigned integer in [0, 4294967295]"
        self._submaps_loaded = value

    @builtins.property
    def descriptors_loaded(self):
        """Message field 'descriptors_loaded'."""
        return self._descriptors_loaded

    @descriptors_loaded.setter
    def descriptors_loaded(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'descriptors_loaded' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'descriptors_loaded' field must be an unsigned integer in [0, 4294967295]"
        self._descriptors_loaded = value

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


class Metaclass_LoadSession(type):
    """Metaclass of service 'LoadSession'."""

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
                'automap_pro.srv.LoadSession')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__load_session

            from automap_pro.srv import _load_session
            if _load_session.Metaclass_LoadSession_Request._TYPE_SUPPORT is None:
                _load_session.Metaclass_LoadSession_Request.__import_type_support__()
            if _load_session.Metaclass_LoadSession_Response._TYPE_SUPPORT is None:
                _load_session.Metaclass_LoadSession_Response.__import_type_support__()


class LoadSession(metaclass=Metaclass_LoadSession):
    from automap_pro.srv._load_session import LoadSession_Request as Request
    from automap_pro.srv._load_session import LoadSession_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
