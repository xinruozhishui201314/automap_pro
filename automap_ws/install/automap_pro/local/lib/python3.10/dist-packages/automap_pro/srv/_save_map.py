# generated from rosidl_generator_py/resource/_idl.py.em
# with input from automap_pro:srv/SaveMap.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SaveMap_Request(type):
    """Metaclass of message 'SaveMap_Request'."""

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
                'automap_pro.srv.SaveMap_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__save_map__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__save_map__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__save_map__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__save_map__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__save_map__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SaveMap_Request(metaclass=Metaclass_SaveMap_Request):
    """Message class 'SaveMap_Request'."""

    __slots__ = [
        '_output_dir',
        '_save_pcd',
        '_save_ply',
        '_save_las',
        '_save_trajectory_tum',
        '_save_trajectory_kitti',
    ]

    _fields_and_field_types = {
        'output_dir': 'string',
        'save_pcd': 'boolean',
        'save_ply': 'boolean',
        'save_las': 'boolean',
        'save_trajectory_tum': 'boolean',
        'save_trajectory_kitti': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.output_dir = kwargs.get('output_dir', str())
        self.save_pcd = kwargs.get('save_pcd', bool())
        self.save_ply = kwargs.get('save_ply', bool())
        self.save_las = kwargs.get('save_las', bool())
        self.save_trajectory_tum = kwargs.get('save_trajectory_tum', bool())
        self.save_trajectory_kitti = kwargs.get('save_trajectory_kitti', bool())

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
        if self.output_dir != other.output_dir:
            return False
        if self.save_pcd != other.save_pcd:
            return False
        if self.save_ply != other.save_ply:
            return False
        if self.save_las != other.save_las:
            return False
        if self.save_trajectory_tum != other.save_trajectory_tum:
            return False
        if self.save_trajectory_kitti != other.save_trajectory_kitti:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def output_dir(self):
        """Message field 'output_dir'."""
        return self._output_dir

    @output_dir.setter
    def output_dir(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'output_dir' field must be of type 'str'"
        self._output_dir = value

    @builtins.property
    def save_pcd(self):
        """Message field 'save_pcd'."""
        return self._save_pcd

    @save_pcd.setter
    def save_pcd(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'save_pcd' field must be of type 'bool'"
        self._save_pcd = value

    @builtins.property
    def save_ply(self):
        """Message field 'save_ply'."""
        return self._save_ply

    @save_ply.setter
    def save_ply(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'save_ply' field must be of type 'bool'"
        self._save_ply = value

    @builtins.property
    def save_las(self):
        """Message field 'save_las'."""
        return self._save_las

    @save_las.setter
    def save_las(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'save_las' field must be of type 'bool'"
        self._save_las = value

    @builtins.property
    def save_trajectory_tum(self):
        """Message field 'save_trajectory_tum'."""
        return self._save_trajectory_tum

    @save_trajectory_tum.setter
    def save_trajectory_tum(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'save_trajectory_tum' field must be of type 'bool'"
        self._save_trajectory_tum = value

    @builtins.property
    def save_trajectory_kitti(self):
        """Message field 'save_trajectory_kitti'."""
        return self._save_trajectory_kitti

    @save_trajectory_kitti.setter
    def save_trajectory_kitti(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'save_trajectory_kitti' field must be of type 'bool'"
        self._save_trajectory_kitti = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SaveMap_Response(type):
    """Metaclass of message 'SaveMap_Response'."""

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
                'automap_pro.srv.SaveMap_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__save_map__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__save_map__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__save_map__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__save_map__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__save_map__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SaveMap_Response(metaclass=Metaclass_SaveMap_Response):
    """Message class 'SaveMap_Response'."""

    __slots__ = [
        '_success',
        '_output_path',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'output_path': 'string',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.output_path = kwargs.get('output_path', str())
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
        if self.output_path != other.output_path:
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
    def output_path(self):
        """Message field 'output_path'."""
        return self._output_path

    @output_path.setter
    def output_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'output_path' field must be of type 'str'"
        self._output_path = value

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


class Metaclass_SaveMap(type):
    """Metaclass of service 'SaveMap'."""

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
                'automap_pro.srv.SaveMap')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__save_map

            from automap_pro.srv import _save_map
            if _save_map.Metaclass_SaveMap_Request._TYPE_SUPPORT is None:
                _save_map.Metaclass_SaveMap_Request.__import_type_support__()
            if _save_map.Metaclass_SaveMap_Response._TYPE_SUPPORT is None:
                _save_map.Metaclass_SaveMap_Response.__import_type_support__()


class SaveMap(metaclass=Metaclass_SaveMap):
    from automap_pro.srv._save_map import SaveMap_Request as Request
    from automap_pro.srv._save_map import SaveMap_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
