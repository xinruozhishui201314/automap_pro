# generated from rosidl_generator_py/resource/_idl.py.em
# with input from overlap_transformer_msgs:srv/ComputeDescriptor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ComputeDescriptor_Request(type):
    """Metaclass of message 'ComputeDescriptor_Request'."""

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
            module = import_type_support('overlap_transformer_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'overlap_transformer_msgs.srv.ComputeDescriptor_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__compute_descriptor__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__compute_descriptor__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__compute_descriptor__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__compute_descriptor__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__compute_descriptor__request

            from sensor_msgs.msg import PointCloud2
            if PointCloud2.__class__._TYPE_SUPPORT is None:
                PointCloud2.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeDescriptor_Request(metaclass=Metaclass_ComputeDescriptor_Request):
    """Message class 'ComputeDescriptor_Request'."""

    __slots__ = [
        '_pointcloud',
    ]

    _fields_and_field_types = {
        'pointcloud': 'sensor_msgs/PointCloud2',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import PointCloud2
        self.pointcloud = kwargs.get('pointcloud', PointCloud2())

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
        if self.pointcloud != other.pointcloud:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pointcloud(self):
        """Message field 'pointcloud'."""
        return self._pointcloud

    @pointcloud.setter
    def pointcloud(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'pointcloud' field must be a sub message of type 'PointCloud2'"
        self._pointcloud = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ComputeDescriptor_Response(type):
    """Metaclass of message 'ComputeDescriptor_Response'."""

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
            module = import_type_support('overlap_transformer_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'overlap_transformer_msgs.srv.ComputeDescriptor_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__compute_descriptor__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__compute_descriptor__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__compute_descriptor__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__compute_descriptor__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__compute_descriptor__response

            from std_msgs.msg import Float32MultiArray
            if Float32MultiArray.__class__._TYPE_SUPPORT is None:
                Float32MultiArray.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ComputeDescriptor_Response(metaclass=Metaclass_ComputeDescriptor_Response):
    """Message class 'ComputeDescriptor_Response'."""

    __slots__ = [
        '_descriptor',
    ]

    _fields_and_field_types = {
        'descriptor': 'std_msgs/Float32MultiArray',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Float32MultiArray'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Float32MultiArray
        self.descriptor = kwargs.get('descriptor', Float32MultiArray())

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
        if self.descriptor != other.descriptor:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def descriptor(self):
        """Message field 'descriptor'."""
        return self._descriptor

    @descriptor.setter
    def descriptor(self, value):
        if __debug__:
            from std_msgs.msg import Float32MultiArray
            assert \
                isinstance(value, Float32MultiArray), \
                "The 'descriptor' field must be a sub message of type 'Float32MultiArray'"
        self._descriptor = value


class Metaclass_ComputeDescriptor(type):
    """Metaclass of service 'ComputeDescriptor'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('overlap_transformer_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'overlap_transformer_msgs.srv.ComputeDescriptor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__compute_descriptor

            from overlap_transformer_msgs.srv import _compute_descriptor
            if _compute_descriptor.Metaclass_ComputeDescriptor_Request._TYPE_SUPPORT is None:
                _compute_descriptor.Metaclass_ComputeDescriptor_Request.__import_type_support__()
            if _compute_descriptor.Metaclass_ComputeDescriptor_Response._TYPE_SUPPORT is None:
                _compute_descriptor.Metaclass_ComputeDescriptor_Response.__import_type_support__()


class ComputeDescriptor(metaclass=Metaclass_ComputeDescriptor):
    from overlap_transformer_msgs.srv._compute_descriptor import ComputeDescriptor_Request as Request
    from overlap_transformer_msgs.srv._compute_descriptor import ComputeDescriptor_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
