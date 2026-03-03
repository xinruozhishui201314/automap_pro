// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from automap_pro:msg/SubMapEventMsg.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "automap_pro/msg/detail/sub_map_event_msg__struct.h"
#include "automap_pro/msg/detail/sub_map_event_msg__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool automap_pro__msg__sub_map_event_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("automap_pro.msg._sub_map_event_msg.SubMapEventMsg", full_classname_dest, 49) == 0);
  }
  automap_pro__msg__SubMapEventMsg * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // submap_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "submap_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->submap_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // session_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "session_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->session_id = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // event_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "event_type");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->event_type, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // keyframe_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "keyframe_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->keyframe_count = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // spatial_extent_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "spatial_extent_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->spatial_extent_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // anchor_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "anchor_pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->anchor_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // has_valid_gps
    PyObject * field = PyObject_GetAttrString(_pymsg, "has_valid_gps");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->has_valid_gps = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * automap_pro__msg__sub_map_event_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SubMapEventMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("automap_pro.msg._sub_map_event_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SubMapEventMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  automap_pro__msg__SubMapEventMsg * ros_message = (automap_pro__msg__SubMapEventMsg *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // submap_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->submap_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "submap_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // session_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->session_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "session_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // event_type
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->event_type.data,
      strlen(ros_message->event_type.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "event_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // keyframe_count
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->keyframe_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "keyframe_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // spatial_extent_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->spatial_extent_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "spatial_extent_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // anchor_pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->anchor_pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "anchor_pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // has_valid_gps
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->has_valid_gps ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "has_valid_gps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
