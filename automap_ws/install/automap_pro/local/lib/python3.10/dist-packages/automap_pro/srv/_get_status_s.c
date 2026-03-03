// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from automap_pro:srv/GetStatus.idl
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
#include "automap_pro/srv/detail/get_status__struct.h"
#include "automap_pro/srv/detail/get_status__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool automap_pro__srv__get_status__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("automap_pro.srv._get_status.GetStatus_Request", full_classname_dest, 45) == 0);
  }
  automap_pro__srv__GetStatus_Request * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * automap_pro__srv__get_status__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetStatus_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("automap_pro.srv._get_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetStatus_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "automap_pro/srv/detail/get_status__struct.h"
// already included above
// #include "automap_pro/srv/detail/get_status__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool automap_pro__srv__get_status__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("automap_pro.srv._get_status.GetStatus_Response", full_classname_dest, 46) == 0);
  }
  automap_pro__srv__GetStatus_Response * ros_message = _ros_message;
  {  // state
    PyObject * field = PyObject_GetAttrString(_pymsg, "state");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->state, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
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
  {  // keyframe_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "keyframe_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->keyframe_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // submap_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "submap_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->submap_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // loop_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "loop_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->loop_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // gps_aligned
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_aligned");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->gps_aligned = (Py_True == field);
    Py_DECREF(field);
  }
  {  // gps_align_score
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_align_score");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->gps_align_score = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // total_distance_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "total_distance_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->total_distance_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * automap_pro__srv__get_status__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetStatus_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("automap_pro.srv._get_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetStatus_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  automap_pro__srv__GetStatus_Response * ros_message = (automap_pro__srv__GetStatus_Response *)raw_ros_message;
  {  // state
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->state.data,
      strlen(ros_message->state.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "state", field);
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
  {  // keyframe_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->keyframe_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "keyframe_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // submap_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->submap_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "submap_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // loop_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->loop_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "loop_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gps_aligned
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->gps_aligned ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_aligned", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gps_align_score
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->gps_align_score);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_align_score", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // total_distance_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->total_distance_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "total_distance_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
