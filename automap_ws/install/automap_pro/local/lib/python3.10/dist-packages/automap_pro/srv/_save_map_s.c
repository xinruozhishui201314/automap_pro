// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from automap_pro:srv/SaveMap.idl
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
#include "automap_pro/srv/detail/save_map__struct.h"
#include "automap_pro/srv/detail/save_map__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool automap_pro__srv__save_map__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
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
    assert(strncmp("automap_pro.srv._save_map.SaveMap_Request", full_classname_dest, 41) == 0);
  }
  automap_pro__srv__SaveMap_Request * ros_message = _ros_message;
  {  // output_dir
    PyObject * field = PyObject_GetAttrString(_pymsg, "output_dir");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->output_dir, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // save_pcd
    PyObject * field = PyObject_GetAttrString(_pymsg, "save_pcd");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->save_pcd = (Py_True == field);
    Py_DECREF(field);
  }
  {  // save_ply
    PyObject * field = PyObject_GetAttrString(_pymsg, "save_ply");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->save_ply = (Py_True == field);
    Py_DECREF(field);
  }
  {  // save_las
    PyObject * field = PyObject_GetAttrString(_pymsg, "save_las");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->save_las = (Py_True == field);
    Py_DECREF(field);
  }
  {  // save_trajectory_tum
    PyObject * field = PyObject_GetAttrString(_pymsg, "save_trajectory_tum");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->save_trajectory_tum = (Py_True == field);
    Py_DECREF(field);
  }
  {  // save_trajectory_kitti
    PyObject * field = PyObject_GetAttrString(_pymsg, "save_trajectory_kitti");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->save_trajectory_kitti = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * automap_pro__srv__save_map__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SaveMap_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("automap_pro.srv._save_map");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SaveMap_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  automap_pro__srv__SaveMap_Request * ros_message = (automap_pro__srv__SaveMap_Request *)raw_ros_message;
  {  // output_dir
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->output_dir.data,
      strlen(ros_message->output_dir.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "output_dir", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // save_pcd
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->save_pcd ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "save_pcd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // save_ply
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->save_ply ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "save_ply", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // save_las
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->save_las ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "save_las", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // save_trajectory_tum
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->save_trajectory_tum ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "save_trajectory_tum", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // save_trajectory_kitti
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->save_trajectory_kitti ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "save_trajectory_kitti", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

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
// #include "automap_pro/srv/detail/save_map__struct.h"
// already included above
// #include "automap_pro/srv/detail/save_map__functions.h"

// already included above
// #include "rosidl_runtime_c/string.h"
// already included above
// #include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool automap_pro__srv__save_map__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("automap_pro.srv._save_map.SaveMap_Response", full_classname_dest, 42) == 0);
  }
  automap_pro__srv__SaveMap_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // output_path
    PyObject * field = PyObject_GetAttrString(_pymsg, "output_path");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->output_path, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * automap_pro__srv__save_map__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SaveMap_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("automap_pro.srv._save_map");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SaveMap_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  automap_pro__srv__SaveMap_Response * ros_message = (automap_pro__srv__SaveMap_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // output_path
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->output_path.data,
      strlen(ros_message->output_path.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "output_path", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
