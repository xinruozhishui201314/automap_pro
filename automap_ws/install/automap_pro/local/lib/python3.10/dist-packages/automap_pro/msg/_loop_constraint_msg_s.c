// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from automap_pro:msg/LoopConstraintMsg.idl
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
#include "automap_pro/msg/detail/loop_constraint_msg__struct.h"
#include "automap_pro/msg/detail/loop_constraint_msg__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

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
bool automap_pro__msg__loop_constraint_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[55];
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
    assert(strncmp("automap_pro.msg._loop_constraint_msg.LoopConstraintMsg", full_classname_dest, 54) == 0);
  }
  automap_pro__msg__LoopConstraintMsg * ros_message = _ros_message;
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
  {  // submap_i
    PyObject * field = PyObject_GetAttrString(_pymsg, "submap_i");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->submap_i = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // submap_j
    PyObject * field = PyObject_GetAttrString(_pymsg, "submap_j");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->submap_j = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // session_i
    PyObject * field = PyObject_GetAttrString(_pymsg, "session_i");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->session_i = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // session_j
    PyObject * field = PyObject_GetAttrString(_pymsg, "session_j");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->session_j = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // overlap_score
    PyObject * field = PyObject_GetAttrString(_pymsg, "overlap_score");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->overlap_score = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // inlier_ratio
    PyObject * field = PyObject_GetAttrString(_pymsg, "inlier_ratio");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->inlier_ratio = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rmse
    PyObject * field = PyObject_GetAttrString(_pymsg, "rmse");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rmse = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // is_inter_session
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_inter_session");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_inter_session = (Py_True == field);
    Py_DECREF(field);
  }
  {  // information_matrix
    PyObject * field = PyObject_GetAttrString(_pymsg, "information_matrix");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 36;
      double * dest = ros_message->information_matrix;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // delta_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "delta_pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->delta_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->status, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * automap_pro__msg__loop_constraint_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LoopConstraintMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("automap_pro.msg._loop_constraint_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LoopConstraintMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  automap_pro__msg__LoopConstraintMsg * ros_message = (automap_pro__msg__LoopConstraintMsg *)raw_ros_message;
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
  {  // submap_i
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->submap_i);
    {
      int rc = PyObject_SetAttrString(_pymessage, "submap_i", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // submap_j
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->submap_j);
    {
      int rc = PyObject_SetAttrString(_pymessage, "submap_j", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // session_i
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->session_i);
    {
      int rc = PyObject_SetAttrString(_pymessage, "session_i", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // session_j
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->session_j);
    {
      int rc = PyObject_SetAttrString(_pymessage, "session_j", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // overlap_score
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->overlap_score);
    {
      int rc = PyObject_SetAttrString(_pymessage, "overlap_score", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // inlier_ratio
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->inlier_ratio);
    {
      int rc = PyObject_SetAttrString(_pymessage, "inlier_ratio", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rmse
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rmse);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rmse", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_inter_session
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_inter_session ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_inter_session", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // information_matrix
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "information_matrix");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->information_matrix[0]);
    memcpy(dst, src, 36 * sizeof(double));
    Py_DECREF(field);
  }
  {  // delta_pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->delta_pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "delta_pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // status
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->status.data,
      strlen(ros_message->status.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
