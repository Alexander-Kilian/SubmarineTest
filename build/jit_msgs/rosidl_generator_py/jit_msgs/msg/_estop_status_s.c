// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from jit_msgs:msg/EstopStatus.idl
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
#include "jit_msgs/msg/detail/estop_status__struct.h"
#include "jit_msgs/msg/detail/estop_status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool jit_msgs__msg__estop_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
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
    assert(strncmp("jit_msgs.msg._estop_status.EstopStatus", full_classname_dest, 38) == 0);
  }
  jit_msgs__msg__EstopStatus * ros_message = _ros_message;
  {  // stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // commanded
    PyObject * field = PyObject_GetAttrString(_pymsg, "commanded");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->commanded = (Py_True == field);
    Py_DECREF(field);
  }
  {  // feedback
    PyObject * field = PyObject_GetAttrString(_pymsg, "feedback");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->feedback = (Py_True == field);
    Py_DECREF(field);
  }
  {  // mismatch
    PyObject * field = PyObject_GetAttrString(_pymsg, "mismatch");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->mismatch = (Py_True == field);
    Py_DECREF(field);
  }
  {  // settled
    PyObject * field = PyObject_GetAttrString(_pymsg, "settled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->settled = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * jit_msgs__msg__estop_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EstopStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("jit_msgs.msg._estop_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EstopStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  jit_msgs__msg__EstopStatus * ros_message = (jit_msgs__msg__EstopStatus *)raw_ros_message;
  {  // stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // commanded
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->commanded ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "commanded", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // feedback
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->feedback ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "feedback", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mismatch
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->mismatch ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mismatch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // settled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->settled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "settled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
