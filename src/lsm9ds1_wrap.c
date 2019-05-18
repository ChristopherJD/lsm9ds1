#include <stdio.h>
#include <Python.h>
#include "lsm9ds1.h"

// Module method definitions
static PyObject* init(PyObject *self, PyObject *args) {
	lsm9ds1_status_t function_return = LSM9DS1_UNKNOWN_ERROR;
	lsm9ds1_devices_t device = LSM9DS1_UNKNOWN_DEVICE;

	int bus_type = 0;
	int range = 0;
	int gain = 0;
	int scale = 0;
	if (!PyArg_ParseTuple(args, "iiii", &bus_type, &range, &gain, &scale)) {
		return NULL;
	}
	function_return = lsm9ds1_init(bus_type, range, gain, scale);

	return Py_BuildValue("i", device);
}

// Module method definitions
static PyObject* read_sub_device(PyObject *self, PyObject *args) {
	lsm9ds1_status_t function_return = LSM9DS1_UNKNOWN_ERROR;
	lsm9ds1_devices_t device = LSM9DS1_UNKNOWN_DEVICE;
	function_return = lsm9ds1_read_sub_device(&device);

	return Py_BuildValue("i", device);
}

// Method definition object for this extension, these argumens mean:
// ml_name: The name of the method
// ml_meth: Function pointer to the method implementation
// ml_flags: Flags indicating special features of this method, such as
//          accepting arguments, accepting keyword arguments, being a
//          class method, or being a static method of a class.
// ml_doc:  Contents of this method's docstring
static PyMethodDef pylsm9ds1_methods[] = {
		{
			"read_sub_device",
			read_sub_device,
			METH_NOARGS,
			"Get the sub-device from the lsm9ds1."
		},
		{
			"init",
			init,
			METH_VARARGS,
			"Initialize the lsm9ds1."
		},
		{ NULL, NULL, 0, NULL } };

// Module definition
// The arguments of this structure tell Python what to call your extension,
// what it's methods are and where to look for it's method definitions
static struct PyModuleDef lsm9ds1_definition = { PyModuleDef_HEAD_INIT, "pylsm9ds1",
		"A Python module for controlling the lsm9ds1.", -1,
		pylsm9ds1_methods };

// Module initialization
// Python calls this function when importing your extension. It is important
// that this function is named PyInit_[[your_module_name]] exactly, and matches
// the name keyword argument in setup.py's setup() call.
PyMODINIT_FUNC PyInit_pylsm9ds1(void) {
	PyObject *m = NULL;
	Py_Initialize();
	m = PyModule_Create(&lsm9ds1_definition);
	if(m == NULL) {
		return m;
	}
	PyModule_AddIntConstant(m, "LSM9DS1_SPI_BUS", 0);
	return m;
}
