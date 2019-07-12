/* 
 * This file is part of the lsm9ds1 library (https://github.com/ChristopherJD/lsm9ds1.git).
 * Copyright (c) 2019 Christopher Jordan-Denny.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <Python.h>
#include "lsm9ds1.h"

lsm9ds1_device_t *lsm9ds1 = NULL;

// Module method definitions
static PyObject* init(PyObject *self, PyObject *args) {
	lsm9ds1_sub_device_t device = LSM9DS1_UNKNOWN_DEVICE;

	int bus_type = 0;
	int range = 0;
	int gain = 0;
	int scale = 0;
	if (!PyArg_ParseTuple(args, "iiii", &bus_type, &range, &gain, &scale)) {
		return NULL;
	}
	lsm9ds1 = malloc(sizeof(lsm9ds1_device_t));
	(void)lsm9ds1_init(lsm9ds1, bus_type, range, gain, scale);

	return Py_BuildValue("i", device);
}

// Module method definitions
static PyObject* get_temp(PyObject *self, PyObject *args) {

	lsm9ds1->update_temp(lsm9ds1);

	return Py_BuildValue("f", lsm9ds1->converted_data.temperature);
}

static PyObject* get_accel(PyObject *self, PyObject *args) {

	lsm9ds1->update_accel(lsm9ds1);

	return Py_BuildValue("fff", lsm9ds1->converted_data.accelerometer.x, 
		lsm9ds1->converted_data.accelerometer.y, 
		lsm9ds1->converted_data.accelerometer.z);
}

static PyObject* get_mag(PyObject *self, PyObject *args) {

	lsm9ds1->update_mag(lsm9ds1);

	return Py_BuildValue("fff", lsm9ds1->converted_data.magnetometer.x, 
		lsm9ds1->converted_data.magnetometer.y, 
		lsm9ds1->converted_data.magnetometer.z);
}

static PyObject* get_gyro(PyObject *self, PyObject *args) {

	lsm9ds1->update_gyro(lsm9ds1);

	return Py_BuildValue("fff", lsm9ds1->converted_data.gyroscope.x, 
		lsm9ds1->converted_data.gyroscope.y, 
		lsm9ds1->converted_data.gyroscope.z);
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
		"get_temp",
		get_temp,
		METH_NOARGS,
		"Read the temperature from the lsm9ds1."
	},
	{
		"get_accel",
		get_accel,
		METH_NOARGS,
		"Read the accelerometer from the lsm9ds1."
	},
	{
		"get_mag",
		get_mag,
		METH_NOARGS,
		"Read the magnetometer from the lsm9ds1."
	},
	{
		"get_gyro",
		get_gyro,
		METH_NOARGS,
		"Read the gyroscope from the lsm9ds1."
	},
	{
		"init",
		init,
		METH_VARARGS,
		"Initialize the lsm9ds1."
	},
	{ NULL, NULL, 0, NULL }
};

// Module definition
// The arguments of this structure tell Python what to call your extension,
// what it's methods are and where to look for it's method definitions
static struct PyModuleDef lsm9ds1_definition = { PyModuleDef_HEAD_INIT, "pylsm9ds1",
	"A Python module for controlling the lsm9ds1.", -1,
	pylsm9ds1_methods
};

// Module initialization
// Python calls this function when importing your extension. It is important
// that this function is named PyInit_[[your_module_name]] exactly, and matches
// the name keyword argument in setup.py's setup() call.
PyMODINIT_FUNC PyInit_pylsm9ds1(void) {
	PyObject *m = NULL;
	Py_Initialize();
	m = PyModule_Create(&lsm9ds1_definition);
	if (m == NULL) {
		return m;
	}
	PyModule_AddIntConstant(m, "LSM9DS1_SPI_BUS", LSM9DS1_SPI_BUS);
	PyModule_AddIntConstant(m, "LSM9DS1_I2C_BUS", LSM9DS1_I2C_BUS);

	PyModule_AddIntConstant(m, "LSM9DS1_ACCELRANGE_2G", LSM9DS1_ACCELRANGE_2G);
	PyModule_AddIntConstant(m, "LSM9DS1_ACCELRANGE_16G", LSM9DS1_ACCELRANGE_16G);
	PyModule_AddIntConstant(m, "LSM9DS1_ACCELRANGE_4G", LSM9DS1_ACCELRANGE_4G);
	PyModule_AddIntConstant(m, "LSM9DS1_ACCELRANGE_8G", LSM9DS1_ACCELRANGE_8G);

	PyModule_AddIntConstant(m, "LSM9DS1_MAGGAIN_4GAUSS", LSM9DS1_MAGGAIN_4GAUSS);
	PyModule_AddIntConstant(m, "LSM9DS1_MAGGAIN_8GAUSS", LSM9DS1_MAGGAIN_8GAUSS);
	PyModule_AddIntConstant(m, "LSM9DS1_MAGGAIN_12GAUSS", LSM9DS1_MAGGAIN_12GAUSS);
	PyModule_AddIntConstant(m, "LSM9DS1_MAGGAIN_16GAUSS", LSM9DS1_MAGGAIN_16GAUSS);

	PyModule_AddIntConstant(m, "LSM9DS1_GYROSCALE_245DPS", LSM9DS1_GYROSCALE_245DPS);
	PyModule_AddIntConstant(m, "LSM9DS1_GYROSCALE_500DPS", LSM9DS1_GYROSCALE_500DPS);
	PyModule_AddIntConstant(m, "LSM9DS1_GYROSCALE_2000DPS", LSM9DS1_GYROSCALE_2000DPS);

	return m;
}
