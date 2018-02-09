#include "ScriptLoader.h"
#include <Python.h>



glm::vec3 ScriptLoader::GetVec3(const std::string scriptPath, const std::string functionName)
{
	glm::vec3 result;

	PyObject *pName, *pModule, *pDict, *pFunc, *pValue;

	Py_Initialize();

	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"C:\\Python27\")");

	pName = PyString_FromString(scriptPath.c_str());
	pModule = PyImport_Import(pName);
	pDict = PyModule_GetDict(pModule);
	pFunc = PyDict_GetItemString(pDict, functionName.c_str());

	if (PyCallable_Check(pFunc))
	{
		pValue = PyObject_CallObject(pFunc, NULL);
		result.x = PyFloat_AsDouble(PyTuple_GetItem(pValue, 0));
		result.y = PyFloat_AsDouble(PyTuple_GetItem(pValue, 1));
		result.z = PyFloat_AsDouble(PyTuple_GetItem(pValue, 2));
	}
	else
	{
		PyErr_Print();
	}

	Py_DECREF(pModule);
	Py_DECREF(pName);


	return result;
}

