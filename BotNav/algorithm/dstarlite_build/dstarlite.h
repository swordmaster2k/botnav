#ifndef DSTARLITEH
#define DSTARLITEH

#include <Python.h>

#include "include.h"
#include "heap.h"
#include "maze.h"
#include "print.h"

int keymodifier;
cell goaltmpcell, oldtmpcell;

int vertexaccesses = 0; /* number of times cells are accessed */

/*
 * *********************************************************************
 * Callable Functions
 * *********************************************************************
 */

/*
 * 
 */  
static PyObject * 
setup(PyObject *self, PyObject *args); 

/*
 * 
 */ 
static PyObject * 
plan(PyObject *self, PyObject *args);

/*
*
*/
static PyObject *
getvertexaccesses(PyObject *self, PyObject *args);

/*
*
*/
static PyObject *
getrobotpath(PyObject *self, PyObject *args);

/*
*
*/
static PyObject *
getoccupancygrid(PyObject *self, PyObject *args);

/*
*
*/
static PyObject *
getcostgrid(PyObject *self, PyObject *args);

/*
 * 
 */
static PyObject * 
updaterobotposition(PyObject *self, PyObject *args);
 
/*
 * 
 */  
static PyObject *
updatecelloccupancy(PyObject *self, PyObject *args);

/*
 * registration table
 */  
static PyMethodDef dstarlite_methods[] = {
	{"setup", setup, METH_VARARGS, "func doc"},    /* name, &func, fmt, doc */
	{"plan", plan, METH_VARARGS, "func doc"},
	{"getvertexaccesses", getvertexaccesses, METH_VARARGS, "func doc"},
	{"getrobotpath", getrobotpath, METH_VARARGS, "func doc"},
	{"getoccupancygrid", getoccupancygrid, METH_VARARGS, "func doc"},
	{"getcostgrid", getcostgrid, METH_VARARGS, "func doc"},
	{"updaterobotposition", updaterobotposition, METH_VARARGS, "func doc"},  
	{"updatecelloccupancy", updatecelloccupancy, METH_VARARGS, "func doc"},
	{NULL, NULL, 0, NULL}                       /* end of table marker */
};

/*
 * 
 */ 
static struct PyModuleDef dstarlitemodule = { 
	PyModuleDef_HEAD_INIT,
	"dstarlite_c",
	"mod doc",
	-1,
	dstarlite_methods
};

/*
 * module initializer
 */ 
PyMODINIT_FUNC PyInit_dstarlite_c();
 
 /*
 * *********************************************************************
 * D* Lite
 * *********************************************************************
 */

/*
 * 
 */ 
void initialize();

/*
 * 
 */ 
int computeshortestpath();

/*
 * 
 */ 
void updatemaze(cell *robot);

/*
 * 
 */ 
void initializecell(cell *thiscell);

/*
 * 
 */ 
void updatecell(cell *thiscell);

/*
 * 
 */ 
void updatekey(cell *thiscell);

/*
 * 
 */ 
void updaterhs(cell *thiscell);

/*
 * *********************************************************************
 * Main - To use this comment out Python C API code
 * *********************************************************************
 */

int main(int argc, char *argv[]);

#endif
