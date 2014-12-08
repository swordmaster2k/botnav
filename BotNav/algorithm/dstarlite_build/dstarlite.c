/*
 * D* Lite (modified version)
 * 
 * Modified version of D* Lite example provided by the algorithms
 * authors. The modifications include large amounts of removed example
 * code, refactoring, commenting, and modularisation.
 * 
 * Note: This version of D* Lite is optimized for grids 
 * It assumes, for example, that no cell can be a successor of itself.
 * 
 * N.B: Remember that if this is used on a Pi for example it will need
 * to be compiled for ARM architecture.
 * 
 * Modified By:
 * 	Joshua Michael Daly (ITB)
 * 
 * Original Authors: 
 * 	Maxim Likhachev (CMU) and Sven Koenig (USC) 
 */ 

#include "dstarlite.h"

/*
 * *********************************************************************
 * Callable Functions
 * *********************************************************************
 */
 
/*
 * 
 */
static PyObject * 
setup(PyObject *self, PyObject *args)
{
	int result;
	const char *filepath;
	
	result = PyArg_ParseTuple(args, "s", &filepath);
	
	if (result)
	{
		FILE *file;

		file = fopen(filepath, "r");
	
		if (file == NULL)
		{ 
			result = -1;
		}
		else
		{
		    vertexaccesses = 0;
			establishmaze(file);
			initialize();
		}
	}
	
    return Py_BuildValue("i", result);
}

/*
 * 
 */ 
static PyObject * 
plan(PyObject *self, PyObject *args)
{
	computeshortestpath();

	// Return a path object like [(x1, y1), (xn, yn)].
	return Py_BuildValue("i", 0);
}

/*
*
*/
static PyObject *
getvertexaccesses(PyObject *self, PyObject *args)
{
    return Py_BuildValue("i", vertexaccesses);
}

static PyObject *
getrobotpath(PyObject *self, PyObject *args)
{
    cell *tmpcell;
    tmpcell = mazegoal->searchtree;
    int pathlength = 0;
    int i = 0;

    for (tmpcell = mazegoal->searchtree; tmpcell != NULL; tmpcell = tmpcell->searchtree)
	{
	    ++pathlength;
	}

	PyObject* list = PyList_New(pathlength);

	for (tmpcell = mazegoal->searchtree; tmpcell != NULL; tmpcell = tmpcell->searchtree)
	{
	    PyList_SetItem(list, i, Py_BuildValue("(i,i)", tmpcell->x, tmpcell->y));
	    ++i;
	}

	return list;
}

/*
*
*/
static PyObject *
getoccupancygrid(PyObject *self, PyObject *args)
{
    int x, y;
    PyObject* grid = PyList_New(mazesize);

    for (y = mazesize - 1; y >= 0; --y)
    {
        PyObject* column = PyList_New(mazesize);

		for (x = 0; x < mazesize; ++x)
		{
			if (mazegoal == &maze[y][x])
			{
			    PyList_SetItem(column, x, Py_BuildValue("s", "R"));
			}
			else if (mazestart == &maze[y][x])
			{
			    PyList_SetItem(column, x, Py_BuildValue("s", "G"));
			}
			else if (maze[y][x].obstacle)
			{
			    PyList_SetItem(column, x, Py_BuildValue("s", "#"));
			}
			else
			{
			    PyList_SetItem(column, x, Py_BuildValue("s", " "));
			}
		}

		PyList_SetItem(grid, y, column);
    }

    return grid;
}

/*
*
*/
static PyObject *
getcostgrid(PyObject *self, PyObject *args)
{
    int x, y;
    PyObject* grid = PyList_New(mazesize);

    for (y = mazesize - 1; y >= 0; --y)
    {
        PyObject* column = PyList_New(mazesize);

		for (x = 0; x < mazesize; ++x)
		{
			PyList_SetItem(column, x, Py_BuildValue("i", maze[y][x].g));
		}

		PyList_SetItem(grid, y, column);
    }

    return grid;
}

/*
 * 
 */
static PyObject * 
updaterobotposition(PyObject *self, PyObject *args)
{
	int result; /* 1 for success */
	int x, y; /* x, y position of robot */
	
	/* 2 parameters passed as integers x, y */
	result = PyArg_ParseTuple(args, "ii", &x, &y);
	
	/* simply assign cell data */
	mazegoal = &maze[y][x];
	initializecell(mazegoal);
	
	return Py_BuildValue("i", 0);
}

/*
 * 
 */ 
static PyObject * 
updatecelloccupancy(PyObject *self, PyObject *args)
{
	int result; /* 1 for success */
	int x, y; /* x, y position of cell */
	int occupancy; /* new occupancy state */
	
	/* 3 parameters passed as integers x, y, occupancy */
	result = PyArg_ParseTuple(args, "iii", &x, &y, &occupancy);
	
	if (result)
	{
		/* validate the parameters */
		if ((x > -1 && x < mazesize) && (y > -1 && y < mazesize) && 
			(occupancy > -1 && occupancy < 2))
		{
			/* check for a change */
			if (maze[y][x].obstacle != occupancy)
			{
				maze[y][x].obstacle = occupancy;
				
				int d1;
				
				/* cycle over all the connected cells looking for a
				 * non-NULL move, if exists use it to update maze
				 * it doesn't matter if all the surrounding cells
				 * are obstacles */
				for (d1 = 0; d1 < DIRECTIONS; ++d1)
				{
					if (maze[y][x].move[d1])
					{
						updatemaze(maze[y][x].move[d1]);
						break;
					}
				}
			}
		}
		else
		{
			result = -1;
		}
	}
	
	return Py_BuildValue("i", result); 
}

/*
 * module initializer
 */  
PyMODINIT_FUNC PyInit_dstarlite_c()      /* called on first import */
{                                      /* name matters if loaded dynamically */
	return PyModule_Create(&dstarlitemodule); 
}

/*
 * *********************************************************************
 * D* Lite
 * *********************************************************************
 */

/*
 *
 */
void initialize()
{
    ++mazeiteration;
    keymodifier = 0;
    mazestart->g = LARGE;
    mazestart->rhs = 0;
    
#ifdef TIEBREAKING
    emptyheap(3);
    mazestart->key[0] = H(mazestart);
    mazestart->key[1] = H(mazestart) + 1;
    mazestart->key[2] = H(mazestart);
#else
    emptyheap(2);
    mazestart->key[0] = H(mazestart);
    mazestart->key[1] = 0;
#endif

    mazestart->searchtree = NULL;
    mazestart->generated = mazeiteration;
    insertheap(mazestart);
    mazegoal->g = LARGE;
    mazegoal->rhs = LARGE;
    mazegoal->searchtree = NULL;
    mazegoal->generated = mazeiteration;
}

/*
 * replan
 */
int computeshortestpath()
{
    cell *tmpcell1, *tmpcell2;
    int x, d;

#ifdef TIEBREAKING
    if (mazegoal->g < mazegoal->rhs)
    {
		goaltmpcell.key[0] = mazegoal->g + keymodifier;
		goaltmpcell.key[1] = mazegoal->g + keymodifier;
		goaltmpcell.key[2] = mazegoal->g;
    }
    else
    {
		goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
		goaltmpcell.key[1] = mazegoal->rhs + keymodifier + 1;
		goaltmpcell.key[2] = keymodifier;
    }
#else
    if (mazegoal->g < mazegoal->rhs)
    {
		goaltmpcell.key[0] = mazegoal->g + keymodifier;
		goaltmpcell.key[1] = mazegoal->g;
    }
    else
    {
		goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
		goaltmpcell.key[1] = mazegoal->rhs;
    }
#endif
    while (topheap() && (mazegoal->rhs > mazegoal->g || keyless(topheap(), &goaltmpcell)))
    {
		tmpcell1 = topheap();
		oldtmpcell.key[0] = tmpcell1->key[0];
		oldtmpcell.key[1] = tmpcell1->key[1];
		
#ifdef TIEBREAKING
		oldtmpcell.key[2] = tmpcell1->key[2];
#endif

		updatekey(tmpcell1);
		 
		if (keyless(&oldtmpcell, tmpcell1))
		{
			updatecell(tmpcell1);
		}
		else if (tmpcell1->g > tmpcell1->rhs)
		{
			tmpcell1->g = tmpcell1->rhs;
			deleteheap(tmpcell1);
			
			for (d = 0; d < DIRECTIONS; ++d)
			{
				if (tmpcell1->move[d])
				{
					tmpcell2 = tmpcell1->move[d];
					initializecell(tmpcell2);
					
					if (tmpcell2 != mazestart && tmpcell2->rhs > tmpcell1->g + 1)
					{
						tmpcell2->rhs = tmpcell1->g + 1;
						tmpcell2->searchtree = tmpcell1;
						updatecell(tmpcell2);
					}
				}
			}
		}
		else
		{
			tmpcell1->g = LARGE;
			updatecell(tmpcell1);
		
			for (d = 0; d < DIRECTIONS; ++d) 
			{
				if (tmpcell1->move[d])
				{
					tmpcell2 = tmpcell1->move[d];
					initializecell(tmpcell2);
					
					if (tmpcell2 != mazestart && tmpcell2->searchtree == tmpcell1)
					{
						updaterhs(tmpcell2);
					}
				}
			}
		}
	
#ifdef TIEBREAKING
		if (mazegoal->g < mazegoal->rhs)
		{
			goaltmpcell.key[0] = mazegoal->g + keymodifier;
			goaltmpcell.key[1] = mazegoal->g + keymodifier;
			goaltmpcell.key[2] = mazegoal->g;
		}
		else
		{
			goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
			goaltmpcell.key[1] = mazegoal->rhs + keymodifier + 1;
			goaltmpcell.key[2] = keymodifier;
		}    
#else
		if (mazegoal->g < mazegoal->rhs)
		{
			goaltmpcell.key[0] = mazegoal->g + keymodifier;
			goaltmpcell.key[1] = mazegoal->g;
		}
		else
		{
			goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
			goaltmpcell.key[1] = mazegoal->rhs;
		}
#endif
    }
    
	return (mazegoal->rhs == LARGE);
}

/*
 * 
 */
void updatemaze(cell *robot)
{
    int d1, d2;
    cell *tmpcell;

    for (d1 = 0; d1 < DIRECTIONS; ++d1)
    {
		if (robot->move[d1] && robot->move[d1]->obstacle)
		{
			tmpcell = robot->move[d1];
			initializecell(tmpcell);
			
			for (d2 = 0; d2 < DIRECTIONS; ++d2)
			{
				if (tmpcell->move[d2])
				{
					tmpcell->move[d2] = NULL;
					tmpcell->succ[d2]->move[reverse[d2]] = NULL;
					initializecell(tmpcell->succ[d2]);
					
					if (tmpcell->succ[d2] != mazestart && tmpcell->succ[d2]->searchtree == tmpcell)
					{
						updaterhs(tmpcell->succ[d2]);
					}
				}
			}
				
			if (tmpcell != mazestart)
			{
				tmpcell->rhs = LARGE;
				updatecell(tmpcell);
			}
		}
	}
}

/*
 *
 */
void initializecell(cell *thiscell)
{
    if (thiscell->generated != mazeiteration)
    {
		thiscell->g = LARGE;
		thiscell->rhs = LARGE;
		thiscell->searchtree = NULL;
		thiscell->generated = mazeiteration;
    }
}

/*
 *
 */
void updatecell(cell *thiscell)
{
    ++vertexaccesses;

    if (thiscell->g < thiscell->rhs)
    {
#ifdef TIEBREAKING
		thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
		thiscell->key[1] = thiscell->g + H(thiscell) + keymodifier;
		thiscell->key[2] = thiscell->g;
#else
		thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
		thiscell->key[1] = thiscell->g; 
#endif
		insertheap(thiscell);
    }
    else if (thiscell->g > thiscell->rhs)
    {
#ifdef TIEBREAKING
		thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
		thiscell->key[1] = thiscell->rhs + H(thiscell) + keymodifier + 1;
		thiscell->key[2] = H(thiscell) + keymodifier;
#else
		thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
		thiscell->key[1] = thiscell->rhs;
#endif
		insertheap(thiscell);
    }
    else
    { 
		deleteheap(thiscell);
	}
}

/*
 * 
 */
void updatekey(cell *thiscell)
{
    if (thiscell->g < thiscell->rhs)
    {
#ifdef TIEBREAKING
		thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
		thiscell->key[1] = thiscell->g + H(thiscell) + keymodifier;
		thiscell->key[2] = thiscell->g;
#else
        thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
		thiscell->key[1] = thiscell->g; 
#endif
    }
    else 
    {
#ifdef TIEBREAKING
		thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
		thiscell->key[1] = thiscell->rhs + H(thiscell) + keymodifier + 1;
		thiscell->key[2] = H(thiscell) + keymodifier;
#else
        thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
        thiscell->key[1] = thiscell->rhs;
#endif
    }
}

/*
 *
 */
void updaterhs(cell *thiscell)
{
    int d;

    thiscell->rhs = LARGE;
    thiscell->searchtree = NULL;
    
    for (d = 0; d < DIRECTIONS; ++d)
    {
		if (thiscell->move[d] && thiscell->move[d]->generated == 
			mazeiteration && thiscell->rhs > thiscell->move[d]->g + 1)
		{
			thiscell->rhs = thiscell->move[d]->g + 1;
			thiscell->searchtree = thiscell->move[d];
		}
    }
    
    updatecell(thiscell);
}

/*
 * *********************************************************************
 * Main - To use this comment out Python C API code
 * *********************************************************************
 */

/*
 *
 */
 /*
int main(int argc, char *argv[])
{
    int k, l;
    cell *tmpcell;
    cell *lastcell;
    
    for (k = 0; k < RUNS; ++k)
    {
		printf("maze %d\n", k);

		establishmaze();

#ifdef DISPLAY
		printactualmaze(stdout);
#endif

		initialize();
		fflush(stdout);
		lastcell = mazegoal;
		
		while (mazestart != mazegoal)
		{
			if (computeshortestpath())
			{
				break; // robot has reached its goal
			}

			mazegoal->trace = NULL;
			
			// traverse the current path until we reach the goal or
			// encounter an obstacle 
			do
			{
				#ifdef DISPLAY
				printrobotpath(stdout);
				printknownmaze(stdout);
				#endif
				
				// store where we are moving from 
				mazegoal->searchtree->trace = mazegoal;
				
				// simulated movement of robot 
				mazegoal = mazegoal->searchtree; 
			} while (mazestart != mazegoal && !mazegoal->searchtree->obstacle);
			
			if (mazestart != mazegoal)
			{
				// encountered an obstacle update the cell
				keymodifier += H(lastcell);
				lastcell = mazegoal;
				
				for (tmpcell = mazegoal; tmpcell; tmpcell = tmpcell->trace)
				{
					updatemaze(tmpcell);
				}
			}
		}
		
		printknownmaze(stdout);
    }
}
*/
