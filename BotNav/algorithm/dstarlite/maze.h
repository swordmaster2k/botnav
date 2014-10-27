/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */

#ifndef MAZEH
#define MAZEH

#include "include.h"

struct cell;
typedef struct cell cell;

struct cell
{
    cell *move[DIRECTIONS];
    cell *succ[DIRECTIONS];
    cell *searchtree;
    cell *trace;
    short obstacle;
    int x, y;
    int dfsx, dfsy; /* needed only for generating dfs mazes */
    int g;
    int rhs;
    int key[3];
    int generated;
    int heapindex;
};

/* Note: mazegoal is the start cell of the robot. */
/* Note: mazestart is the goal cell of the robot. */

cell **maze;
cell *mazestart, *mazegoal; 
int mazeiteration;

void newrandommaze();
void newdfsmaze(int wallstoremove);

#endif
