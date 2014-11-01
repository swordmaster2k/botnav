/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */

#ifndef MAZEH
#define MAZEH

#include "include.h"

struct cell;
typedef struct cell cell;

struct cell
{
    cell *move[DIRECTIONS]; /* list of cells can move to from this one */
    cell *succ[DIRECTIONS]; /* list of successor cells */
    cell *searchtree;		/* use this to get the path */
    cell *trace;			/* where the robot has been */		
    short obstacle;			/* 0 = free, 1 = obstacle */
    int x, y;				/* x, y coordinates of cell in grid*/
    int g;					/* estimate of distance to goal */
    int rhs;				/* one step lookahead value based on g */
    int key[3];				/*  */
    int generated;			/*  */
    int heapindex;			/* position of cell on the heap */
};

/* Note: mazegoal is the start cell of the robot. */
/* Note: mazestart is the goal cell of the robot. */
cell **maze;
cell *mazestart, *mazegoal; 
int mazeiteration;

int mazesize;

/* Note: actually the position of the robot */
int goalx;
int goaly;

/* Note: actually the position of the goal */
int startx;
int starty;

void establishmaze(FILE *file);

#endif
