/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */

#ifndef INCLUDEH
#define INCLUDEH

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define LARGE  1000000			/* initial estimate for goal distance */

#define DISPLAY                 /* display what happens (in ASCII) */

//#define DEBUG                 /* whether debugging is on - debugging takes time but performs various checks */

#define INFORMEDSEARCH          /* use Manhattan distance rather than zero heuristics */

#define RUNS 1                  /* number of different runs */
//#define TIEBREAKING           /* tie breaking towards larger g-values (otherwise: smaller g-values) */

#define DIRECTIONS 4
static int dx[DIRECTIONS] = {1, 0, -1,  0};
static int dy[DIRECTIONS] = {0, 1,  0, -1};
static int reverse[DIRECTIONS] = {2, 3, 0, 1};

#ifdef INFORMEDSEARCH
	#define H(cell) (abs((cell)->y - mazegoal->y) + abs((cell)->x - mazegoal->x))
#else
	#define H(cell) 0
#endif
	
#endif
