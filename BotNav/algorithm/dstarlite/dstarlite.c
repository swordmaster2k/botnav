/*
 * D* Lite (modified version)
 * 
 * Heavily modified version of D* Lite example provided by the algorithms
 * authors. The modifications include large amounts of removed example
 * code, refactoring, and modularisation.
 * 
 * Modified By:
 * 	Joshua Michael Daly (ITB)
 * 
 * Original Authors: 
 * 	Maxim Likhachev (CMU) and Sven Koenig (USC)
 * 
 * Note: this version of D* Lite is optimized for grids 
 * It assumes, for example, that no cell can be a successor of itself.
 */ 

#include "dstarlite.h"

/*
 * *********************************************************************
 * Exposed to Python
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
 * update_occupany_grid
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
 * *********************************************************************
 * Internal to C
 * *********************************************************************
 */

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
 * Main
 * *********************************************************************
 */

/*
 *
 */
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
			/*
			 * Step 1: Plan. 
			 * compute shortest path based on current position 
			 */
			if (computeshortestpath())
			{
				break; // robot has reached its goal
			}

			mazegoal->trace = NULL;
			
			/* traverse the current path until we reach the goal or
			 * encounter an obstacle */
			do
			{
				#ifdef DISPLAY
				printrobotpath(stdout);
				printknownmaze(stdout);
				#endif
				
				/* store where we are moving from */
				mazegoal->searchtree->trace = mazegoal;
				
				/* simulated movement of robot */
				mazegoal = mazegoal->searchtree; 
			} while (mazestart != mazegoal && !mazegoal->searchtree->obstacle);
			
			/*
			 * Step 3: Update the map if necessary.
			 */
			if (mazestart != mazegoal)
			{
				/* encountered an obstacle update the cell */
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
