#include "print.h"

/*
 * *********************************************************************
 * Print Functions
 * *********************************************************************
 */

/*
 * Prints the entire path of the robot.
 * 
 * @param output file to output print data to
 */ 
void printrobotpath(FILE *output)
{
    cell *tmpcell;
    tmpcell = mazegoal->searchtree;
	
	fprintf(output, "%s", "\n");
	fprintf(output, "%s", "path: ");
	
	for (tmpcell = mazegoal->searchtree; tmpcell != NULL; tmpcell = tmpcell->searchtree)
	{
			fprintf(output, "[%i, %i] ", tmpcell->x, tmpcell->y);
			
			if (tmpcell != mazestart)
				fprintf(output, "%s", "-> ");
	}
	
	fprintf(output, "\n");
}

/*
 * Prints the entire maze.
 * 
 * @param output file to output print data to
 */
void printactualmaze(FILE *output)
{
    int x, y;
    
    fprintf(output, "\n");
    
    for (y = mazesize - 1; y >= 0; --y)
    {
		fprintf(output, "#");
		
		for (x = 1; x < mazesize - 1; ++x)
		{
			if (mazegoal == &maze[y][x])
			{
				fprintf(output, "R");
			}
			else if (mazestart == &maze[y][x])
			{
				fprintf(output, "G");
			}
			else if (maze[y][x].obstacle)
			{
				fprintf(output, "#");
			}
			else
			{
				fprintf(output, " ");
			}
		}
		
		fprintf(output, "#\n");
    }
	
    fprintf(output, "\n\n\n");
}

/*
 * Prints only the known parts of the maze.
 * 
 * @param output file to output print data to
 */
void printknownmaze(FILE *output)
{
    int x, y, d;
    static char **display = NULL;
    cell *tmpcell;

    if (display == NULL)
    {
		display = (char **)calloc(mazesize, sizeof(char *));
		
		for (y = 0; y < mazesize; ++y)
		{
			display[y] = (char *)calloc(mazesize, sizeof(char));
		}
    }
    
    for (y = mazesize - 1; y >= 0; --y)
    {
		for (x = 1; x < mazesize - 1; ++x)
		{
			display[y][x] = '#';
		
			for (d = 0; d < DIRECTIONS; ++d)
			{
				if (maze[y][x].move[d])
				{
					display[y][x] = ' ';
				}
			}
		}
	}	
	
    for (tmpcell = mazegoal; tmpcell != mazestart; tmpcell = tmpcell->searchtree)
    {
		display[tmpcell->y][tmpcell->x] = '.';
	}
		
    display[mazestart->y][mazestart->x] = 'G';
    display[mazegoal->y][mazegoal->x] = 'R';
    
    for (y = mazesize - 1; y >= 0; --y)
    {
		fprintf(output, "#");
		
		for (x = 0; x < mazesize; ++x)
		{
			fprintf(output, "%c", display[y][x]);
		}
			
		fprintf(output, "#\n");
    }
		
    fprintf(output, "\n\n\n");
}
