/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */

#include "include.h"
#include "maze.h"

cell **maze = NULL;
cell *mazegoal, *mazestart;
int mazeiteration = 0;

/*
 *
 */
void preprocessmaze()
{
    int x, y, d;
    int newx, newy;

    if (maze == NULL)
    {
		// allocate array space for y dimension, returns a pointer
		maze = (cell **)calloc(mazesize, sizeof(cell *));
		
		for (y = 0; y < mazesize; ++y)
		{
			// allocate array space for x dimension, returns a pointer
			maze[y] = (cell *)calloc(mazesize, sizeof(cell));
		}
		
		for (y = 0; y < mazesize; ++y)
		{
			for (x = 0; x < mazesize; ++x)
			{
				maze[y][x].x = x;
				maze[y][x].y = y;
				
				// setup this cells successors
				for (d = 0; d < DIRECTIONS; ++d)
				{
					newy = y + dy[d];
					newx = x + dx[d];
					maze[y][x].succ[d] = (newy >= 0 && newy < mazesize &&
						newx >= 0 && newx < mazesize) ? &maze[newy][newx] : NULL;
				}
			}
		 }
    }

#ifdef DEBUG
    assert(starty % 2 == 0);
    assert(startx % 2 == 0);
    assert(goaly % 2 == 0);
    assert(goalx % 2 == 0);
#endif

    mazestart = &maze[starty][startx];
    mazegoal = &maze[goaly][goalx];

    mazeiteration = 0;
}

/*
 *
 */
void postprocessmaze()
{
    int x, y;
    int d1, d2;
    cell *tmpcell;

	/* do the initial cell setup O(n^2)*/
    for (y = 0; y < mazesize; ++y)
    {
		for (x = 0; x < mazesize; ++x)
		{
			maze[y][x].generated = 0;
			maze[y][x].heapindex = 0;
			
			/* assign all the cell's successors */
			for (d1 = 0; d1 < DIRECTIONS; ++d1)
			{
				maze[y][x].move[d1] = maze[y][x].succ[d1];
			}
		}
	}
	
	/* now check every cell's movements for obstacles O(n^2) */
	for (y = 0; y < mazesize; ++y)
    {
		for (x = 0; x < mazesize; ++x)
		{
			for (d1 = 0; d1 < DIRECTIONS; ++d1)
			{
				if (maze[y][x].move[d1] && maze[y][x].move[d1]->obstacle)
				{
					tmpcell = maze[y][x].move[d1];
					
					for (d2 = 0; d2 < DIRECTIONS; ++d2)
					{
						if (tmpcell->move[d2])
						{
							tmpcell->move[d2] = NULL;
							tmpcell->succ[d2]->move[reverse[d2]] = NULL;
						}
					}
				}
			}
		}
	}
}

void xerror(char *msg)
{
	fprintf(stderr, "%s", msg);
	exit(1);
}

void establishmaze(FILE *file)
{
	int result;	
	float mazedimension, cellsize;
	 
	result = fscanf(file, "%f", &mazedimension);
	result = fscanf(file, "%f", &cellsize);

	mazesize = (int)ceil(mazedimension / cellsize);

	char	c;
	int	x, y, k;
	short foundrobot = 0, foundgoal = 0;

	k = fscanf(file, "%c", &c); // Consume random '\n'.

	fpos_t pos;
	result = fgetpos(file, &pos);

	/* do an initial run over the file looking for the robot and goal
	 * positions */
	for (y = mazesize - 1; y >= 0; --y)
	{
		for (x = 0; x < mazesize; ++x)
		{
			/*--- Get a character ---*/
			k = fscanf(file, "%c", &c);

			/*--- If it's the robot ---*/
			if (c == 'R')
			{
				goalx = x;
				goaly = y;
				foundrobot = 1;
			}

			/*--- If it's the goal ---*/
			if (c == 'G')
			{
				startx = x;
				starty = y;
				foundgoal = 1;
			}

			//fprintf(stderr, "%c", c);
		}
		
		if (foundrobot == 1 && foundgoal == 1)
				break;

		/*--- Scan til the end of line char ---*/
		while ( c != '\n' )
			k = fscanf(file, "%c", &c);

		//fprintf(stderr, "%c", '\n');
	}

	//fprintf(stderr, "\n\nRx%d\n", goalx);
	//fprintf(stderr, "Ry%d\n", goaly);
	//fprintf(stderr, "Gx%d\n", startx);
	//fprintf(stderr, "Gy%d\n\n", startx);
	
	result = fsetpos(file, &pos); /* rewind */ 
	
	preprocessmaze();

	/* run back over the file looking for obstacles and free space */
	for (y = mazesize - 1; y >= 0; --y)
	{
		for (x = 0; x < mazesize; ++x)
		{
			/*--- Get a character ---*/
			k = fscanf(file, "%c", &c);
			//fprintf(stderr, "%c", c);

			/*--- If an obstacle ---*/
			if (c == '#')
			{ 
				maze[y][x].obstacle = 1;

				//fprintf(stderr, "\nx:%d ", x);
				//fprintf(stderr, "y:%d\n", y);
			}

			/*--- If it's free space ---*/
			if (c == ' ')
			{
				maze[y][x].obstacle = 0;
			}

			/*--- If it's the end of a line ---*/
			if (c == '\n')
				break;

			//fprintf(stderr, "x%d", x);
		}

        if (y - 1 < 0) // End of grid.
            break;

		/*--- Scan til the end of line char ---*/
		while ( c != '\n' )
			k = fscanf(file, "%c", &c);

		//fprintf(stderr, "%c", '\n');
	}

	mazestart->obstacle = 0;
	mazegoal->obstacle = 0;
	
	postprocessmaze();
	
	fclose(file);
}
