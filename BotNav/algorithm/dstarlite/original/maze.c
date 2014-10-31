/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */

#include "include.h"
#include "maze.h"

cell **maze = NULL;
cell *mazestart, *mazegoal;
int mazeiteration = 0;

int goaly = GOALY;
int goalx = GOALX;
int starty = STARTY;
int startx = STARTX;

/*
 *
 */
void preprocessmaze()
{
    int x, y, d;
    int newx, newy;

    if (maze == NULL)
    {
		maze = (cell **)calloc(MAZEHEIGHT, sizeof(cell *));
		
		for (y = 0; y < MAZEHEIGHT; ++y)
		{
			maze[y] = (cell *)calloc(MAZEWIDTH, sizeof(cell));
		}
		
		for (y = 0; y < MAZEHEIGHT; ++y)
		{
			for (x = 0; x < MAZEWIDTH; ++x)
			{
				maze[y][x].x = x;
				maze[y][x].y = y;
				
				for (d = 0; d < DIRECTIONS; ++d)
				{
					newy = y + dy[d];
					newx = x + dx[d];
					maze[y][x].succ[d] = (newy >= 0 && newy < MAZEHEIGHT && newx >= 0 && newx < MAZEWIDTH) ? &maze[newy][newx] : NULL;
				}
			}
		 }
    }
#ifdef RANDOMSTARTGOAL
    goaly = (random() % ((MAZEHEIGHT + 1) / 2)) * 2;
    goalx = (random() % ((MAZEWIDTH + 1) / 2)) * 2;
    
    while (1)
    {
		starty = (random() % ((MAZEHEIGHT + 1) / 2)) * 2;
		startx = (random() % ((MAZEWIDTH + 1) / 2)) * 2;
		
        if (startx != goalx || starty != goaly)
        {
            break;
		}
    }
    
    mazestart = &maze[starty][startx];
    mazegoal = &maze[goaly][goalx];
#else
#ifdef DEBUG
    assert(STARTY % 2 == 0);
    assert(STARTX % 2 == 0);
    assert(GOALY % 2 == 0);
    assert(GOALX % 2 == 0);
#endif
    mazestart = &maze[STARTY][STARTX];
    mazegoal = &maze[GOALY][GOALX];
#endif
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

    for (y = 0; y < MAZEHEIGHT; ++y)
    {
		for (x = 0; x < MAZEWIDTH; ++x)
		{
			maze[y][x].generated = 0;
			maze[y][x].heapindex = 0;
			 
			for (d1 = 0; d1 < DIRECTIONS; ++d1)
			{
				maze[y][x].move[d1] = maze[y][x].succ[d1];
			}
		}
	}
	
    for (d1 = 0; d1 < DIRECTIONS; ++d1)
    {
		if (mazegoal->move[d1] && mazegoal->move[d1]->obstacle)
		{
			tmpcell = mazegoal->move[d1];
			
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

/*
 *
 */
void newrandommaze()
{
    int d1, d2;
    int x, y;
    int newx, newy;
    cell *tmpcell;

    preprocessmaze();
    for (y = 0; y < MAZEHEIGHT; ++y)
    {
		for (x = 0; x < MAZEWIDTH; ++x)
		{
			maze[y][x].obstacle = (random() % 10000 < 10000 * MAZEDENSITY);
		}
	}
	
    mazegoal->obstacle = 0;
#ifndef STARTCANBEBLOCKED
    mazestart->obstacle = 0;
#endif
    postprocessmaze();
}

/*
 *
 */
void newdfsmaze(int wallstoremove)
{
    int d, dtmp;
    int x, y;
    int newx, newy;
    int randomnumber;
    cell *tmpcell;
    int permute[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    int permutetmp;

    preprocessmaze();
  #ifdef DEBUG
    assert(MAZEWIDTH % 2 == 1);
    assert(MAZEHEIGHT % 2 == 1);
  #endif
    for (y = 0; y < MAZEHEIGHT; ++y)
    {
		for (x = 0; x < MAZEWIDTH; ++x)
		{
			maze[y][x].obstacle = 1;
			maze[y][x].dfsx = 0;
			maze[y][x].dfsy = 0;
		}
	}
	
    x = 0;
    y = 0;
    maze[y][x].dfsx = -1;
    maze[y][x].dfsy = -1;
    
    while (1)
    {
		if (maze[y][x].obstacle)
		{
			maze[y][x].obstacle = 0;
		}
		
		for (d = 0; d < DIRECTIONS - 1; ++d)
		{
			randomnumber = random() % (DIRECTIONS-d);
			permutetmp = permute[randomnumber];
			permute[randomnumber] = permute[DIRECTIONS-1-d];
			permute[DIRECTIONS - 1 - d] = permutetmp;
		}
		
		for (dtmp = 0; dtmp < DIRECTIONS; ++dtmp)
		{
			d = permute[dtmp];
			newx = x + 2 * dx[d];
			newy = y + 2 * dy[d];
			
			if (maze[y][x].succ[d] != NULL && maze[newy][newx].obstacle)
			{
				if (maze[y + dy[d]][x + dx[d]].obstacle)
				{
					maze[y + dy[d]][x + dx[d]].obstacle = 0;
				}
				
				maze[newy][newx].dfsx = x;
				maze[newy][newx].dfsy = y;
				x = newx;
				y = newy;
				
				break;
			}
		}
		
		if (dtmp == DIRECTIONS)
		{
			if (maze[y][x].dfsx == -1)
			{
				break;
			}
			
			newx = maze[y][x].dfsx;
			newy = maze[y][x].dfsy;
			x = newx;
			y = newy;
		}
    }
    
    while (wallstoremove)
    {
		newx = random() % MAZEWIDTH;
		newy = random() % MAZEHEIGHT;
		
		if (maze[newy][newx].obstacle)
		{
			maze[newy][newx].obstacle = 0;
			--wallstoremove;
		}
    }
    
    mazegoal->obstacle = 0;
#ifndef STARTCANBEBLOCKED
    mazestart->obstacle = 0;
#endif
    postprocessmaze();
}
