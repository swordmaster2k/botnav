#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define LARGE 1000000.0

typedef struct node node;

struct node
{
	double x, y;	// coordinates.
	double g;		// g-value.
};

/* 
* Test if two nodes are diagonal neigbours.
*
* Returns 1 if true, otherwise 0 false.
*/
int isdiagonalneighbour(node *s, node *sa)
{
	double xdistance = s->x - sa->x;
	double ydistance = s->y - sa->y;
	
	// Assume unit cells.
	if (xdistance == 1 || xdistance == -1)
	{
		if (ydistance == 1 || ydistance == -1)
		{
			return 1; // Is a diagonal neighbour.
		}
	}
	
	return 0; // Not a diagonal neighbour.
}

/*
 * ComputeCost algorithm from:
 * 
 *  "The Field D* Algorithm for Improved Path Planning and Replanning
 *   in Uniform and Non-Uniform Cost Environments"
 * 
 * top of page 7.
 */
double computecost(node s, node sa, node sb)
{
	double vs; 			// Cost of node s.
	double c = 0; 		// Traversal cost of center cell.
	double b = 500.0; 	// Traversal cost of bottom cell.
	node *s1, *s2;		// Edge nodes s1 and s2.
	
	if (isdiagonalneighbour(&s, &sa))
	{
		s1 = &sb;
		s2 = &sa;
	}
	else
	{
		s1 = &sa; 
		s2 = &sb;
	}
	
	if (fmin(c, b) == LARGE)
	{
		vs = LARGE; // No path.
	}
	else if (s1->g <= s2->g)
	{
		vs = fmin(c, b) + s1->g;
	}
	else
	{
		double f = s1->g - s2->g;
		
		if (f <= b)
		{
			if (c <= f)
			{
				vs = c * (sqrt(2)) + s2->g;
			}
			else
			{
				double y = fmin(f / sqrt(pow(c, 2) - pow(f, 2)), 1);

				vs = c * (sqrt(1 + pow(y, 2))) + f * (1 - y) + s2->g;
			}
		}
		else
		{
			if (c <= b)
			{
				vs = c * (sqrt(2)) + s2->g;
			}
			else
			{
				double x = 1 - fmin(b / sqrt((pow(c, 2) - pow(b, 2))), 1);
				
				vs = c * (sqrt(1 + pow(1 - x, 2))) + b * (x) + s2->g;
			}
		}
	}
	
	return vs;
}

void testcomputecost()
{
	node s, sa, sb;
	
	s.x = 1;
	s.y = 0.5;
	
	sa.x = 1;
	sa.y = 1;
	sa.g = 1;
	
	sb.x = 1;
	sb.y = 0;
	sb.g = 1.41;
	
	s.g = computecost(s, sa, sb);
	
	printf("s.g: %f\n", s.g);
}

int main()
{
	//double c = 0.4;
	//double f = 0.4;
	//double y = fmin(f / sqrt(pow(c, 2) - pow(f, 2)), 1);
	
	testcomputecost();
}
