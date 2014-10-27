/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */

#ifndef INCLUDEH
#define INCLUDEH

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define LARGE  1000000

#define DISPLAY                   /* display what happens (in ASCII)                                                     */
#define RANDOMIZESUCCS            /* randomize the order in which successors of a node are generated                     */
// #define RANDOMMAZE             /* whether the gridworld has random obstacles or is a maze created with dfs            */
#define WALLSTOREMOVE 4           /* number of walls to remove if RANDOMMAZE is NOT defined - infinite loop if too large */
// #define DEBUG                  /* whether debugging is on - debugging takes time but performs various checks          */
#define MAZEWIDTH 21              /* the width of the maze                                                               */
#define MAZEHEIGHT 21             /* the height of the mze                                                               */
#define MAZEDENSITY 0.25          /* percentage of blocked cells if RANDOMMAZE is defined                                */
// #define STARTCANBEBLOCKED      /* whether the goal cell of the robot can be blocked                                   */
#define RANDOMSTARTGOAL           /* whether the start and goal state are drawn randomly                                 */
#define STARTX 0                  /* x coordinate of the start cell                                                      */
#define STARTY 0                  /* y coordinate of the start cell                                                      */
#define GOALX 20                  /* x coordinate of the goal  cell                                                      */
#define GOALY 20                  /* y coordinate of the goal  cell                                                      */
#define INFORMEDSEARCH            /* use Manhattandistance rather than zero heuristics                                   */
#define RUNS 10                   /* number of different runs                                                            */
#define TIEBREAKING               /* tie breaking towards larger g-values (otherwise: smaller g-values)                  */

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
