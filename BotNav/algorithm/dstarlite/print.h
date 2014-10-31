#ifndef PRINTH
#define PRINTH

#include "include.h"
#include "maze.h"

/*
 * Prints the entire path of the robot.
 * 
 * @param output file to output print data to
 */
void printrobotpath(FILE *output);

/*
 * Prints the entire maze.
 * 
 * @param output file to output print data to
 */ 
void printknownmaze(FILE *output);

/*
 * Prints only the known parts of the maze.
 * 
 * @param output file to output print data to
 */
void printactualmaze(FILE *output);

#endif
