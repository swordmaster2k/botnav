#ifndef DSTARLITEH
#define DSTARLITEH

#include "include.h"
#include "heap.h"
#include "maze.h"
#include "print.h"

int keymodifier;
cell goaltmpcell, oldtmpcell;

void initialize();

int computeshortestpath();

void updatemaze(cell *robot);

void initializecell(cell *thiscell);

void updatecell(cell *thiscell);

void updatekey(cell *thiscell);

void updaterhs(cell *thiscell);

int main(int argc, char *argv[]);

#endif
