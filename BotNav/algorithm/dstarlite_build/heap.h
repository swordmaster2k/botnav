/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */

#ifndef HEAPH
#define HEAPH

#include "maze.h"

#define HEAPSIZE 100000
cell *heap[HEAPSIZE];
int heapsize;
int keylength;

void emptyheap(int length);
int testheap();
cell* popheap();
cell *topheap();
void deleteheap(cell *thiscell);
void insertheap(cell *thiscell);

#endif
