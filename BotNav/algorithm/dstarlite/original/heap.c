/* D* Lite (final version) - Maxim Likhachev (CMU) and Sven Koenig (USC) */

#include "include.h"
#include "heap.h"

cell *heap[HEAPSIZE];
int heapsize = 0;
int keylength = 3;

/*
 *
 */ 
int keyless(cell *cell1, cell* cell2)
{
    int keyindex;

    for (keyindex = 0; keyindex < keylength; ++keyindex)
    {
		if (cell1->key[keyindex] < cell2->key[keyindex])
		{
			return 1;
		}
		else if (cell1->key[keyindex] > cell2->key[keyindex])
		{
			return 0;
		}
    }
    
    return 0;
}

/*
 *
 */ 
int testheap()
{
    int d;

    for (d = heapsize/2; d > 0; d--)
    {
		assert(!keyless(heap[2*d],heap[d]));
		
		if (2*d+1 <= heapsize)
		{
			assert(!keyless(heap[2*d+1],heap[d]));
		}
    }
}

/*
 *
 */ 
void percolatedown(int hole, cell *tmp)
{
    int child;

    if (heapsize != 0)
    {
		for (; 2*hole <= heapsize; hole = child)
		{
			child = 2*hole;
			
			if (child != heapsize && keyless(heap[child+1], heap[child]))
			{
				++child;
			}
			
			if (keyless(heap[child], tmp))
			{
				heap[hole] = heap[child];
				heap[hole]->heapindex = hole;
			}
			else
			{
				break;
			}
		}
		
		heap[hole] = tmp;
		heap[hole]->heapindex = hole;
    }
}

/*
 *
 */ 
void percolateup(int hole, cell *tmp)
{
    if (heapsize != 0)
    {
		for (; hole > 1 && keyless(tmp, heap[hole/2]); hole /= 2)
		{
			heap[hole] = heap[hole/2];
			heap[hole]->heapindex = hole;
		}
		
		heap[hole] = tmp;
		heap[hole]->heapindex = hole;
    }
}

/*
 *
 */ 
void percolateupordown(int hole, cell *tmp)
{
    if (heapsize != 0)
    {
		if (hole > 1 && keyless(tmp, heap[hole/2]))
		{
			percolateup(hole, tmp);
		}
		else
		{
			percolatedown(hole, tmp);
		}
    }
}

/*
 *
 */ 
void emptyheap(int length)
{
    int i;

    keylength = length;
    heapsize = 0;
}

/*
 *
 */ 
cell *topheap()
{
    if (heapsize == 0)
    {
		return NULL;
	}
		
    return heap[1];
}

/*
 *
 */ 
void deleteheap(cell *thiscell)
{
    if (thiscell->heapindex != 0 && thiscell->generated == mazeiteration)
    {
		percolateupordown(thiscell->heapindex, heap[heapsize--]);
		thiscell->heapindex = 0;
    }
}

/*
 *
 */ 
cell* popheap()
{
    cell *thiscell;

    if (heapsize == 0)
    {
	return NULL;
	}
	
    thiscell = heap[1];
    thiscell->heapindex = 0;
    percolatedown(1, heap[heapsize--]);
    
    return thiscell;
}

/*
 *
 */ 
void insertheap(cell *thiscell)
{
    int hole;

    if (thiscell->heapindex == 0 || thiscell->generated != mazeiteration)
    {
		thiscell->heapindex = 0;
#ifdef DEBUG
		assert(heapsize < HEAPSIZE-1);
#endif
		percolateup(++heapsize, thiscell);
    }
    else
    {
		percolateupordown(thiscell->heapindex, heap[thiscell->heapindex]);
	}
}

