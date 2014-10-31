CFLAGS		= -O3
LFLAGS		= -O3
CC		= gcc

OBJ		= heap.o maze.o dstarlite.o 
dstarlite:	$(OBJ)
		$(CC) $(LFLAGS) -o dstarlite $(OBJ) 

.cc.o:
		$(CC) $(CFLAGS) -c $<
		
clean:
	rm -f dstarlite core
