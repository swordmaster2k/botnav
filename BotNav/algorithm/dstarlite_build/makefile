PYLIB = /usr/bin
PYINC = /usr/include/python3.4 

CFLAGS		= -O3 -g -fPIC -I$(PYINC) -shared -L$(PYLIB)python3.4
LFLAGS		= -O3 -g -fPIC -I$(PYINC) -shared -L$(PYLIB)python3.4
CC			= gcc

OBJ			= heap.o maze.o print.o dstarlite.o
OUT			= dstarlite_c.so

$(OUT):	$(OBJ)
		$(CC) $(LFLAGS) -o $(OUT) $(OBJ) 

.cc.o:
		$(CC) $(CFLAGS) $<
		
clean:
	rm -f $(OBJ) $(OUT)
