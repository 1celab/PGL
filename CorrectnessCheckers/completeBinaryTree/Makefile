INCLUDEDIR='/home/michai/Projects/thesis/include'
LEDAINCLUDEDIR='/usr/local/LEDA/incl'
LEDALIBDIR='/usr/local/LEDA'

all: release

release: 
	g++ complete.cpp -O2  -ftree-vectorize -fno-rtti -fno-exceptions -fomit-frame-pointer -fno-strict-aliasing -I$(INCLUDEDIR) -DNDEBUG

zoom:
	g++ complete.cpp -g -O2 -I/opt/rotateright/Zoom/include -L/opt/rotateright/Zoom/lib -Wl,-rpath,/opt/rotateright/Zoom/lib -lRRZoomScript -lm -lrt -I$(INCLUDEDIR) -DNDEBUG


plain:
	g++ complete.cpp -O0 -I$(INCLUDEDIR) 

compile:
	g++ complete.cpp -g -fno-strict-aliasing -I$(INCLUDEDIR) -DQUEUESTATS

debug:
	g++ complete.cpp -O0 -g -fno-strict-aliasing -I$(INCLUDEDIR) -Wall -DQUEUESTATS	 

print:
	dot graph.dot -Tsvg -o graph.svg
	dot forward.dot -Tsvg -o forward.svg
	dot backward.dot -Tsvg -o backward.svg
	dot nodes.dot -Tsvg -o nodes.svg

run:
	./a.out

valgrind:
	valgrind --leak-check=full ./a.out

profile:   
	g++ complete.cpp -O0 -g -fno-strict-aliasing -I$(INCLUDEDIR) -Wall -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11 -DNDEBUG
	valgrind --tool=callgrind --callgrind-out-file=profile ./a.out
	kcachegrind profile &

clean: 
	rm *.out *.dot *.svg *.pdf
