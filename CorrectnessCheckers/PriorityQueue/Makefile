INCLUDEDIR='/home/michai/Projects/pgl/include'
LEDAINCLUDEDIR='/usr/local/LEDA/incl'
LEDALIBDIR='/usr/local/LEDA'


all: release

release: 
	g++ priorityQueue.cpp -O3 -ftree-vectorize -fno-rtti -fno-exceptions -fomit-frame-pointer -fno-strict-aliasing -I$(INCLUDEDIR) -DNDEBUG

plain:
	g++ priorityQueue.cpp -O0 -ftree-vectorize -fno-rtti -fno-exceptions -fomit-frame-pointer -fno-strict-aliasing -I$(INCLUDEDIR) 

compile:
	g++ priorityQueue.cpp -g -fno-strict-aliasing -I$(INCLUDEDIR) -DQUEUESTATS

stats:
	g++ priorityQueue.cpp -O0 -ftree-vectorize -fno-rtti -fno-exceptions -fomit-frame-pointer -fno-strict-aliasing -I$(INCLUDEDIR) -DQUEUESTATS

debug:
	g++ priorityQueue.cpp -O0 -g -fno-strict-aliasing -I$(INCLUDEDIR) -Wall	 

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
	g++ priorityQueue.cpp -O0 -g -fno-strict-aliasing -I$(INCLUDEDIR) -Wall -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11 -DNDEBUG
	valgrind --tool=callgrind --callgrind-out-file=profile --dump-before="comparePQ(PriorityQueue<unsigned int, double, VebStorage>&)" --dump-before="void compareNull()" ./a.out
	kcachegrind profile &

clean: 
	rm *.out *.dot *.svg *.pdf
