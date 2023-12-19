INCLUDEDIR='/home/michai/Projects/pgl/include'
LEDAINCLUDEDIR='/usr/local/LEDA/incl'
LEDALIBDIR='/usr/local/LEDA'
GRAPHFILE='/home/michai/Projects/pgl/include/CorrectnessCheckers/leda/graph'

all: release

release: 
	g++ leda.cpp -O3 -ftree-vectorize -fno-rtti -fno-exceptions -fomit-frame-pointer -fno-strict-aliasing -I$(INCLUDEDIR) -DNDEBUG -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11

compile:
	g++ leda.cpp -g -O3 -fno-strict-aliasing -I$(INCLUDEDIR) -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11
debug:
	g++ leda.cpp -O0 -g -fno-strict-aliasing -I$(INCLUDEDIR) -Wall -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11	

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
	g++ leda.cpp -O0 -g -fno-strict-aliasing -I$(INCLUDEDIR) -Wall -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11 -DNDEBUG
	valgrind --tool=callgrind --callgrind-out-file=profile --dump-after="void measureGraph<unsigned int, unsigned int>(DynamicGraph<unsigned int, unsigned int>&)" ./a.out
	kcachegrind profile &

clean: 
	rm *.out *.dot *.svg *.pdf
