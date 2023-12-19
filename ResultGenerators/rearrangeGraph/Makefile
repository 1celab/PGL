INCLUDEDIR=$(HOME)/Projects/pgl/include
BOOSTINCLUDEDIR=/usr/local/include
BOOSTLIBDIR=/usr/local/lib

all: compile

compile:
	g++ rearrangeGraph.cpp -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -DNDEBUG -lboost_program_options


debug:
	g++ rearrangeGraph.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options 
	
debug-noassert:
	g++ rearrangeGraph.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options -DNDEBUG

print:
	dot graph.dot -Tsvg -o graph.svg
	dot forward.dot -Tsvg -o forward.svg
	dot backward.dot -Tsvg -o backward.svg
	dot nodes.dot -Tsvg -o nodes.svg

run:
	./a.out

valgrind:
	valgrind --leak-check=full ./a.out -g 2

profile: debug
	valgrind --tool=callgrind --callgrind-out-file=profile  ./a.out -g 2
	kcachegrind profile &

clean: 
	rm *.out *.dot *.svg 
