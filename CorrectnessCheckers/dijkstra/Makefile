INCLUDEDIR='/Users/panosmichail/Projects/pgl/include'
BOOSTINCLUDEDIR='/opt/local/include'
BOOSTLIBDIR='/opt/local/lib'

all: compile

compile:
	g++ dijkstra.cpp -O2 -fno-strict-aliasing -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -DNDEBUG


debug:
	g++ dijkstra.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall
	
print:
	dot graph.dot -Tsvg -o graph.svg
	dot forward.dot -Tsvg -o forward.svg
	dot backward.dot -Tsvg -o backward.svg
	dot nodes.dot -Tsvg -o nodes.svg

run:
	./a.out

valgrind:
	valgrind --leak-check=full ./a.out

profile: debug
	valgrind --tool=callgrind --callgrind-out-file=profile  ./a.out 
	kcachegrind profile &

clean: 
	rm *.out *.dot *.svg 
