INCLUDEDIR='/home/michai/Projects/pgl/include'
BOOSTINCLUDEDIR='/usr/local/include'
BOOSTLIBDIR='/usr/local/lib'

all: compile

compile:
	g++ concepts.cpp -g -O0 -fno-strict-aliasing -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -DNDEBUG


debug:
	g++ concepts.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall
	
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
	./a.out
	gprof -ab > profile

clean: 
	rm *.out *.dot *.svg 
