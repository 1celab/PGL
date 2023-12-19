INCLUDEDIR='/home/michai/Projects/pgl_old/include'
BOOSTINCLUDEDIR='/opt/local/include'
BOOSTLIBDIR='/opt/local/lib'

all: compile

compile:
	g++ multiCriteria.cpp -O2 -fno-strict-aliasing -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -DNDEBUG -lboost_program_options


debug:
	g++ multiCriteria.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options

test:
	g++ multiCriteriaFirst.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options
	
print:
	dot graph.dot -Tsvg -o graph.svg
	dot forward.dot -Tsvg -o forward.svg
	dot backward.dot -Tsvg -o backward.svg
	dot nodes.dot -Tsvg -o nodes.svg

run:
	./a.out

valgrind:
	valgrind --leak-check=full ./a.out -s 10 -g 1 -d 2

profile: 
	./a.out
	gprof -ab > profile

clean: 
	rm *.out *.dot *.svg 
