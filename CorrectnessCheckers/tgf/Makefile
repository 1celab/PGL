INCLUDEDIR='/home/andreas/Thesis/pgl/include'
BOOSTINCLUDEDIR='/opt/local/include'
BOOSTLIBDIR='/opt/local/lib'

all: compile

compile:
	g++ tgf.cpp -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -DNDEBUG -lboost_program_options


debug:
	g++ tgf.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options 
	
debug-noassert:
	g++ tgf.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options -DNDEBUG

print:
	dot graph.dot -Tsvg -o graph.svg
	dot forward.dot -Tsvg -o forward.svg
	dot backward.dot -Tsvg -o backward.svg
	dot nodes.dot -Tsvg -o nodes.svg

run:
	./a.out

valgrind:
	valgrind --leak-check=full ./a.out -s 10

profile: debug
	valgrind --tool=callgrind --callgrind-out-file=profile  ./a.out -s 10
	kcachegrind profile &

clean: 
	rm *.out *.dot *.svg 
