INCLUDEDIR=$(HOME)/Projects/pgl/include/
BOOSTINCLUDEDIR='/usr/local/include'
BOOSTLIBDIR='/usr/local/lib'

all: compile

compile:
	g++ $(file) -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -DNDEBUG -lboost_program_options

debug:
	g++ $(file) -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options 
	
debug-noassert:
	g++ $(file) -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options -DNDEBUG

run:
	./a.out

valgrind:
	valgrind --leak-check=full ./a.out -s 10

profile: debug
	valgrind --tool=callgrind --callgrind-out-file=profile  ./a.out -s 10
	kcachegrind profile &

clean: 
	rm *.out
