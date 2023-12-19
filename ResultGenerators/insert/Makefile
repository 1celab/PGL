INCLUDEDIR= $(HOME)/Projects/pgl/include
BOOSTINCLUDEDIR='/opt/local/include'
BOOSTLIBDIR='/opt/local/lib'

all: compile

compile:
	g++ insert.cpp -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -DNDEBUG -lboost_program_options

debug:
	g++ insert.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options -DMEMSTATS
	
debug-noassert:
	g++ insert.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options -DNDEBUG

stats:
	g++ insert.cpp -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -DNDEBUG -lboost_program_options -DMEMSTATS

run:
	./a.out

valgrind:
	valgrind --leak-check=full ./a.out -s 10

profile: debug
	valgrind --tool=callgrind --callgrind-out-file=profile  ./a.out -s 10
	kcachegrind profile &

clean: 
	rm *.out *.dot *.svg 
