INCLUDEDIR='/home/michai/Projects/pgl/include'
BOOSTINCLUDEDIR='/opt/local/include'
BOOSTLIBDIR='/opt/local/lib'

all: compile

compile: 
	g++ mersenneTwister.cpp -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options

debug: 
	g++ mersenneTwister.cpp -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options -Wall
	
valgrind: 
	valgrind --leak-check=full ./a.out

clean: 
	rm *.out

