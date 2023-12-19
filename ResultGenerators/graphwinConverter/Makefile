INCLUDEDIR='/home/michai/Projects/THESIS/include'
HEIGHT=5

all: compile

compile: ledagraphConverter.cpp
	g++ ledagraphConverter.cpp -O2 -I$(INCLUDEDIR) -DNDEBUG
	./a.out ../../CorrectnessCheckers/leda/graph ../../CorrectnessCheckers/leda/dynamicgraph

debug: ledagraphConverter.cpp
	g++ ledagraphConverter.cpp -O0 -g -I$(INCLUDEDIR) -Wall
	valgrind --leak-check=full ./a.out

profile: ledagraphConverter.cpp
	g++ ledagraphConverter.cpp -O0 -g -I$(INCLUDEDIR) -Wall
	valgrind --tool=callgrind ./a.out

clean: rm a.out out.dot
