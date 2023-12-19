INCLUDEDIR='/home/michai/Projects/THESIS/include'
HEIGHT=5

all: compile print

compile: pmaDensityTree.cpp
	g++ pmaDensityTree.cpp -g -O0 -I$(INCLUDEDIR)
	./a.out $(HEIGHT)


print: out.dot
	dot out.dot -o out.svg -Tsvg

debug: pmaDensityTree.cpp
	g++ pmaDensityTree.cpp -O0 -g -I$(INCLUDEDIR)
	valgrind --leak-check=full ./a.out

clean: rm a.out out.dot
