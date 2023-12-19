INCLUDEDIR='/home/michai/Projects/THESIS/include'
SIZE=5

all: compile print

compile: numElementsInLeaf.cpp
	g++ numElementsInLeaf.cpp -g -O0 -I$(INCLUDEDIR)
	./a.out $(SIZE)

print: out.dot
	dot out.dot -o out.svg -Tsvg

debug: numElementsInLeaf.cpp
	g++ numElementsInLeaf.cpp -O0 -g -I$(INCLUDEDIR)
	valgrind --leak-check=full ./a.out

clean: rm a.out
