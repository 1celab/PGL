INCLUDEDIR='/home/michai/Projects/THESIS/include'
SIZE=5

all: compile print

compile: searchTree.cpp
	g++ searchTree.cpp -g -O0 -I$(INCLUDEDIR)
	./a.out $(SIZE)

print: out.dot
	dot out.dot -o out.svg -Tsvg

debug: searchTree.cpp
	g++ searchTree.cpp -O0 -g -I$(INCLUDEDIR)
	valgrind --leak-check=full ./a.out

clean: rm a.out
