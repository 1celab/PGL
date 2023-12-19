INCLUDEDIR='/home/michai/Projects/thesis/include'
SIZE=5

all: compile print

compile: printBinTrees.cpp
	g++ printBinTrees.cpp -g -O0 -I$(INCLUDEDIR)
	./a.out $(SIZE)


print: bfs.dot veb.dot
	dot bfs.dot -o bfs$(SIZE).svg -Tsvg
	dot veb.dot -o veb$(SIZE).svg -Tsvg

debug: printBinTrees.cpp
	g++ printBinTrees.cpp -O0 -g -I$(INCLUDEDIR)
	valgrind --leak-check=full ./a.out

clean: rm a.out bfs.dot veb.dot
