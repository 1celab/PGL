INCLUDEDIR='/home/michai/Projects/THESIS/include'
HEIGHT=5
ITERATIONS=10

all: compile

compile: searchTree.cpp
	g++ searchTree.cpp -g -O0 -lrt -I$(INCLUDEDIR)
	./a.out $(HEIGHT) $(ITERATIONS)

debug: searchTree.cpp
	g++ searchTree.cpp -O0 -g -lrt -I$(INCLUDEDIR)
	valgrind --leak-check=full ./a.out

clean: rm a.out
