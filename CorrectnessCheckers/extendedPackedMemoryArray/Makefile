INCLUDEDIR='/home/michai/Projects/THESIS/include'
HEIGHT=5

all: compile print

compile: extendedPackedMemoryArray.cpp
	g++ extendedPackedMemoryArray.cpp -g -O0 -I$(INCLUDEDIR)
	./a.out $(HEIGHT)


print: out.dot
	dot out.dot -o out.svg -Tsvg

debug: extendedPackedMemoryArray.cpp out.dot
	g++ extendedPackedMemoryArray.cpp -O0 -g -I$(INCLUDEDIR) -Wall
	valgrind --leak-check=full ./a.out
	dot out.dot -o out.svg -Tsvg

clean: 
	rm *.out *.dot *.svg 
