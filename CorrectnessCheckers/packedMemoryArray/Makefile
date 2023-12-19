INCLUDEDIR='/home/michai/Projects/pgl/include'
HEIGHT=5
FILE=packedMemoryArray.cpp

all: release

release: 
	g++ $(FILE) -O2 -ftree-vectorize -fno-rtti -fno-exceptions -fomit-frame-pointer -fno-strict-aliasing -I$(INCLUDEDIR) -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11 -DNDEBUG

compile:
	g++ $(FILE) -O3 -fno-strict-aliasing -I$(INCLUDEDIR) -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11

debug:
	g++ $(FILE) -O0 -g -I$(INCLUDEDIR) -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11 -Wall 	

run:
	./a.out

valgrind:
	valgrind --leak-check=full ./a.out 

print: out.dot
	dot out.dot -o out.svg -Tsvg

profile:   
	g++ $(FILE) -O0 -g -I$(INCLUDEDIR) -I$(LEDAINCLUDEDIR) -L$(LEDALIBDIR) -lleda -lX11 -Wall -DNDEBUG
	valgrind --tool=callgrind --callgrind-out-file=profile --dump-after="void measureGraph<unsigned int, unsigned int>(DynamicGraph<unsigned int, unsigned int>&)" ./a.out
	kcachegrind profile &

clean: 
	rm *.out *.dot *.svg 
