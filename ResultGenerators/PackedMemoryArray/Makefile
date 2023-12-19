INCLUDEDIR='/home/michai/Projects/pgl/include'
BOOSTINCLUDEDIR='/usr/local/include'
BOOSTLIBDIR='/usr/local/lib'
HEIGHT=5
FILE=packedMemoryArray.cpp

all: release

release: 
	g++ $(FILE) -O3 -ftree-vectorize -fomit-frame-pointer -fno-strict-aliasing -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -DNDEBUG -lboost_program_options

compile:
	g++ $(FILE) -O3 -fno-strict-aliasing -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options

debug:
	g++ $(FILE) -O0 -ggdb -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options

print: out.dot
	dot out.dot -o out.svg -Tsvg

valgrind:
	valgrind --leak-check=full ./a.out -s 10

profile: debug
	valgrind --tool=callgrind --callgrind-out-file=profile  ./a.out 
	kcachegrind profile &

clean: rm a.out out.dot
