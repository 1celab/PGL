#pgl header files
INCLUDEDIR= $(HOME)/Projects/pgl/include

#boost header files
BOOSTINCLUDEDIR='/usr/include/boost'

#boost bin files
BOOSTLIBDIR='/usr/lib'

all: compile

compile:
	g++ redTE.cpp -o redTE -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options -DNDEBUG

debug:
	g++ redTE.cpp -o redTE -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options -Wall -DMEMSTATS

queryGen:
	g++ queryGen.cpp -o queryGen -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options -DNDEBUG

clean: 
	rm -rf ./stats redTE queryGen
