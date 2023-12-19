#pgl header files
INCLUDEDIR= $(HOME)/Projects/pgl/include

#boost header files
BOOSTINCLUDEDIR='/usr/include/boost'

#boost bin files
BOOSTLIBDIR='/usr/lib'

all: compile

compile:
	g++ dynTM.cpp -std=c++11 -o dynTM -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options -DNDEBUG -fopenmp -lpthread

debug:
	g++ dynTM.cpp -std=c++11 -o dynTM -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options -Wall -DMEMSTATS -fopenmp -lpthread

query:
	g++ queryGen.cpp -std=c++11 -o queryGen -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options -DNDEBUG

dquery:
	g++ queryGen.cpp -std=c++11 -o queryGen -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -lboost_program_options -Wall -DMEMSTATS

clean: 
	rm -rf ./stats dynTm queryGen
