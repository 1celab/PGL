#pgl header files
INCLUDEDIR= $(HOME)/Projects/pgl/include

#boost header files
BOOSTINCLUDEDIR='/usr/include/boost'

#boost bin files
BOOSTLIBDIR='/usr/lib'

all: compile

compile:
	g++ astarpre.cpp -o pre -O3 -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR)-Wall -DNDEBUG -lboost_program_options

debug:
	g++ astarpre.cpp -o pre -O0 -g -I$(INCLUDEDIR) -I$(BOOSTINCLUDEDIR) -L$(BOOSTLIBDIR) -Wall -lboost_program_options

run :
	./pre

clean: 
	rm pre 
