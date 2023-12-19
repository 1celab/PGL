# Makefile for the tripleobjective_shortest_path example.
#
# Author:  Christos Nitsas
# Date:    2012
# 


CC=g++
CPPFLAGS=-Wall -Wextra -Werror -g -O2 -I$(PGLROOT)
CPPLIBS=-larmadillo


# Link everything and make tosp_example.out
tosp_example.out: main.cpp RandomGraphProblem.h RandomGraphProblem.cpp FloodVisitor.cpp FloodVisitor.h
	$(CC) $(CPPFLAGS) $(CPPLIBS) main.cpp -o $@



# Clean object files and executables
clean: 
	rm -f tosp_example.out

