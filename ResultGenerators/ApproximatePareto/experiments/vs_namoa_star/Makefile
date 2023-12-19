# Makefile for the experiment vs PGL's NAMOA*.
# 
# Author: Christos Nitsas
# Date:   2013
#


CC=g++
#ARMADILLOROOT=${HOME}/usr/
ARMADILLOROOT=/usr/local/
PGLINCLUDEDIR=$(PGLROOT)
CPPFLAGS=-Wall -L$(ARMADILLOROOT)/lib -I$(ARMADILLOROOT)/include -I$(PGLINCLUDEDIR) -Wno-deprecated -m64
#CPPFLAGS=-Wall -L$(ARMADILLOROOT)/lib -I$(ARMADILLOROOT)/include -I$(PGLINCLUDEDIR) -Wno-deprecated -Wno-unused-but-set-variable
CPPLIBS=-larmadillo


all: compile

compile: main.cpp AStarDijkstra.h MultiobjectiveSpOnPmgProblem.h Hyperplane.h experiments_vs_namoa_star_common.h experiments_vs_namoa_star_utility.h 
	g++ main.cpp -o vns_experiment.out -O2 $(CPPFLAGS) $(CPPLIBS)

debug: main.cpp AStarDijkstra.h MultiobjectiveSpOnPmgProblem.h Hyperplane.h experiments_vs_namoa_star_common.h experiments_vs_namoa_star_utility.h 
	g++ main.cpp -o vns_experiment.out -O0 -g $(CPPFLAGS) $(CPPLIBS)

clean:
	rm vns_experiment.out
