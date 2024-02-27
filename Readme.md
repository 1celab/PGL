# PGL: A library of efficient graph structures and algorithms <br> for large scale networks

## Requirements
1) Install git-client:
```sh
sudo apt-get install git
```
   
2) Install GNU C++ Compiler
```sh
sudo apt-get install g++
```

3) Install third-party libraries:

 - BOOST Library
 
```sh
sudo apt-get install libboost-all-dev libboost-program-options-dev
```

4) Install doxygen
```sh
sudo apt-get install doxygen
```

## Setup

 - Download PGL Library (trunk):

```sh
git clone https://vernon.ceid.upatras.gr:8000/root/PGL.git
(suggested root path: "~/Projects/")
```

## Introduction

 This is a library of algorithms and data structures. Its main goal is to provide structures and algorithms optimized for large-scale graphs. The main building block is a dynamic graph structure, called Packed-memory Graph, that combines the static forward star graph structure with dynamic arrays (packed-memory arrays). It achieves a good cache efficiency during both normal static graph operations, like algorithm queries (which usually operate on a static graph layout), and updates of the layout of the graph. The library also contains several algorithms for shortest path computations on large-scale networks with either single or multi criteria edge costs. 

## Source Code Documentation


## Getting Started

To test the library, you can try the example that is provided in it. You can compile the example application using the Makefile it contains. You will need: 1) the PGL library, 2) the Boost library and 3) a DIMACS10 map. You should check the following:

* The Makefile contains the correct path to the pgl/include folder
* The Makefile contains the correct path to the boost header files folder (you have to install boost first, speciffically libboost-dev and libboost-program-options)
* The example.cpp file contains a correct path to a map and a coordinates file (Lines 155-156). Available maps can be found [here](http://www.cc.gatech.edu/dimacs10/archive/streets.shtml).


## Related Publications/Talks

*   A. Paraskevopoulos and C. Zaroliagis, **"Improved Alternative Route Planning"**  
    _in Proc. 13th Workshop on Algorithmic Approaches for Transportation Modeling, Optimization, and Systems_ - ATMOS 2013, OASIcs Series, to appear. [![](../../pub/pdf_icon.gif)](../../pub/conf/C64-ATMOS2013-AltRoutes.pdf)

*   C. Zaroliagis, **"Engineering Multiobjective Shortest Path Heuristics"**  
    _in 22nd International Conference on Multiple Criteria Decision Making_ - MCDM 2013 [invited talk].

*   G. Mali, P. Michail, A. Paraskevopoulos, and C. Zaroliagis, **["A New Dynamic Graph Structure for Large-Scale Transportation Networks"](http://link.springer.com/chapter/10.1007%2F978-3-642-38233-8_26)**  
    _in Algorithms and Complexity_ - CIAC 2013, Lecture Notes in Computer Science Vol.**7878** (Springer 2013), pp. 312-323. [![](../../pub/pdf_icon.gif)](../../pub/conf/C63-CIAC2013-new-dyn-graph.pdf)

*   G. Mali, P. Michail, and C. Zaroliagis, **["Faster Multiobjective Heuristic Search in Road Maps"](http://doi.searchdl.org/03.CSS.2012.3.16_1) **,  
    _in Proc. Int. Conf. on Advances in Information and Communication Technologies_ - ICT 2012, Vol. 3, pp. 67-72. [![](../../pub/pdf_icon.gif)](../../pub/conf/C62-ICT2012-faster-mobj-search.pdf)
