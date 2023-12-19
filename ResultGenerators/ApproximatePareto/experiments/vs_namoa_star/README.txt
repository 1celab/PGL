This folder contains the code for the experiments we did to compare 
Chord and PGEN with NAMOA*. We used the PGL library's NAMOA* implementation.
We also used PGL's implementation of the A* and (single objective) Dijkstra 
algorithms for our COMB routine. 

In this folder we use the words "Chord" and "PGEN" interchangeably. Both 
algorithms essentially do the same thing but for different numbers of 
objective functions each (Chord for 2, PGEN for 3+. PGEN can also work 
for 2 objective functions but Chord is simpler).

PGEN uses an external program called "qconvex", part of "Qhull" 
(http://www.qhull.org/), to compute the convex hull of sets of points.
In order to use "qconvex", we create simple text files called 
"qconvex-input.txt" and "qconvex-output.txt" where we write qconvex's 
input and read qconvex's output from. The files can be safely deleted after 
the program execution. (We might add code to auto-delete them just before 
the program terminates, at some point.)

What is the COMB routine:
------------------------------
The COMB routine is a routine that optimizes (minimizes) linear combinations 
of the objective functions. We give it a set of weights w_{i} and it 
optimizes (minimizes) the combined objective:
\f$ w_{1} f_{1} + w_{2} f_{2} + ... + w_{n} f_{n} \f$,
where f_{i} is the ith objective function. The COMB routine is specific to 
the underlying problem (in this case Multicriteria Shortest Paths). Chord 
and PGEN use the COMB routine as a black-box to generate Pareto optimal 
points (on the convex hull of the Pareto set).

Requirements:
------------------------------
1. The Armadillo C++ linear algebra library is required (chiefly) for some 
   linear algebra operations inside the Facet class (and some less important 
   operations elsewhere). The earliest version I've tried is 3.6.1 and it 
   worked. 
   (you can find it on http://arma.sourceforge.net/)
2. The Qhull program (specifically "qconvex") is required by PGEN to compute 
   the convex hull of sets of points. 
   (you can find it on http://www.qhull.org/)
3. The PGL library is required specifically for the experiments vs NAMOA*.
   It is not needed for the general Pareto set approximator (i.e. Chord 
   and PGEN implementation). 
   (you can find it on 
   http://www.ceid.upatras.gr/faculty/zaro/software/pgl/index.html)

Options:
------------------------------
The following are options that can be modified by editing the respective 
#defines inside the main.cpp file.

1. The option to use 2 or 3 objectives is currently being handled via the 
   preprocessor #define called OPTION_NUM_OBJECTIVES (defined in "main.cpp"). 
   The places where OPTION_NUM_OBJECTIVES affects the code are labeled with 
   CHANGE-HERE (in comments). If you try to define OPTION_NUM_OBJECTIVES to 
   a value that is not 2 or 3, things will probably not work (I've included 
   some assertions in multiple places).
2. OPTION_QUERIES_FILE_NAME (currently "queries.txt") refers to the name of 
   the file containing the s-t multicriteria shortest paths queries to be 
   run. 
3. OPTION_QUERIES_FILE_PATH (currently "./") refers to the path of the 
   queries file.
4. OPTION_GRAPH_FILES_PATH (currently "./data/graphs/") refers to the folder 
   where the graph files will be searched in.
5. OPTION_DEFAULT_GRAPH (currently "NY") refers to the default graph to be 
   used if no command line arguments are given.
6. OPTION_RESULTS_FOLDER_NAME (currently "results") refers to the folder 
   where all the computed Pareto (and approximate Pareto) sets will be 
   saved. The folder will be created if it does not exist.

Compiling:
------------------------------
To compile the program that does the experiments (vns_experiment.out) you 
just have to type "make" in the command line (a Makefile has been included).

The Armadillo library root folders (include and lib folders) might need to 
be edited inside the Makefile.

The Makefile expects that a global shell variable named PGLROOT exists and 
that it points to PGL's root folder. This can also be edited inside the 
Makefile, if necessary.

Running:
------------------------------
The line:
vns_experiment.out map-name
will run the experiments on the graph identified by map-name. See 
data/graphs/GRAPH-INFO.txt for all possible map-names (e.g. NY for the 
New York DIMACS 9 graph, or luxembourg for the Luxembourg DIMACS 10 graph).

The graph type (DIMACS 9 or DIMACS 10) is recognized automatically, based 
on the map-name identifier. (We have hardcoded all DIMACS 9 and DIMACS 10 
map-name identifiers.)

The program currently searches for the graph files inside the ./data/graphs/ 
folder (OPTION_GRAPH_FILES_PATH), for a queries file named "queries.txt" 
(OPTION_QUERIES_FILE_NAME) in the current folder (OPTION_QUERIES_FILE_PATH),
runs the experiments on the graph whose map-name was given as a command 
line argument (NY if no arguments were given (OPTION_DEFAULT_GRAPH)) and 
prints time-related results on screen and creates a folder named "results" 
(OPTION_RESULTS_FOLDER_NAME) where it will save all the computed Pareto (and 
approximate Pareto) sets.

