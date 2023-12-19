#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#include <LEDA/graph/graph.h>
#include <LEDA/graph/basic_graph_alg.h>
#include <LEDA/system/basic.h>

#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/adjacencyVectorImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
//#include <Structs/Arrays/nodeArray.h>
//#include <Structs/Arrays/edgeArray.h>
#include <Algorithms/basicGraphAlgorithms.h>
#include <Utilities/ledaComparisons.h>


//#define _SECURE_SCL 0

unsigned int n,iterations;
std::string graphfile;


struct edgeData
{
    unsigned int weight;
};

template<typename Vtype, typename Etype, template <typename vtype,typename etype> class GraphImplementation>
void measureGraph(DynamicGraph<Vtype,Etype,GraphImplementation>& G)
{
    typedef DynamicGraph<Vtype, Etype, GraphImplementation>     Graph;
    typedef typename Graph::NodeIterator                        NodeIterator;
    typedef typename Graph::EdgeIterator                        EdgeIterator;
    float T1, T2 ;
    
    NodeIterator v = G.chooseNode();
    G.addNodeProperty( "marked", false); 
    std::cout << "Running bfs with dynamicGraph\n";
//    T1=leda::used_time();
    EdgeIterator k;
    for( unsigned int i = 0; i < iterations; ++i)
    {
        bfs(G,v);
    }
//    T2=leda::used_time(T1);
    std::cout << T2 << std::endl;
    G.removeNodeProperty( "marked");
}

int main( int argc, char* argv[])
{   
    if( argc != 4 )
    {
        n = 100;
        iterations = 100;
        graphfile = std::string("/home/michai/Projects/pgl/include/CorrectnessCheckers/leda/graph");
    }
    else
    {
        n = atoi( argv[2]);
        iterations = atoi( argv[3]);    
        graphfile = std::string(argv[1]);
    }

    float T1, T2 ;
    
    leda::GRAPH< unsigned int, unsigned int> LG;
    std::cout << "Creating leda graph with " << n << " nodes and " << int(n*log(n)) << " edges\n";
    
    
    leda::random_simple_loopfree_graph(LG, n, int(n*log(n)));   
    leda::Make_Connected(LG);
    
    //std::ifstream in( graphfile.c_str());
    //LG.read( in );
    //in.close();

    leda::node u;

    leda::node_array<int> dists(LG,-1);  
    leda::node_array<unsigned int> ids(LG); 
    leda::node_array<leda::edge> pred(LG);
    leda::node_array<bool> reached(LG,false);

    std::cout << "Num Nodes: " << LG.number_of_nodes() << std::endl;

    //unsigned int i = 0;
    u = LG.choose_node(); 

    std::cout << "Running bfs with leda\n";
    T1=leda::used_time();
    //leda::edge e;
    for( unsigned int i = 0; i < iterations; i++)
    {
        //forall_nodes(u,LG){}
        //forall_edges(e,LG){}
        volatile leda::list<leda::node> LN2 = leda::BFS(LG,u,dists);
    
        //forall_nodes( u,LG)
        //{
        //    forall_out_edges( e, u) {}
        //}
    }
    T2=leda::used_time(T1);
    std::cout << T2 << std::endl;
    
    std::ofstream out(graphfile.c_str());
    LG.write(out);
    out.close();

    typedef DynamicGraph< SearchNode, edgeData, AdjacencyListImpl>  Graph;
    typedef Graph::NodeIterator                                             NodeIterator;
    typedef Graph::EdgeIterator                                             EdgeIterator;


    //std::ifstream in("dynamicgraph");
    Graph G;
    std::cout << "Reading dynamic graph with " << n << " nodes and " << int(n*floorLog2(n)) << " edges\n";
    //readGraphwin<Graph, SearchNodeData, unsigned int>(G,graphfile);    

    leda::node_array<Graph::NodeDescriptor> map(LG);
    graphFromLEDA(G,LG,map);

    G.printDot( "pmagraph");

    if( !compareGraphs(G,LG,map))
    {
        std::cout << "\nFailure!\n";
        return 0;
    }

    measureGraph(G);
    //in.close();
    return 0;
}
