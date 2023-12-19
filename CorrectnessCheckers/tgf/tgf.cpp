#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <Algorithms/basicGraphAlgorithms.h>
#include <Algorithms/ShortestPath/dijkstra.h>
#include <Algorithms/ShortestPath/aStarDijkstra.h>
#include <Algorithms/ShortestPath/aStarLmk.h>
#include <Algorithms/ShortestPath/bidirectionalDijkstra.h>
#include <boost/utility/enable_if.hpp>
#include <iomanip>
#include <Utilities/geographic.h>
#include <Utilities/graphGenerators.h>
#include <Utilities/timer.h>
#include <boost/program_options.hpp>


//namespace po = boost::program_options;

struct node: DefaultGraphItem
{
	node():x(0),y(0),dist(std::numeric_limits<unsigned int>::max()),pqitem(0),pred(0),timestamp(0)
	{	
	}

    unsigned int x,y;
    unsigned int dist,distBack;
    unsigned int pqitem,pqitemBack;
    void* pred;
    void* succ;
    unsigned int timestamp;
    
    bool marked;

    struct LandmarkData
    {
        unsigned int distanceToLandmark;
        unsigned int distanceFromLandmark;
    };

    std::vector<LandmarkData> landmark; 
};


struct edge: DefaultGraphItem
{
	edge():weight(0)
	{	
	}

    unsigned int weight;
};


int main( int argc, char* argv[])
{
    MersenneTwister gen;  

    std::string basePath = "/home/andreas/Thesis/Leda/Graphs/";
    //std::string basePath = "/Users/panosmichail/Projects/Graphs/DIMACS10/";
    std::string map = "";

    typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   pmaGraph;
    typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
    typedef Graph::NodeIterator                                NodeIterator;
    typedef Graph::EdgeIterator                                EdgeIterator;
    typedef Graph::InEdgeIterator                              InEdgeIterator;
    typedef Graph::NodeDescriptor                              NodeDescriptor;

    
    Graph G;
    pmaGraph pmaG;

    //TGFReader<Graph> reader(basePath + map + "nodes.dat",
    //                        basePath + map + "edges.dat");
                            
                            
    DIMACS10Reader<Graph> reader(basePath + "luxembourg.osm.graph",
                                 basePath + "luxembourg.osm.xyz");

    G.read(&reader);

    /* === fill the weights of the edges === */
    for(NodeIterator v, u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {   
        for(EdgeIterator e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
        {
            v = G.target(e);
            e->weight = euclideanDistance( u->x, u->y, v->x, v->y);
        }
            
        //necessary for Backward Dijkstra   
        for(InEdgeIterator e = G.beginInEdges(u), lastedge = G.endInEdges(u); e != lastedge; ++e)
        {
            v = G.source(e);
            e->weight = euclideanDistance( u->x, u->y, v->x, v->y);
        }
    }

    LandmarkGenerator<Graph> landmarkGen(G, reader.getIds());
    
    //select 10 landmarks in the map
    landmarkGen.farthestLandmarkSelection(10);
    
    landmarkGen.writeLandmarkDistances(basePath + "landmarks.dat");
    //G.memUsage();

    landmarkGen.loadLandmarkDistances(basePath + "landmarks.dat");

    unsigned int timestamp = 1;
    AStarLmk<Graph> aStarLmk(G, &timestamp);
    Dijkstra<Graph> dijkstra(G, &timestamp);

    NodeIterator source, target;

    do {source = G.chooseNode();} 
    while( (target = G.chooseNode()) == source);
        
    aStarLmk.runQuery(source, target);
    std::cout << "\ndist :" << target->dist << std::endl;
    
    dijkstra.runQuery(source, target);
    std::cout << "\ndist :" << target->dist << std::endl;

    return 0;
}
