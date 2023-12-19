#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <boost/utility/enable_if.hpp>
#include <iomanip>
#include <Utilities/geographic.h>
#include <Utilities/graphGenerators.h>
#include <Utilities/timer.h>
#include <boost/program_options.hpp>

#include <Algorithms/basicGraphAlgorithms.h>
#include <Algorithms/ShortestPath/dijkstra.h>
#include <Algorithms/ShortestPath/Astar/aStarLmk.h>
#include <Algorithms/ShortestPath/Astar/bidAstarAveLmk.h>
#include <Algorithms/ShortestPath/Astar/bidAstarMaxLmk.h>
#include <Algorithms/ShortestPath/Astar/bidAstarSymLmk.h>
#include <Algorithms/ShortestPath/Astar/bidAstarAveEcl.h>
#include <Algorithms/ShortestPath/Astar/bidAstarMaxEcl.h>
#include <Algorithms/ShortestPath/Astar/bidAstarSymEcl.h>
#include <Algorithms/ShortestPath/aStarDijkstra.h>
#include <Algorithms/ShortestPath/bidirectionalDijkstra.h>
#include <Algorithms/ShortestPath/Astar/LandmarkGenerator/lmkGen.h>

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

    std::string basePath = "/home/andreas/Projects/Graphs/DIMACS10/";
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
    InEdgeIterator k;

    for(NodeIterator v, u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {   
	    for(EdgeIterator e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
	    {
            
            v = G.target(e);
            k = G.getInEdgeIterator(e);

            e->weight = k->weight = euclideanDistance( u->x, u->y, v->x, v->y);
            
        }

        u->timestamp = 0;             
    }

    std::vector<NodeDescriptor>& ids = reader.getIds();

    LandmarkGenerator<Graph> landmarkGen(G, ids);
    //select 10 landmarks in the map
    //landmarkGen.farthestLandmarkSelection(10);
    //landmarkGen.writeLandmarkDistances(basePath + map + "landmarks.dat");  
    landmarkGen.loadLandmarkDistances(basePath + /*map +*/ "/landmarks.dat");


    unsigned int timestamp = 1;
    //BidirectionalDijkstra<Graph> bid(G, &timestamp);
    //Dijkstra<Graph> dijkstra(G, &timestamp);
    
    AStarLmk<Graph> aStarLmk (G, &timestamp);    

    NodeIterator source= G.getNodeIterator(ids[113788]), target=G.getNodeIterator(ids[106928]);

    aStarLmk.runQuery(source, target);

    aStarLmk.printShortestPath(source, target, ids);

    return 0;
}
