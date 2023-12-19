#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <Algorithms/basicGraphAlgorithms.h>
#include <Algorithms/ShortestPath/dijkstra.h>
#include <boost/utility/enable_if.hpp>
#include <iomanip>
#include <Utilities/geographic.h>
#include <Utilities/graphGenerators.h>


struct node: DefaultGraphItem
{
	node( unsigned int data = 0):x(0),y(0)
	{	
	}


    //double lon,lat;
    unsigned int x,y;

    /*unsigned int m_data;
    unsigned int dist;
    DynamicGraph< AdjacencyListImpl, wrapper, wrapper>::NodeDescriptor pred;
    unsigned int weight;
    unsigned int pqitem;*/
};


struct edge: DefaultGraphItem
{
	edge( unsigned int data = 0):weight(0)
	{	
	}

    unsigned int weight;
};


typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   Graph;
//typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;

template < typename GraphType>
void calcWeights( GraphType& G)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typedef typename GraphType::EdgeIterator EdgeIterator;
    typedef typename GraphType::SizeType SizeType;

    NodeIterator u,v,lastnode;
    EdgeIterator e,lastedge;

    unsigned int max = 0;
    unsigned int max_ux, max_uy, max_vx, max_vy;

    std::stringstream sstr;
    sstr << "Calculating weights of " << G.getNumEdges() << " edges";
    ProgressBar edge_progress( G.getNumEdges(),sstr.str());

    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
	    for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
	    {
            
            v = G.target(e);

            e->weight = euclideanDistance( u->x, u->y, v->x, v->y);
            if( e->weight > max)
            {
                max_ux = u->x;
                max_uy = u->y;  
                max_vx = v->x;
                max_vy = v->y;
                max = e->weight;
            }
            ++edge_progress;
            //std::cout << u->lat << " " << v->lat << std::endl;
        }
    }

    std::cout << "Max weight: " << max << std::endl;
    std::cout << "( " << max_ux << ", " << max_uy << ") " <<  " ( " << max_vx << ", " << max_vy << ")\n";
}



int main( int argc, char* argv[])
{
    unsigned int height;
    if( argc != 2 )
    {
        height = 5;
    }
    else
    {
        height = atoi( argv[1]);
    }

    Graph G;
    NodeIterator u,v,lastnode;


    std::string map = "luxembourg";

    DIMACS10Reader<Graph> reader("/home/michai/Projects/Graphs/DIMACS10/" + map + ".osm.graph","/home/michai/Projects/Graphs/DIMACS10/" + map + ".osm.xyz");
    G.read(&reader);

    std::cout << "Graph has " << G.memUsage() << " bytes " << std::endl;


    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
        if ( reader.getIds()[G.getRelativePosition(u)] != G.getNodeDescriptor(u))
        {
            std::cout << "Failure!\n";
            exit(0);
        }
    }
    

    calcWeights(G);
    //calcWeights(pmaG);

    DDSGWriter<Graph> writer("/home/michai/Projects/Graphs/DIMACS10/" + map + ".ddsg");
    G.write(&writer);
    
    //NodeIterator s = G.chooseNode();
    //dijkstra(G,s);

    /*JSONWriter<Graph> jsonwriter("/home/michai/Projects/Graphs/JSON/luxembourg.json");
    G.write(&jsonwriter);

    GMLWriter<Graph> gmlwriter("/home/michai/Projects/Graphs/gml/luxembourg.gml");
    G.write(&gmlwriter);
    */
    return 0;
}
