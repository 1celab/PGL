#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>

#include <Utilities/timer.h>
#include <Utilities/geographic.h>
#include <Utilities/progressBar.h>
#include <Utilities/graphGenerators.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <Algorithms/ShortestPath/Astar/LandmarkGenerator/lmkGen.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

struct node: DefaultGraphItem
{
	node():x(0),y(0),dist(std::numeric_limits<unsigned int>::max()),pqitem(0),pred(0),timestamp(0)
	{
    }

    union
    {
        unsigned int dist;
        bool marked;
    };

    unsigned int distBack;
    unsigned int x, y;
    unsigned int pqitem, pqitemBack;
    void* pred;
    void* succ;
    unsigned int timestamp;
};


struct edge: DefaultGraphItem
{
	edge():weight(0)
	{	
	}

    unsigned int weight;
};


template < typename GraphType>
void calcWeights( GraphType& G)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typedef typename GraphType::EdgeIterator EdgeIterator;
    typedef typename GraphType::InEdgeIterator InEdgeIterator;
    typedef typename GraphType::SizeType SizeType;

    NodeIterator u,v,lastnode;
    EdgeIterator e,lastedge;
    InEdgeIterator k;

    unsigned int max = 0;
    //unsigned int max_ux, max_uy, max_vx, max_vy, u_id, v_id;

    std::stringstream sstr;
    sstr << "Calculating weights of " << G.getNumEdges() << " edges";
    ProgressBar edge_progress( G.getNumEdges(),sstr.str());
    Timer timer; 
    timer.start();
    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
	    for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
	    {
            
            v = G.target(e);
            k = G.getInEdgeIterator(e);
            
            e->weight = euclideanDistance( u->x, u->y, v->x, v->y);
            k->weight = e->weight;

            if( e->weight > max)
            {
                //max_ux = u->x;
                //max_uy = u->y;  
                //max_vx = v->x;
                //max_vy = v->y;
                //u_id = G.getRelativePosition(u);
                //v_id = G.getRelativePosition(v);
                max = e->weight;
            }
            ++edge_progress;
            //std::cout << u->lat << " " << v->lat << std::endl;
        }
    }

    std::cout << "\tMax weight: " << max << "\tTime:\t" << timer.getElapsedTime() << std::endl;
    //std::cout << "( " << max_ux << ", " << max_uy << ") " << u_id << "->" << v_id << " ( " << max_vx << ", " << max_vy << ")\n";
}

typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   pmaGraph;
typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;

typedef GraphReader< Graph> AdjReader;
typedef GraphReader< pmaGraph> PmaReader;

int main(int argc, char** argv) 
{
    unsigned int algLandmarkSelection = 2, numLandmarks = 0;

    std::string basePath = std::string(getenv("HOME")) + "/Projects/Graphs/";
    std::string map = "luxembourg";
    unsigned int format = 0;

	AdjReader* reader = 0;
    //PmaReader* reader = 0;

    // Declare the supported options.
    po::options_description desc("Supported options");

    desc.add_options()
        ("number of landmarks,l", po::value<unsigned int>(), "The number of the harvested landmarks.")
        ("selection technique,s", po::value<unsigned int>(), "Select between Random[1], Farthest[2], Planar[3]. Default:2. Based on hop count distance measure.")
		("format,f", po::value< unsigned int>(), "map format. DIMACS10[0], DIMACS9[1]. Default:0")
        ("map,m", po::value< std::string>(), "input map. The name of the map to read. Default:'luxembourg'. Maps should reside in '$HOME/Projects/Graphs/' and should consist of 2 files, both with the same map name prefix, and suffixes 'osm.graph' and 'osm.xyz' in the case of DIMACS10 and, '.gr' and '.co' in the case of DIMACS9.")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if ( vm.empty()) 
    {
        std::cout << desc << "\n";
        return 0;
    }

    if ( vm.count("number of landmarks"))
    {
        numLandmarks = vm["number of landmarks"].as< unsigned int>();
    }

    if ( vm.count("selection technique"))
    {
        algLandmarkSelection = vm["selection technique"].as< unsigned int>();
    }

    if ( vm.count("map"))
    {
        map = vm["map"].as<std::string>();
    }


	if ( vm.count("format") && ( format = vm["format"].as<unsigned int>()) == 1)
    {
        basePath = basePath + "DIMACS9/" + map + "/";

        reader = new DIMACS9Reader<Graph>( basePath + map + ".gr",
                                           basePath + map + ".co");
        //alternative
        //reader = new DIMACS9Reader<pmaGraph>( basePath + map + ".gr",
        //                                         basePath + map + ".co");
    }

    else
    {
        basePath = basePath + "DIMACS10/" + map + "/";

        reader = new DIMACS10Reader<Graph>( basePath + map + ".osm.graph",
                                            basePath + map + ".osm.xyz");

        //alternative
        //reader = new DIMACS10Reader<pmaGraph>( basePath + map + ".osm.graph",
        //                                          basePath + map + ".osm.xyz");
    }

    Graph G;
    //alternative
    //pmaGraph G;

    G.read(reader);

    if(format == 0)
        calcWeights(G);
    
    LandmarkGenerator<Graph> landmarkGen(G, reader->getIds());

    switch( algLandmarkSelection)
    {
        case 1:
            landmarkGen.randomLandmarkSelection(numLandmarks);
            break;
        case 2:
            landmarkGen.farthestLandmarkSelection(numLandmarks);
            break;
        case 3:
            landmarkGen.planarLandmarkSelection(numLandmarks);
            break;
    }
    
    landmarkGen.writeLandmarkDistances(basePath + "landmarks.dat");  

    return 0;
}

