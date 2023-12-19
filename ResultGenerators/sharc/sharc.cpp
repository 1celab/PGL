#include <iostream>
#include <fstream>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <limits>
#include <Utilities/timer.h>
#include "preprocessingSharc.h"
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;

struct node: DefaultGraphItem
{
	node():x(0),y(0),key(std::numeric_limits<unsigned int>::max()),pqitem(0),timestamp(0),inactiveLevel(std::numeric_limits<unsigned int>::max()),id(0),pred(0),isOneShell(false)
	{	
	}

    union{ unsigned int x; unsigned int dist; };
    union{ unsigned int y; unsigned int pqBitem; };
    union{ unsigned int key; unsigned int idBoundary; };
    unsigned int pqitem;
    unsigned int timestamp;
	unsigned short inactiveLevel;
	unsigned int id;
    void* pred;
    bool isOneShell;
    std::vector<unsigned short> cell;
    std::vector<unsigned int> labels;
    
};

struct edge: DefaultGraphItem
{
    edge():hops(1),levelInsert(std::numeric_limits<unsigned int>::max()),isActive(true)
    {
    }

    unsigned int weight;
    unsigned short hops;
    unsigned short levelInsert;
    bool isActive;
	//void* viaNode;
    std::vector<std::vector<bool> > flag;
};

		
typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;

int main( int argc, char* argv[])
{
    //number of levels of the partition
    unsigned short numLevel = 3;
    unsigned int timestamp = 0;
    NodeIterator u;

    //name of the graph
    std::string map ="luxembourg";
    std::string basePath = std::string(getenv("HOME")) + "/Projects/Graphs/DIMACS10/" + map + "/";
    Graph G;
    DIMACS10Reader<Graph> reader( basePath + map + ".osm.graph", basePath + map + ".osm.xyz");
    G.read(&reader);
    Timer timer; 
    std::cout << "Graph has " << (double)G.memUsage() / 1048576.0 << " Mbytes. Time spent to read:\t" 
              << timer.getElapsedTime() << "sec" << std::endl;
    std::cout << "IDs container capacity: "
              << (sizeof(NodeDescriptor) * reader.getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;

    for( unsigned int i = 1; i <= G.getNumNodes(); ++i)
    {
        u = G.getNodeIterator(reader.getIds()[i]);
        u->id = i;
    }
    
    //number of cells at each level	
	std::vector<unsigned short> parts;
    parts.push_back(256);
    parts.push_back(16);
    parts.push_back(4);
        
	PreprocessingSharc<Graph> prepo( G, numLevel, &timestamp, parts);
    
    timer.start();	
    prepo.runPreprocessing();
    timer.stop();
    std::cout <<"\n\tTime spent to run Preprocessing: " << timer.getElapsedTime() / 60 << " min.\n" << std::endl;

    //write the modified graph after the preprocessing
	DIMACS10Writer<Graph> writer( basePath + map + ".osm.graph.modified", basePath + map + ".osm.xyz.modified");
    G.write(&writer);
	
	return 0;
}
