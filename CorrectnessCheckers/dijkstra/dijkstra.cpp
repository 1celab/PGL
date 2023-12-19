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
#include <Utilities/timer.h>


struct node: DefaultGraphItem
{
	node( unsigned int data = 0):x(0),y(0),dist(std::numeric_limits<unsigned int>::max()),pqitem(0),pred(0),timestamp(0)//,marked(false),component(0)
	{	
	}

    void print( std::ofstream& out)
    {
        out << "\\|" << dist;
        return;
    }

    void setProperty( const std::string& name, const std::string& value)
    {
        DefaultGraphItem::setProperty( name, value);
        if( !name.compare("x"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>x;
        }
        if( !name.compare("y"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>y;
        }
    }

    void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        //out << "lon" << std::setprecision(10) << valueDelimiter << lon << propertyDelimiter;
        //out << "lat" << std::setprecision(10) << valueDelimiter << lat << propertyDelimiter;
        out << "x" << valueDelimiter << x << propertyDelimiter;
        out << "y" << valueDelimiter << y << propertyDelimiter;
    }
    
    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"x\":" << x << "," << "\"y\":" << y << "";
    }

    //double lon,lat;
    unsigned int x,y;
    unsigned int dist;
    unsigned int pqitem;
    void* pred;
    unsigned int timestamp;
    //bool marked;
    //unsigned int component;

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

    void setProperty( const std::string& name, const std::string& value)
    {
        DefaultGraphItem::setProperty( name, value);
        if( !name.compare("weight"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>weight;
        }
    }
    
    void print( std::ofstream& out)
    {
        out << weight;
        return;
    }

    void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "w" << valueDelimiter << weight << propertyDelimiter;
    }

    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"w\":" << weight << "";
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
    unsigned int max_ux, max_uy, max_vx, max_vy, u_id, v_id;

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
                u_id = G.getId(u);
                v_id = G.getId(v);
                max = e->weight;
            }
            ++edge_progress;
            //std::cout << u->lat << " " << v->lat << std::endl;
        }
    }

    std::cout << "Max weight: " << max << std::endl;
    std::cout << "( " << max_ux << ", " << max_uy << ") " << u_id << "->" << v_id << " ( " << max_vx << ", " << max_vy << ")\n";
}


template < typename GraphType>
void readXYZ( GraphType& G, const std::string& filename)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    NodeIterator u,lastnode;
    std::ifstream in;
    in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
    try 
    {
        in.open( filename.c_str());
        unsigned int z;
        std::stringstream sstr;
        sstr << "Reading coordinates of " << G.getNumNodes() << " nodes";
        ProgressBar node_progress( G.getNumNodes(),sstr.str());
        for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
        {
            in >> u->x;
            in >> u->y;
            in >> z;
            ++node_progress;
        }
        assert( in.good());
    }
    catch (std::ifstream::failure e) 
    {
       std::cerr << "Exception opening/reading file '" << filename << "'\n";
       throw e;
    }
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

    RandomWeightedGenerator<Graph> generator(5,15,10);
    G.generateFrom(&generator);

    //DIMACS10Reader<Graph> reader("/Users/panosmichail/Projects/Graphs/DIMACS10/germany.osm.graph",
    //                            "/Users/panosmichail/Projects/Graphs/DIMACS10/germany.osm.xyz");
    //G.read(&reader);

    std::cout << "Graph has " << G.memUsage() << " bytes " << std::endl;

    //std::cout << "Found " << findStronglyConnectedComponents(G) << " strongly connected components!\n";
    
    /*if( isConnected(G)) std::cout << "G is connected\n";
    else                std::cout << "G is not connected\n";

    if( isWeaklyConnected(G)) std::cout << "G is weakly connected\n";
    else                std::cout << "G is not weakly connected\n";


    for( NodeIterator u = G.beginNodes(); u != G.endNodes(); ++u)
    {
        if( G.beginEdges(u) == G.endEdges(u))
        {
            std::cout << "Node " << G.getId(u) << " has no edges\n";
        }
    }*/

    //readXYZ( G, "/Users/panosmichail/Projects/Graphs/DIMACS10/luxembourg.osm.xyz");
    //readXYZ( pmaG, "/home/michai/Projects/Graphs/DIMACS/luxembourg.osm.xyz");
    

    /*calcWeights(G);*/
    
    unsigned int num_queries = 1;
    std::stringstream querystream;
    querystream << "Running " << num_queries << " queries";
    ProgressBar query_progress(num_queries,querystream.str());
    Timer timer;
    timer.start();
    unsigned int timestamp;
    Dijkstra<Graph> dijkstra(G,&timestamp);
    for( unsigned int i = 0; i < num_queries; ++i)    
    {
        NodeIterator s = G.chooseNode();
        dijkstra.buildTree(s);
        ++query_progress;
    }
    std::cout << "\nTime:\t" << timer.getElapsedTime() << "sec\n\n";

    GraphVizWriter<Graph> writer("/Users/panosmichail/Projects/Graphs/GraphViz/randomgraph.dot");
    G.write(&writer);

    return 0;
}
