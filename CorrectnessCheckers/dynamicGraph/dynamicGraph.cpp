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
	node( unsigned int data = 0):x(0),y(0),component(0)
	{	
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
    bool marked;
    unsigned int component;

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
	
    void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "weight" << valueDelimiter << weight << propertyDelimiter;
    }

    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"w\":" << weight << "";
    }

    unsigned int weight;
};


//typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   Graph;
typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   pmaGraph;
typedef DynamicGraph< AdjacencyListImpl>                   graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;

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
    unsigned int max_ux, max_uy, max_vx, max_vy, u_id, v_id;

    std::stringstream sstr;
    sstr << "Calculating weights of " << G.getNumEdges() << " edges";
    ProgressBar edge_progress( G.getNumEdges(),sstr.str());

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
                max_ux = u->x;
                max_uy = u->y;  
                max_vx = v->x;
                max_vy = v->y;
                u_id = G.getRelativePosition(u);
                v_id = G.getRelativePosition(v);
                max = e->weight;
            }
            ++edge_progress;
            //std::cout << u->lat << " " << v->lat << std::endl;
        }
    }

    std::cout << "Max weight: " << max << std::endl;
    //std::cout << "( " << max_ux << ", " << max_uy << ") " << u_id << "->" << v_id << " ( " << max_vx << ", " << max_vy << ")\n";
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

    //typedef DynamicGraph< AdjacencyListImpl, node, edge>   Graph;
    
    Graph G;
    pmaGraph pmaG;

    RandomGenerator<Graph> generator(4,6);
    G.generateFrom(&generator);


    std::string basePath = "/home/michai/Projects/Graphs/";
    std::string map ="random";
    std::string format = "DIMACS10";
    /*DIMACS10Reader<Graph> reader( basePath + "/" + format + "/" + map + ".osm.graph",
                                basePath + "/" + format + "/" + map + ".osm.xyz");
    
    G.read(&reader);*/

    /*std::cout << "Found " << findStronglyConnectedComponents(G) << " strongly connected components!\n";

    
    if( isConnected(G)) std::cout << "G is connected\n";
    else                std::cout << "G is not connected\n";

    if( isWeaklyConnected(G)) std::cout << "G is weakly connected\n";
    else                std::cout << "G is not weakly connected\n";


    for( NodeIterator u = G.beginNodes(); u != G.endNodes(); ++u)
    {
        if( G.beginEdges(u) == G.endEdges(u))
        {
            std::cout << "Node " << G.getRelativePosition(u) << " has no edges\n";
        }
    }
*/

    //calcWeights(G);

    format = "gml";
    GMLWriter<Graph> gmlwriter(basePath + "/" + format + "/" + map + ".gml");

    
    return 0;
}
