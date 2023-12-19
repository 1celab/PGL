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


typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
//typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   Graph;
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

    Graph G;
    pmaGraph pmaG;


    std::vector< Graph::NodeDescriptor> adjDesc;
    std::vector< pmaGraph::NodeDescriptor> pmaDesc;
    std::vector< std::pair<unsigned int, unsigned int> > edges;

    adjDesc.push_back(Graph::NodeDescriptor());
    pmaDesc.push_back(pmaGraph::NodeDescriptor());
    edges.push_back( std::pair<unsigned int, unsigned int>(1,3));
    edges.push_back( std::pair<unsigned int, unsigned int>(1,2));
    edges.push_back( std::pair<unsigned int, unsigned int>(1,5));
    edges.push_back( std::pair<unsigned int, unsigned int>(2,4));
    edges.push_back( std::pair<unsigned int, unsigned int>(2,5));
    edges.push_back( std::pair<unsigned int, unsigned int>(2,1));
    edges.push_back( std::pair<unsigned int, unsigned int>(2,3));
    edges.push_back( std::pair<unsigned int, unsigned int>(3,1));
    edges.push_back( std::pair<unsigned int, unsigned int>(3,4));
    edges.push_back( std::pair<unsigned int, unsigned int>(3,5));
    edges.push_back( std::pair<unsigned int, unsigned int>(3,2));
    edges.push_back( std::pair<unsigned int, unsigned int>(4,2));
    edges.push_back( std::pair<unsigned int, unsigned int>(4,3));
    edges.push_back( std::pair<unsigned int, unsigned int>(5,2));
    edges.push_back( std::pair<unsigned int, unsigned int>(5,3));
    edges.push_back( std::pair<unsigned int, unsigned int>(5,1));

    for( unsigned int i = 0; i < 5; ++i)
    {
        G.insertNode();
        pmaG.insertNode();
    }

    for( Graph::NodeIterator u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
    {
        adjDesc.push_back(G.getNodeDescriptor(u));
    }

    for( pmaGraph::NodeIterator u = pmaG.beginNodes(), lastNode = pmaG.endNodes(); u != lastNode; ++u)
    {
        pmaDesc.push_back(pmaG.getNodeDescriptor(u));
    }

    for( unsigned int i = 0; i < edges.size(); ++i)
    {
        G.insertEdge( adjDesc[edges[i].first], adjDesc[edges[i].second]);
        pmaG.insertEdge( pmaDesc[edges[i].first], pmaDesc[edges[i].second]);
    }

    std::string basePath = "/home/michai/Projects/Graphs/";
    std::string map ="random";
    std::string format = "GraphViz";

    GraphVizWriter<Graph> writer(basePath + "/" + format + "/" + map + ".dot");
    GraphVizWriter<pmaGraph> pmaWriter(basePath + "/" + format + "/random2"  + ".dot");

    G.write(&writer);
    pmaG.write(&pmaWriter);

    pmaG.printInternalDot(basePath + "/" + format + "/internal.dot");

    return 0;
}
