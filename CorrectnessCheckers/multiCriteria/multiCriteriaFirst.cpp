#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <Algorithms/ShortestPath/multiCriteriaDijkstra.h>
#include <Algorithms/ShortestPath/namoaStar.h>


struct node: DefaultGraphItem
{
	node( unsigned int data = 0):id(std::numeric_limits<unsigned int>::max()), marked(false)
	{	
	}

    void setProperty( const std::string& name, const std::string& value)
    {
        DefaultGraphItem::setProperty( name, value);
        if( !name.compare("id"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>id;
        }
		if( !name.compare("h"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>heuristic;
        }

       /* if( !name.compare("lat"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>lat;
        }*/
    }
	
    void print( std::ofstream& out)
    {
        DefaultGraphItem::print(out);
        out <<  "\\|" << marked;
        return;
    }
    
    template <class GraphType>
    void printLabels( std::ostream& out, GraphType& G)
    {
        for ( std::vector<Label>::iterator it = labels.begin(); it != labels.end(); ++it)
        {
            out << "\t";
            it->print(out, G);
            out << "\n";
        }
        return;
    }

   /* void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "lon" << std::setprecision(10) << valueDelimiter << lon << propertyDelimiter;
        out << "lat" << std::setprecision(10) << valueDelimiter << lat << propertyDelimiter;
    }
    
    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"lon\":\"" << lon << "\"," << "\"lat\":\"" << lat << "\"";
    }*/

    unsigned int id;
    bool marked;
    std::vector<Label> labels;
	unsigned int heuristic;

    /*unsigned int m_data;
    unsigned int dist;
    DynamicGraph< AdjacencyListImpl, wrapper, wrapper>::NodeDescriptor m_pred;
    unsigned int weight;
    unsigned int pqitem;*/
};


struct edge: DefaultGraphItem
{
	edge( unsigned int data = 0): criteriaList(2)
	{	
	}

   void setProperty( const std::string& name, const std::string& value)
    {
        DefaultGraphItem::setProperty( name, value);
        if( !name.compare("t"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>criteriaList[0];
        }
        if( !name.compare("c"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>criteriaList[1];
        }

    }

    /*void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "weight" << valueDelimiter << weight << propertyDelimiter;
    }

    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"weight\":" << weight << "";
    }*/

    CriteriaList criteriaList;
};


typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
//typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   Graph;
typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   pmaGraph;
typedef DynamicGraph< AdjacencyListImpl>                   graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;


int main( int argc, char* argv[])
{

    Graph G;

    GMLReader<Graph> reader("/home/mali/Projects/pgl/CorrectnessCheckers/multiCriteria/graph1.gml");
    G.read(&reader);

    //NodeIterator u, v;
   // EdgeIterator e, k;
    NodeIterator s, t, u, lastNode;

    for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
    {
        if ( u->id == 0)
        {
            s = u;
        }
		if ( u->id == 7)
        {
            t = u;
        }
    }

	unsigned int timestamp;
	NamoaStarDijkstra<Graph> dij(G,2,&timestamp);
	dij.runQuery(s,t);

    //multiCriteriaDijkstra( G, 4, s);

    for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
    {
        std::cout << u->id << ":\n";
        u->printLabels(std::cout, G);
    }

    GraphVizWriter<Graph> writer("/home/mali/Projects/pgl/CorrectnessCheckers/multiCriteria/graph1.dot");
    G.write(&writer);
        
    return 0;
}
