#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Arrays/nodeArray.h>
#include <Structs/Arrays/edgeArray.h>
#include <Algorithms/basicGraphAlgorithms.h>
#include <Utilities/graphGenerators.h>
#include <Utilities/Timer.h>

struct wrapper
{
	wrapper( unsigned int data = 0):m_data(data)
	{	
	}
	
	bool operator ==( const wrapper& other) const
	{
		return m_data == other.m_data;
	}
	
	bool operator !=( const wrapper& other) const
	{
		return m_data != other.m_data;
	}
	
	bool operator <( const wrapper& other) const
	{
		return m_data < other.m_data;
	}
	/*
	bool operator >( const wrapper& other) const
	{
		return m_data > other.m_data;
	}
	
	bool operator <=( const wrapper& other) const
	{
		return m_data <= other.m_data;
	}
	
	bool operator >=( const wrapper& other) const
	{
		return m_data >= other.m_data;
	}*/
	
	friend std::ostream& operator << ( std::ostream& out, const wrapper& other)
	{
		//out << other;
		return out;
	}
	
	unsigned int m_data;
};


int main( int argc, char* argv[])
{
    Timer timer; 
    unsigned int height;
    if( argc != 2 )
    {
        height = 5;
    }
    else
    {
        height = atoi( argv[1]);
    }

    typedef DynamicGraph< unsigned int, unsigned int, AdjacencyListImpl>    Graph;
    typedef Graph::node                                                     node;
    typedef Graph::edge                                                     edge;


    Graph G;
    Graph::sizeType numnodes = 100;
    Graph::sizeType numedges = 10000;
    timer.start();
    randomGraph<std::string,unsigned int>( G, numnodes, numedges);
    std::cout << "\tTime:\t" << timer.getElapsedTime() << "sec\n";
    
    std::ofstream out;
    out.open("graph.dot");
    printDot(G,out);   
    out.close();
    out.open("forward.dot");
    G.edges()->printDot(out);   
    out.close();
    out.open("backward.dot");
    G.backwardEdges()->printDot(out);   
    out.close();
    out.open("nodes.dot");
    G.nodes()->printDot(out);   
    out.close();
    
    return 0;
}
