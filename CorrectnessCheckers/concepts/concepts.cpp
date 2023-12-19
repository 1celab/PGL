#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
//#include <Structs/Arrays/nodeArray.h>
//#include <Structs/Arrays/edgeArray.h>
#include <Algorithms/basicGraphAlgorithms.h>
//#include <Utilities/graphGenerators.h>

#include <boost/utility/enable_if.hpp>


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

	void print ( std::ostream& out)
	{
		//out << other;
	}
	
	unsigned int m_data;
};


using namespace boost;


template<class T> T& ref();
template<class T> T  val();

template<class T>
struct has_inserter
{
    template<class U> 
    static char test(char(*)[sizeof(T.marked)]);

    template<class U> 
    static long test(...);

    enum { value = 1 == sizeof test<T>(0) };
};


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

    std::cout << has_inserter<wrapper>::value << "\n";
    
    return 0;
}
