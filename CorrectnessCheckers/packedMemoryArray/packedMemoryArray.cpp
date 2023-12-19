#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Arrays/packedMemoryArray.h>
#include <Utilities/mersenneTwister.h>
#include <vector>

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
		out << other.m_data;
		return out;
	}
	
	unsigned int m_data;
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


    PackedMemoryArray< wrapper> array;
    std::vector< unsigned int> vector;
    unsigned int i = 0;
    bool failed = false;
      
    MersenneTwister gen;
	for( unsigned int i = 0; i < 40; i++)
    {
        /*std::stringstream ss;
        ss << "/home/michai/Projects/pgl/include/CorrectnessCheckers/packedMemoryArray/out" << i << "s.dot";
        std::string s = ss.str();
        std::ofstream out(s.c_str());
        array.printDot( out);
        out.close();*/

		double random = gen.getRandomNormalizedDouble();

        unsigned int position = random * array.size();
        std::cout << "\nInsertion Request " << i << ": " << position << std::endl;
        
        if( i == 114)
        {
            unsigned int a = 0;
        }

        //array.insert( array.begin() + position, wrapper(position));
       // array.optimalInsert( wrapper(position));
		array.push_back( wrapper(i));
		//vector.insert( vector.begin() + position, position);
		vector.push_back( i);
		
        /*std::stringstream ssf;
        ssf << "/home/michai/Projects/pgl/include/CorrectnessCheckers/packedMemoryArray/out" << i << "f.dot";
        std::string sf = ssf.str();
        std::ofstream outf(sf.c_str());
        array.printDot( outf);
        outf.close();*/
    }	


    std::ofstream outarray("array.dot");   

    i = 0;
    failed = false;

    outarray << "digraph BST {\n\tnode [fontname=\"Arial\"]\n";
    
    for( PackedMemoryArray< wrapper>::Iterator it = array.begin(); it != array.end(); ++it)
    {
        if( array.getElementIndexOf(it) % 8 == 0)
        {
            outarray << "bucket"<< array.getElementIndexOf(it) <<"[ shape = \"record\", label = \"";
        }
        if( (*it).m_data != vector[i])
        {
            failed = true;
            break;
        }  
        i++;

        outarray << "{" << array.getElementIndexOf(it) << "|" << *it << "}|";

        if( array.getElementIndexOf(it) % 8 == 7)
        {
            outarray << "\"]";
            outarray << "bucket" << array.getElementIndexOf(it) - 7 << " -> bucket" << array.getElementIndexOf(it) + 1 << "\n";
        }
    }

    if( failed)
    {
        std::cout << "Failure at " << i << "!\n";
    }
    else
    {
        std::cout << "Success!\n";
    }

    outarray << "\"]}";
    outarray.close();

    

    
    for( PackedMemoryArray< wrapper>::Iterator it = array.begin(); it != array.end(); ++it)
    {
        std::cout << *it  << " | ";
    }
    std::cout << std::endl;

    i = 0;
    for( PackedMemoryArray< wrapper>::Iterator it = array.begin(); it != array.end(); ++it)
    {
        std::cout << vector[i] << " | ";
        ++i;
    }
    std::cout << std::endl;

    std::ofstream out("out.dot");
    array.printDot( out);
    out.close();


    return 0;
}
