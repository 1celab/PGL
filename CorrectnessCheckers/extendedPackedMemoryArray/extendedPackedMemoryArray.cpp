#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Arrays/extendedPackedMemoryArray.h>

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


    ExtendedPackedMemoryArray< wrapper> array;
	PackedMemoryArray< wrapper> pma;
    std::vector< unsigned int> vector;
    unsigned int i = 0;
    bool failed = false;
      
    std::fstream in;
	in.open("../../ResultGenerators/mersenneTwister/10M_random_numbers.out");
	for( unsigned int i = 0; i < 2; i++)
    {
        std::stringstream ss;
        ss << "out" << i << "s.dot";
        std::string s = ss.str();
        std::ofstream out(s.c_str());
        array.printDot( out);
        out.close();

		double random;
        in >> random; 
        unsigned int position = random * array.size();
        std::cout << "\nInsertion Request " << i << ": " << position << std::endl;
        
        array.insert( array.begin() + position, wrapper(position));
		pma.insert( pma.begin() + position, wrapper(position));
		vector.insert( vector.begin() + position, position);

        std::stringstream ssf;
        ssf << "out" << i << "f.dot";
        std::string sf = ssf.str();
        std::ofstream outf(sf.c_str());
        array.printDot( outf);
        outf.close();
    }	
	in.close();


    /*in.open("../../ResultGenerators/mersenneTwister/10M_random_numbers.out");
	for( unsigned int i = 0; i < 2; i++)
    {
        std::stringstream ss;
        ss << "outer" << i << "s.dot";
        std::string s = ss.str();
        std::ofstream out(s.c_str());
        array.printDot( out);
        out.close();


		double random;
        in >> random; 
        unsigned int position = random * array.size();
        std::cout << "Erasure Request " << i << ": " << position << std::endl;
        array.erase( position);
        vector.erase( vector.begin() + position);

        std::stringstream ssf;
        ssf << "outer" << i << "f.dot";
        std::string sf = ssf.str();
        std::ofstream outf(sf.c_str());
        array.printDot( outf);
        outf.close();
    }	
	in.close();
*/


    std::ofstream outarray("array.dot");   

    i = 0;
    failed = false;

    outarray << "digraph BST {\n\tnode [fontname=\"Arial\"]\n";
    
    for( ExtendedPackedMemoryArray< wrapper>::Iterator it = array.begin(); it != array.end(); ++it)
    {
        if( (it - array.begin()) % 8 == 0)
        {
            outarray << "bucket"<< it - array.begin() <<"[ shape = \"record\", label = \"";
        }
        
        if( it->m_data != vector[i])
        {
			std::cout << "Wrong Element "<< (*it).m_data << " != " << vector[i] << "\t";
            failed = true;
            break;
        }  

        i++;

        outarray << "{" << it - array.begin() << "|" << *it << "}|";

        if( (it - array.begin()) % 8 == 7)
        {
            outarray << "\"]";
            outarray << "bucket" << it - array.begin() - 7 << " -> bucket" << it - array.begin() + 1 << "\n";
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

    std::cout << std::endl;
    for( ExtendedPackedMemoryArray< wrapper>::Iterator it = array.begin(); it != array.end(); ++it)
    {
        std::cout << *it << " | ";
    }

	std::cout << std::endl;
    for( PackedMemoryArray< wrapper>::Iterator it = pma.begin(); it != pma.end(); ++it)
    {
        std::cout << *it << " | ";
    }

    std::cout << std::endl;

    for( std::vector< unsigned int>::iterator it = vector.begin(); it != vector.end(); ++it)
    {
        std::cout << *it << " | ";
    }

    std::cout << std::endl;
    std::ofstream out("out.dot");
    array.printDot( out);
    out.close();

    return 0;
}
