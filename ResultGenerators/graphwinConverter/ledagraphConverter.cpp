#include <iostream>
#include <fstream>
#include <limits>

int main( int argc, char* argv[])
{
    std::ifstream in(argv[1]);
    std::ofstream out(argv[2]);    

    typedef unsigned int sizeType;

    sizeType numnodes, numedges, source, target;

    for( unsigned int i = 0; i < 4; i++)
    {
        in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }

    in >> numnodes;
    out << numnodes;
    std::cout << "\rCopying nodes..." << std::flush;
    for( sizeType i = 0; i < numnodes + 1; i++)
    {
        in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
        if( i % (numnodes/100) == 0)
        {
            std::cout << "\rCopying nodes..." << i / (numnodes/100) << "%" << std::flush;
        }
    }
    std::cout << "\rCopying nodes...done!\n";
    
    
    in >> numedges;
    out << numedges;
    std::cout << "\rCopying edges..." << std::flush;
    for( sizeType i = 0; i < numedges; i++)
    {
        in >> source;
        in >> target;
        out << source << " ";
        out << target << "\n";
        in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
        if( i % (numedges/100) == 0)
        {
            std::cout << "\rCopying edges..." << i / (numedges/100) << "%" << std::flush;
        }
    }
    std::cout << "\rCopying edges...done!\n";
    in.close();
    out.close();

    return 0;
}
