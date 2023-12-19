#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <Structs/Trees/pmaDensityTree.h>

int main( int argc, char* argv[])
{

    if( argc != 2 )
    {
        return -1;
    }

    unsigned int height = atoi( argv[1]);


    pmaDensityTree A(height,4096);
    
    std::ofstream out("density.dot");
    A.printDot( out);
    out.close();

    return 0;
}
