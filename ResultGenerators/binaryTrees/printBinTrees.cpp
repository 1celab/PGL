#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <Structs/Trees/perfectBinaryTreeVEB.h>
#include <Structs/Trees/perfectBinaryTreeBFS.h>

int main( int argc, char* argv[])
{

    if( argc != 2 )
    {
        return -1;
    }

    unsigned int height = atoi( argv[1]);


    perfectBinaryTreeVEB< int > A(height);
    perfectBinaryTreeBFS< int > B(height);
    
    std::ofstream outveb("veb.dot");
    A.printDot( outveb, false);
    outveb.close();

    std::ofstream outbfs("bfs.dot");
    B.printDot( outbfs, false);
    outbfs.close();

    return 0;
}
