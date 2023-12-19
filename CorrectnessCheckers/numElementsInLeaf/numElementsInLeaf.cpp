#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <Structs/Trees/perfectBinaryTreeVEB.h>
#include <Structs/Trees/perfectBinaryTreeBFS.h>
#include <Structs/Trees/perfectBinaryTree.h>
#include <Algorithms/perfectBinaryTree.h>
#include <ctype.h>


int main( int argc, char* argv[])
{
    unsigned int height;
    
    if( argc == 2 && isdigit(argv[1][0]))
    {
        height = atoi( argv[1]);
    }
    else
    {
        height = 100;
    }

    perfectBinaryTree< unsigned int>* A = new perfectBinaryTreeVEB< unsigned int >(height);

    unsigned int numLeaves = pow2(height);
    
    for( unsigned int i = 0; i < numLeaves; i++)
    {
		for(unsigned int j = 0; j < i; j++)
		{
			increaseElements<unsigned int>* strategy = new increaseElements<unsigned int>( A, i);
        	search( A, strategy);
        	delete strategy;
		}
    }

    for( unsigned int i = 0; i < numLeaves; i++)
    {
        maximumElements<unsigned int>* strategy = new maximumElements<unsigned int>(i);
        search( A, strategy);
        std::cout << "Value " << i << " is located at position " <<  strategy->getResult() << std::endl;
        delete strategy;
    }

    std::ofstream out("out.dot");
    perfectBinaryTree< unsigned int>::nodeVisitor* visitor = new printDataVisitor<unsigned int>(out);
    printDot( A, out, false, visitor);
    out.close();
    delete visitor;

    return 0;
}
