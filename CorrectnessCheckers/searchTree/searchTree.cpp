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
        height = 5;
    }

    perfectBinaryTree< unsigned int>* A = new perfectBinaryTreeBFS< unsigned int >(height);

    unsigned int numLeaves = pow2(height);
    
    for( unsigned int i = 0; i < numLeaves; i++)
    {
        updateSearchTree<unsigned int>* strategy = new updateSearchTree<unsigned int>( A, i, i);
        search( A, strategy);
        delete strategy;
    }

    std::ofstream out("out.dot");
    perfectBinaryTree< unsigned int>::nodeVisitor* visitor = new printDataVisitor<unsigned int>(out);
    printDot( A, out, false, visitor);
    out.close();
    delete visitor;

    for( unsigned int i = 0; i < numLeaves; i++)
    {
        simpleSearch<unsigned int>* strategy = new simpleSearch<unsigned int>(i);
        search( A, strategy);
        std::cout << "Value " << i << " is located at position " <<  strategy->getResult() << std::endl;
        delete strategy;
    }
 
    

    delete A;

    return 0;
}
