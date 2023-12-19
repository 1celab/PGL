#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <Structs/Trees/perfectBinaryTreeVEB.h>
#include <Structs/Trees/perfectBinaryTreeBFS.h>
#include <Structs/Trees/perfectBinaryTree.h>

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

    perfectBinaryTree< int>* B = new perfectBinaryTreeBFS< int >(height,20);
    perfectBinaryTree< int>* A = new perfectBinaryTreeVEB< int >(height,20);
    
    std::ofstream outveb("veb.dot");
    perfectBinaryTree< int>::nodeVisitor* visitor = new printDataVisitor<int>(outveb);
    printDot( A, outveb, false, visitor);
    outveb.close();
    delete visitor;

    
    std::ofstream outbfs("bfs.dot");
    visitor = new printChildrenVisitor<int>(outbfs);
    printDot( B, outbfs, false, visitor);
    outbfs.close();
    delete visitor;
    
  
    delete A;
    delete B;

    return 0;
}
