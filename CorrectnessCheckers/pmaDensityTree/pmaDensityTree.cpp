#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <Algorithms/binaryTreeVisitors.h>
#include <Structs/Trees/perfectBinaryTreePMA.h>

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


	perfectBinaryTreePMA<pmaTreeData>* A = new perfectBinaryTreePMA<pmaTreeData>(height,pow2(height + 3));
    perfectBinaryTreePMA<pmaTreeData>::nodeVisitor* visitor;
    perfectBinaryTreePMA<pmaTreeData>::node* u;

	std::fstream in;
	in.open("../../ResultGenerators/mersenneTwister/10M_random_numbers.out");
	for( unsigned int i = 0; i < pow2(height); i++)
    {
		double random;
        in >> random; 
        unsigned int elements = random * 10;
        if( elements == 10)
        {
            elements--;
        }
		
        u = A->getRoot();
		u->setAtPos(0,i);
		visitor = new settingCardinalityVisitor<pmaTreeData>( A, u, elements);		
		searchUpwards( A, u, visitor);		
		delete u;
		delete visitor;
    }	
	in.close();

    //REBALANCE
    u = A->getRoot();
    //u->setAtPos(3,1);
    visitor = new settingCardinalityVisitor<pmaTreeData>( A, u, (*(*u)).m_cardinality + 1);
    searchUpwards( A, u, visitor);       
    delete visitor; 
    rebalancingElementsVisitor<pmaTreeData>* vis = new rebalancingElementsVisitor<pmaTreeData>(u);
	dfs( A, u, vis);		
    for( unsigned int i = 0; i < vis->getLeafElements().size(); i++)
    {
        std::cout << vis->getLeafElements().at(i) << " ";
    }	
    std::cout << std::endl;
    delete u;
	delete vis;


    //SEARCH
	findInsertionNodeStrategy<pmaTreeData>* strategy = new findInsertionNodeStrategy<pmaTreeData>(54);
	u = A->getRoot();        	
	searchDownwards( A, u, strategy);    
	std::cout << u->getHorizontalIndex() << ", " << strategy->getPmaIndex() << std::endl;
    delete strategy;
	delete u;

    //PRINT
    std::ofstream out("out.dot");
    visitor = new printDataVisitor<pmaTreeData>(out);
    printDot( A, out, false, visitor);
    out.close();
    delete visitor;
    
    delete A;
	//delete u;
    return 0;
}
