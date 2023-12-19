#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <Structs/Trees/perfectBinaryTreeVEB.h>
#include <Structs/Trees/perfectBinaryTreeBFS.h>
#include <Structs/Trees/perfectBinaryTree.h>
#include <Structs/Trees/pmaDensityTree.h>
#include <Algorithms/perfectBinaryTree.h>
#include <Utilities/mersenneTwister.h>
#include <ctype.h>
#include <time.h>


#define DATASIZE 15



int main( int argc, char* argv[])
{
    unsigned int height, iterations, pos = 0;
    double random;
    timespec tS1, tS2;    

    std::cout << sizeof(long long) << std::endl;

    if( argc == 3 && isdigit(argv[1][0]) && isdigit(argv[2][0]))
    {
        height = atoi( argv[1]);
        iterations = atoi( argv[2]);
    }
    else
    {
        height = 5;
        iterations = 100;    
    }

	unsigned int size = pow2(height);
    std::cout << "Height: " << height << ", Size: " << size << ", Iterations: " << iterations << std::endl;

    perfectBinaryTree< unsigned int>* B = new perfectBinaryTreeVEB< unsigned int >(height);
    pmaDensityTree* A = new pmaDensityTree(height);

	
    for( unsigned int i = 0; i < size; i++)
    {
		updateSearchTree<unsigned int>* strategy = new updateSearchTree<unsigned int>( A, i, i);
        search( A, strategy);
        delete strategy;
        //B.set(i,data);
    }

    std::cout << "Tree A filled... Element size: " << sizeof(unsigned int) << "bytes \tMemory: " << double(A->memUsage())/1048576 << "MB\n";


	for( unsigned int i = 0; i < size; i++)
    {
		updateSearchTree<unsigned int>* strategy = new updateSearchTree<unsigned int>( B, i, i);
        search( B, strategy);
        delete strategy;
    }

    std::cout << "Tree B filled... Element size: " << sizeof(unsigned int) << "bytes \tMemory: " << double(B->memUsage())/1048576 << "MB\n";

	std::fstream in;

	in.open("../mersenneTwister/10M_random_numbers.out");
    for( unsigned int i = 0; i < iterations; i++)
    {
        in >> random; 
        pos = random * size;
        if( pos == size)
        {
            pos--;
        }

        simpleSearch<unsigned int>* strategy = new simpleSearch<unsigned int>(pos);
        search( B, strategy);
        delete strategy;
    }
    in.close();


    tS1.tv_sec = 0;
    tS1.tv_nsec = 0;
    clock_settime(CLOCK_PROCESS_CPUTIME_ID, &tS1);

    
    in.open("../mersenneTwister/10M_random_numbers.out");
    for( unsigned int i = 0; i < iterations; i++)
    {
        in >> random; 
        pos = random * size;
        if( pos == size)
        {
            pos--;
        }
        
		simpleSearch<unsigned int>* strategy = new simpleSearch<unsigned int>(pos);
        search( A, strategy);
        delete strategy;
    }
    in.close();

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tS1);
    double microsec1 = tS1.tv_sec * 1000000 + (double)tS1.tv_nsec/1000;
    std::cout << "Time VEB:\t" << microsec1/1000000 << "sec\t(" << microsec1/iterations << " microsec per search)\n";

	tS2.tv_sec = 0;
    tS2.tv_nsec = 0;
    clock_settime(CLOCK_PROCESS_CPUTIME_ID, &tS2);

    in.open("../mersenneTwister/10M_random_numbers.out");
    for( unsigned int i = 0; i < iterations; i++)
    {
        in >> random; 
        pos = random * size;
        if( pos == size)
        {
            pos--;
        }

        simpleSearch<unsigned int>* strategy = new simpleSearch<unsigned int>(pos);
        search( B, strategy);
        delete strategy;
    }
    in.close();

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tS2);
    double microsec2 = tS2.tv_sec * 1000000 + (double)tS2.tv_nsec/1000;
    std::cout << "Time BFS:\t" << microsec2/1000000 << "sec\t(" << microsec2/iterations << " microsec per search)\n";
    
    double speedup = ((double) microsec2 / microsec1) * 100;

    std::cout << "Speedup:\t" << speedup << "%\n";

	delete A;
	delete B;

    return 0;
}
