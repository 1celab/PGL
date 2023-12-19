#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#include <Structs/Trees/priorityQueue.h>
#include <vector>
#include <Utilities/timer.h>


unsigned int n,iterations,resolution;
unsigned int* descriptors;
typedef PriorityQueue< unsigned int, long double, ExplicitVebStorage> PriorityQueueType;
typedef PriorityQueueType::PQItem PQItem;
PQItem pqIt;


void comparePQ( PriorityQueueType& PQ)
{
    for( unsigned int i = 0; i < n; ++i)
    {
        if( !(i%resolution) )
        std::cout << "\r" << i / resolution << "%" <<std::flush;
        pqIt = PQ.min();
        PQ.popMin(); 
        PQ.insert( 1, 1, &descriptors[n-1]); 
    }
    std::cout << "\rdone!"  <<std::endl;
}


int main( int argc, char* argv[])
{   
    if( argc != 3 )
    {
        n = 30;
        resolution = n/100;
    }
    else
    {
        n = atoi( argv[2]);
    }
    
    std::vector< unsigned int*> addresses;
    
    //float T1, T2;

    
    
    PriorityQueueType PQ;

    /*for( unsigned int i = 0; i < 64; ++i)
    {
        PQ.increaseSize();
    }
    PQ.printDot("prio.dot");

    return 0;
*/
    Timer timer;

    descriptors = new unsigned int[n];
    //std::cout << "Allocating " << toMb(n) << "Mb\n";
    //unsigned int* results = new unsigned int[n];
    //T1=leda::used_time();
    
    timer.start();
    for( unsigned int i = 0; i < n; ++i)
    {
        //if( !(i%resolution) )
        //std::cout << "\r" << i / resolution << "%" <<std::flush;
        PQ.insert( n-i, n-i, &descriptors[i]); 
    }

    PQ.printGraphviz("prio.dot");
    std::cout << "\rdone!" <<std::endl;
    std::cout << "interval (s): " << timer.getElapsedTime() << std::endl;
    //T2=leda::used_time(T1);
  
    //std::cout << T2 << std::endl;
    
    

    timer.start();
    comparePQ(PQ);
    std::cout << "interval (s): " << timer.getElapsedTime() << std::endl;

/*

    unsigned int minValue = 0;
    //T1=leda::used_time();
    timer.start();
    for( unsigned int i = 0; i < n; ++i)
    {
        if( !(i%resolution) )
        std::cout << "\r" << i / resolution << "%" <<std::flush;
        pqIt = PQ.min();
        assert( minValue <= pqIt.getKey());
        minValue = pqIt.getKey();
        PQ.popMin(); 
    }
    std::cout << "\rdone!"  <<std::endl;
    std::cout << "interval (s): " << timer.getElapsedTime() << std::endl;
    //T2=leda::used_time(T1);

    //std::cout << T2 << std::endl;    

    assert( PQ.empty());
    
    delete [] descriptors;
    //delete [] results;
    //in.close();*/
    return 0;
}
