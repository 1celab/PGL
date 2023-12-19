#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#include <Structs/Trees/priorityQueue.h>
#include <vector>
#include <Utilities/Timer.h>
//#include <zoom/zoom.h>


/* zoom assert macro exits if status comes back as anything other than 'ok' 
#define ZM_ASSERT(fn, r) { \
	if(result != ZM_OK) { \
		fprintf(stderr, "ERROR: %s - %s [%s:%d]\n", fn, ZMErrorName(r), __FILE__, __LINE__); \
		exit(-1); \
	} \
}
*/

unsigned int n,iterations,resolution;
unsigned int* descriptors;
typedef CompleteBinaryTree< unsigned int, VebStorage> TreeType;
typedef TreeType::Node Node;


int main( int argc, char* argv[])
{   
    /*
    ZMError result;
    // Connect to profiling daemon process. 
	result = ZMConnect();
	ZM_ASSERT("ZMConnect", result);
	
	// Check if zoom is already sampling - if true, 
	// the following ZMStartSession call will fail. 
	result = ZMIsSampling();
	printf("ZMIsSampling: %s\n", (result ? "true" : "false"));*/

    if( argc != 3 )
    {
        n = 10000000;
        resolution = n/100;
    }
    else
    {
        n = atoi( argv[2]);
    }
    
    std::fstream in;

    unsigned int height = floorLog2( n) - 1; 

    TreeType T( height);
    Node u = T.getRootNode();
    Timer timer;
    double random;

    std::cout << randomNumbersFile << "\n";
    in.open( randomNumbersFile.c_str());

    std::vector<double> randomNumbers;
    for( unsigned int i = 0; i < n; ++i)
    {
        if( !(i%resolution) )
        std::cout << "\rReading random numbers..." << i / resolution << "%" <<std::flush;
        in >> random; 
        randomNumbers.push_back(random);
    }
    std::cout << "\rReading random numbers...done!"  <<std::endl;
    in.close();

    timer.start();
    
    /* //Start profiling. 
	result = ZMStartSession();
	ZM_ASSERT("ZMStartSession", result);*/

    unsigned int sum,j;
    for( unsigned int i = 0; i < n; ++i)
    {
        if( !(i%resolution) )
        std::cout << "\r" << i / resolution << "%" <<std::flush;
        u.setAtRoot();
        
        sum = 0;
        j = i;
        while( !u.isLeaf())
        {
            if( randomNumbers[j] > 0.5)
            {
                u.right();
            }
            else
            {
                u.left();
            }
            *u += i;
            ++j;
        }    

        while( !u.isRoot())
        {
            u.parent();
        }
    }
    std::cout << "\rdone!"  <<std::endl;
    std::cout << "interval (s): " << timer.getElapsedTime() << std::endl;
   

    /* Stop profiling.
	result = ZMStopSession();
	ZM_ASSERT("ZMStopSession", result);
	
	/* Disconnect from the profiling daemon process. 
	result = ZMDisconnect();
	ZM_ASSERT("ZMDisconnect", result);*/

    return 0;
}
