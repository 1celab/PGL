#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include <Utilities/timer.h>
#include <boost/program_options.hpp>
#include <boost/utility/enable_if.hpp>

#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <Structs/Graphs/forwardStarImpl.h>

#include "preprocessingSharc.h"

#include <Algorithms/basicGraphAlgorithms.h>
#include <Algorithms/ShortestPath/Astar/uniDijkstra.h>
#include <Algorithms/ShortestPath/Astar/bidDijkstra.h>
#include <Algorithms/ShortestPath/Astar/uniAstarEcl.h>
#include <Algorithms/ShortestPath/Astar/bidAstarAveEcl.h>

using namespace std;
namespace po = boost::program_options;

const char* algLabel[6] = { "", "sharc", "uni", "bid", "uniEcl", "bidAveEcl"};
const char* graphTypeLabel[4] = { "", "Adjacency", "PMA", "ForwardStar"};
unsigned int graphVariant, dijkstraVariant, reliableAlg, algTimestamp;
unsigned short levels = 3;
unsigned short vv[3] = { 256, 16, 4 };
std::vector<unsigned short> parts(&vv[0], &vv[0]+levels);

struct node: DefaultGraphItem
{
	node():x(0),y(0),dist(std::numeric_limits<unsigned int>::max()),pqitem(0),pred(0),timestamp(0),id(0)
	{	
	}

    unsigned int x,y;
    unsigned int dist, distBack;
    unsigned int pqitem, pqitemBack;
    void* pred;
    void* succ;
    unsigned int timestamp;
	unsigned int id;
    std::vector<unsigned int> cell;
};


struct edge: DefaultGraphItem
{
    edge():weight(0)
    {
    }
    
    unsigned int weight;
	std::vector<std::vector<bool> > flag;
};


template < typename GraphType, typename IdsVector>
void calcWeightsAndIds( GraphType& G, IdsVector& ids)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typedef typename GraphType::NodeDescriptor NodeDescriptor;
    typedef typename GraphType::EdgeIterator EdgeIterator;
    typedef typename GraphType::InEdgeIterator InEdgeIterator;
    
    NodeIterator u, v, lastNode;
    EdgeIterator e,lastEdge;
    InEdgeIterator k;
    ProgressStream edge_progress( G.getNumEdges());
    edge_progress.label() << "\tCalculating weights of " << G.getNumEdges() << " edges";
      
    Timer timer; 
    timer.start();
        
    for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
    {
	    for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
	    {
            v = G.target(e);
			e->weight = ceil(euclideanDistance( u->x, u->y, v->x, v->y));
            k = G.getInEdgeIterator(e);
            k->weight = e->weight;
            ++edge_progress;
        }
    }
    
    for( unsigned int i=1; i<=G.getNumNodes(); ++i)
    {
        u = G.getNodeIterator(ids[i]);
        u->id = i;
    }
    std::cout << "\tTime:\t" << timer.getElapsedTime() << std::endl;
}


template < typename GraphType, typename IdsVector>
void giveFlagsAndWeightToEdges( GraphType& G, const std::string& file, IdsVector& ids, unsigned int levels)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typedef typename GraphType::EdgeIterator EdgeIterator;

    NodeIterator u, lastNode;
    EdgeIterator e, lastEdge;
    //std::string map ="germany";
    std::string map ="luxembourg";
    std::ifstream in, in2;

    for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
    {
        u->cell.resize( levels, std::numeric_limits<unsigned int>::max());
	    for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
        {
	        e->flag.resize(levels);
            for( unsigned int l = 0; l < levels; ++l)
                e->flag[l].resize( parts[l], 0);
        }
    }

    
    for( unsigned int i = 0; i < levels; ++i)
    {
        std::ostringstream ostr;
        ostr << parts[i];
        std::string file2 = std::string(getenv("HOME")) + "/Projects/Graphs/DIMACS10/" + map + "/parts/" + map + ".osm.graph.part." + ostr.str();
        in.open(file2.c_str());
        for( u = G.beginNodes(); u != G.endNodes(); ++u)
	        in >> u->cell[i];
        
    }
    in.close();
    ProgressStream edge_progress( G.getNumEdges());
    edge_progress.label() << "\tGiveFlagsToEdges";

    in2.open(file.c_str(), std::ios::in | std::ios::binary);
    unsigned int c1 = 0, from = 0, to = 0;

    for( unsigned int i = 0; i < G.getNumEdges(); ++i)
    {        
        in2.read((char *)&from, sizeof(unsigned int));
        in2.read((char *)&to, sizeof(unsigned int));
        e = G.getEdgeIterator(ids[from], ids[to]);
        in2.read((char *)&e->weight, sizeof(unsigned int));

        for( unsigned int l = 0; l < levels; ++l)
        {
            for( unsigned int y = 0; y < parts[l]; ++y)
            {
                bool flag;
                in2.read((char *)&(flag), sizeof(bool));
                e->flag[l][y] = flag;
            }
        }
        ++edge_progress;
        ++c1;    
    }
    in2.close();
  
    for( unsigned int i=1; i<=G.getNumNodes(); ++i)
    {
        u = G.getNodeIterator(ids[i]);
        u->id = i;
    }
    std::cout<<"\tTotal edges " << c1 << std::endl;
}


template < typename GraphType>
void getGraphStats( GraphType& G)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typedef typename GraphType::EdgeIterator EdgeIterator;
    typedef typename GraphType::InEdgeIterator InEdgeIterator;
    typedef typename GraphType::SizeType SizeType;

    NodeIterator u,v,lastnode;
    EdgeIterator e,lastedge;
    InEdgeIterator k;

    double meanWeight = 0, stdWeight = 0, meanInDegree = 0, stdInDegree = 0, meanOutDegree = 0, stdOutDegree = 0;

    unsigned int inDegree = 0, outDegree = 0;

    unsigned int minWeight    = std::numeric_limits<double>::max();
    unsigned int maxWeight    = std::numeric_limits<double>::min();
    unsigned int minInDegree  = std::numeric_limits<double>::max();
    unsigned int maxInDegree  = std::numeric_limits<double>::min();
    unsigned int minOutDegree = std::numeric_limits<double>::max();
    unsigned int maxOutDegree = std::numeric_limits<double>::min();

    std::stringstream sstr;
    sstr << "Analysing the graph";
    ProgressBar edge_progress( 2* G.getNumEdges(), sstr.str());
    Timer timer; 
    timer.start();
    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
	    for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
	    {
            outDegree++;

            v = G.target(e);
            k = G.getInEdgeIterator(e);
            
            meanWeight += e->weight;

            ++edge_progress;
        }

        meanOutDegree += outDegree;

        if( outDegree > maxOutDegree)
            maxOutDegree = outDegree;

        if( outDegree < minOutDegree)
            minOutDegree = outDegree;

        outDegree = 0;      
    }

    meanWeight /= G.getNumEdges();
    meanOutDegree /= G.getNumNodes();

    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
	    for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
	    {
            stdWeight += pow( e->weight - meanWeight, 2);

            if( e->weight > maxWeight)
            {
                maxWeight = e->weight;
            }

            if( e->weight < minWeight)
            {
                minWeight = e->weight;
            }

            ++edge_progress;
            outDegree++;
        }
        stdOutDegree += pow( outDegree - meanOutDegree, 2); 
        outDegree = 0;
    }

    stdWeight = sqrt( stdWeight/ G.getNumEdges());
    stdOutDegree = sqrt( stdOutDegree/G.getNumNodes());

    minInDegree  = minOutDegree;
    maxInDegree  = maxOutDegree;
    meanInDegree = meanOutDegree;
    stdInDegree  = stdOutDegree;

    std::cout << "Graph stats"
              << "\n\tMax  weight: " << maxWeight
              << "\n\tMin  weight: " << minWeight
              << "\n\tMean weight: " << meanWeight
              << "\n\tStd  weight: " << stdWeight
              << "\n"
              << "\n\tMax  indegree: " << maxOutDegree
              << "\n\tMin  indegree: " << minOutDegree
              << "\n\tMean indegree: " << meanOutDegree
              << "\n\tStd  indegree: " << stdOutDegree
              << "\n"
              << "\n\tMax  outdegree: " << maxInDegree
              << "\n\tMin  outdegree: " << minInDegree
              << "\n\tMean outdegree: " << meanInDegree
              << "\n\tStd  outdegree: " << stdInDegree
              << "\n"
              << "\n\tdensity:" << (2.0 * G.getNumEdges()) / ( G.getNumNodes() * (G.getNumNodes() - 1.0) ) << "\n";
}


template<typename GraphType, typename AlgType, typename IdsVector>
void runQueries( GraphType& G,  AlgType& spAlg, IdsVector &ids, const std::string& queryFileName,  const std::string& logFileName)
{
    std::string map = "luxembourg";
    //std::string map ="germany";
    //read the queries from a file
    std::ifstream queryFile( queryFileName.c_str());

    if (!queryFile)
    {
        std::cerr << "\nerror: unable to open file [" << queryFileName << "]\n";
        exit(1);
    }

    unsigned int numQueries, minNodeID, maxNodeID;

    //get and drop the first line (pact : single-line comment)
    queryFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    queryFile >> numQueries;
    queryFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    //get and drop the third line (pact : single-line comment)
    queryFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    queryFile >> minNodeID >> maxNodeID;
    queryFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    //get and drop the fifth line (pact : single-line comment)
    queryFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    std::fstream logFile;

    //write the results
    logFile.open( logFileName.c_str(), std::ios::out);

    if (!logFile)
    {
        std::cerr << "\nerror: unable to open file [" << logFileName << "]\n";
        exit(1);
    }

    //pact : the first three lines with comments
    logFile << "%map:" << map << " graphType:" << graphTypeLabel[graphVariant]
            << " nodes:" << G.getNumNodes() << " edges:" << G.getNumEdges()
            << "\n%alg:" << algLabel[dijkstraVariant] << " queries:" << numQueries;

    logFile << "\n%time efficiency expansion SPLength NumSPNodes settledNodes visitedNodes\n";

    std::cout << "\nExperiment >  map:[" << map << "]  graphType:[" << graphTypeLabel[graphVariant] 
              << "]  alg:[" << algLabel[dijkstraVariant] << "]" << "  queries:[" << numQueries << "]";

    std::cout << "\n";

    ProgressBar show_progress( numQueries, std::string("\tComputing the shortest paths"));
    Timer timer, experimentTime;
    unsigned int sourceID, targetID;
    double executionTime, efficiency, spreading;

    double maxExecutionTime = std::numeric_limits<double>::min(), 
           maxEfficiency    = std::numeric_limits<double>::min(), 
           maxSpreading     = std::numeric_limits<double>::min(),
           maxSPLength      = std::numeric_limits<double>::min(),
           maxNumSPNodes    = std::numeric_limits<double>::min(),
           maxNumSettledNodes = std::numeric_limits<double>::min(),
           maxNumVisitedNodes = std::numeric_limits<double>::min();

    double minExecutionTime = std::numeric_limits<double>::max(), 
           minEfficiency    = std::numeric_limits<double>::max(), 
           minSpreading     = std::numeric_limits<double>::max(),
           minSPLength      = std::numeric_limits<double>::max(),
           minNumSPNodes    = std::numeric_limits<double>::max(),
           minNumSettledNodes = std::numeric_limits<double>::max(),
           minNumVisitedNodes = std::numeric_limits<double>::max();

    double meanExecutionTime = 0,
           meanEfficiency    = 0,
           meanSpreading     = 0,
           meanSPLength      = 0,
           meanNumSPNodes    = 0,
           meanNumSettledNodes = 0,
           meanNumVisitedNodes = 0;

    //std: standar Deviation
    double stdExecutionTime  = 0, 
           stdEfficiency     = 0, 
           stdSpreading      = 0, 
           stdSPLength       = 0, 
           stdNumSPNodes     = 0,
           stdNumSettledNodes = 0,
           stdNumVisitedNodes = 0;

    experimentTime.start();

    unsigned int SPLength, numSettledNodes, numVisitedNodes, numSPNodes;

    for( unsigned int i = 0; i < numQueries; i++)
    {
        queryFile >> sourceID >> targetID;
        
        timer.start();
        SPLength = spAlg.runQuery( G.getNodeIterator(ids[sourceID]), G.getNodeIterator(ids[targetID]));
        timer.stop();

        numSettledNodes = spAlg.getNumSettledNodes();
        numVisitedNodes = spAlg.getNumVisitedNodes();
        numSPNodes      = spAlg.getNumSPNodes( G.getNodeIterator(ids[targetID]));

        executionTime = 1000.0 * timer.getElapsedTime();          // ms
        efficiency    = 100.0  * numSPNodes/numSettledNodes;      // %
        spreading     = 100.0  * numVisitedNodes/G.getNumNodes(); // %

        meanExecutionTime   += executionTime;
        meanEfficiency      += efficiency;
        meanSpreading       += spreading;
        meanSPLength        += SPLength;
        meanNumSPNodes      += numSPNodes;
        meanNumSettledNodes += numSettledNodes;
        meanNumVisitedNodes += numVisitedNodes;

        if( maxExecutionTime < executionTime)
            maxExecutionTime = executionTime;

        if( minExecutionTime > executionTime)
            minExecutionTime = executionTime;

        if( maxEfficiency < efficiency)
            maxEfficiency = efficiency;

        if( minEfficiency > efficiency)
            minEfficiency = efficiency;

        if( maxSpreading < spreading)
            maxSpreading = spreading;

        if( minSpreading > spreading)
            minSpreading = spreading;

        if( maxSPLength < SPLength)
            maxSPLength = SPLength;

        if( minSPLength > SPLength)
            minSPLength = SPLength;

        if( maxNumSPNodes < numSPNodes)
            maxNumSPNodes = numSPNodes;

        if( minNumSPNodes > numSPNodes)
            minNumSPNodes = numSPNodes;

        if( maxNumSettledNodes < numSettledNodes)
            maxNumSettledNodes = numSettledNodes;

        if( minNumSettledNodes > numSettledNodes)
            minNumSettledNodes = numSettledNodes;

        if( maxNumVisitedNodes < numVisitedNodes)
            maxNumVisitedNodes = numVisitedNodes;

        if( minNumVisitedNodes > numVisitedNodes)
            minNumVisitedNodes = numVisitedNodes;

        //%time efficiency spreading distnace shortestPathLength settledNodes
        logFile << executionTime << " " 
                << efficiency<< " "
                << spreading << " "
                << SPLength << " "
                << numSPNodes << " "
                << numSettledNodes << " "
                << numVisitedNodes << "\n";

        ++show_progress;
    }

    queryFile.close();
    logFile.close();

    meanExecutionTime   /= numQueries;
    meanEfficiency      /= numQueries;
    meanSpreading       /= numQueries;
    meanSPLength        /= numQueries;
    meanNumSPNodes      /= numQueries;
    meanNumSettledNodes /= numQueries;
    meanNumVisitedNodes /= numQueries;

    //read the data
    logFile.open( logFileName.c_str(), std::ios::in);

    //get and drop the first three lines with the comments
    logFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
    logFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
    logFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
    
    show_progress.reset( numQueries, std::string("\tAnalysing experimental data"));

    for(unsigned int i = 0; i < numQueries; i++)
    {
        logFile >> executionTime >> efficiency >> spreading >> SPLength >> numSPNodes >> numSettledNodes >> numVisitedNodes;
        //logFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

        stdExecutionTime   += pow( executionTime - meanExecutionTime, 2);
        stdEfficiency      += pow( efficiency - meanEfficiency, 2);
        stdSpreading       += pow( spreading - meanSpreading, 2);
        stdSPLength        += pow( SPLength - meanSPLength, 2);
        stdNumSPNodes      += pow( numSPNodes - meanNumSPNodes, 2);
        stdNumSettledNodes += pow( numSettledNodes - meanNumSettledNodes, 2);
        stdNumVisitedNodes += pow( numVisitedNodes - meanNumVisitedNodes, 2);

        ++show_progress;
    }

    logFile.close();

    stdExecutionTime   = sqrt( stdExecutionTime/numQueries);
    stdEfficiency      = sqrt( stdEfficiency/numQueries);
    stdSpreading       = sqrt( stdSpreading/numQueries);
    stdSPLength        = sqrt( stdSPLength/numQueries);
    stdNumSPNodes      = sqrt( stdNumSPNodes/numQueries);
    stdNumSettledNodes = sqrt( stdNumSettledNodes/numQueries);
    stdNumVisitedNodes = sqrt( stdNumVisitedNodes/numQueries);

    //write the statistics of the results
    logFile.open( logFileName.c_str(), std::ios::out | std::ios::app);

    logFile << "$Statisitcs:\n"
            << "$executionΤime(ms) - efficiency(%) - spreading(%) - distance(m) - numSPNodes - numSettledNodes - numVisitedNodes\n"
            << "min:  " << minExecutionTime << " " << minEfficiency << " " << minSpreading << " " 
            << minSPLength << " " << minNumSPNodes << " " << minNumSettledNodes << " " << minNumVisitedNodes << "\n"
            << "max:  " << maxExecutionTime << " " << maxEfficiency << " " << maxSpreading << " " 
            << maxSPLength << " " << maxNumSPNodes << " " << maxNumSettledNodes << " " << maxNumVisitedNodes << "\n"
            << "mean: " << meanExecutionTime << " " << meanEfficiency << " " << meanSpreading << " " 
            << meanSPLength << " " << meanNumSPNodes << " " << meanNumSettledNodes << " " << meanNumVisitedNodes << "\n"
            << "std:  " << stdExecutionTime << " " << stdEfficiency << " " << stdSpreading << " " 
            << stdSPLength << " " << stdNumSPNodes << " " << stdNumSettledNodes << " " << stdNumVisitedNodes << "\n";
    logFile.close();
        
    std::cout << "\tTime spent for the experiment:\t" << experimentTime.getElapsedTime() << "sec\n";

    std::cout << "Statistics    time(ms)  efficiency  spreading  distance(m)   SPNodes     settledNodes  visitedNodes\n"
              << "\tmin:  " << std::setw(8) << std::left << minExecutionTime 
              << "  " << std::setw(10) << std::left << minEfficiency 
              << "  " << std::setw(9)  << std::left << minSpreading 
              << "  " << std::setw(12) << std::left << minSPLength 
              << "  " << std::setw(10) << std::left << minNumSPNodes
              << "  " << std::setw(12) << std::left << minNumSettledNodes
              << "  " << std::setw(12) << std::left << minNumVisitedNodes << "\n"

              << "\tmax:  " << std::setw(8) << std::left << maxExecutionTime 
              << "  " << std::setw(10) << std::left << maxEfficiency 
              << "  " << std::setw(9)  << std::left << maxSpreading 
              << "  " << std::setw(12) << std::left << maxSPLength 
              << "  " << std::setw(10) << std::left << maxNumSPNodes
              << "  " << std::setw(12) << std::left << maxNumSettledNodes
              << "  " << std::setw(12) << std::left << maxNumVisitedNodes << "\n"

              << "\tmean: " << std::setw(8) << std::left << meanExecutionTime 
              << "  " << std::setw(10) << std::left << meanEfficiency 
              << "  " << std::setw(9)  << std::left << meanSpreading
              << "  " << std::setw(12) << std::left << meanSPLength 
              << "  " << std::setw(10) << std::left << meanNumSPNodes
              << "  " << std::setw(12) << std::left << meanNumSettledNodes
              << "  " << std::setw(12) << std::left << meanNumVisitedNodes << "\n"

              << "\tstd:  " << std::setw(8) << std::left << stdExecutionTime 
              << "  " << std::setw(10) << std::left << stdEfficiency 
              << "  " << std::setw(9)  << std::left << stdSpreading 
              << "  " << std::setw(12) << std::left << stdSPLength 
              << "  " << std::setw(10) << std::left << stdNumSPNodes
              << "  " << std::setw(12) << std::left << stdNumSettledNodes
              << "  " << std::setw(12) << std::left << stdNumVisitedNodes << "\n";
}


template< typename GraphType, typename IdsVector>
void runExperiments( GraphType& G, IdsVector &ids, std::string map)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    //MersenneTwister gen;
    
    //initilization : reset the status of each node
    for(typename GraphType::NodeIterator u = G.beginNodes(); u != G.endNodes(); ++u)
    {
        u->timestamp = 0;
        u->dist = std::numeric_limits<unsigned int>::max();
        u->distBack = std::numeric_limits<unsigned int>::max();
        u->pred = 0;
        u->succ = 0;
    }
    algTimestamp = 0;
    
    std::string basePath = std::string(getenv("HOME")) + "/Projects/Graphs/DIMACS10/" + map + "/";
    const std::string queryFileName = basePath + "queries.dat";
    std::string logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";

    switch( dijkstraVariant)
    {  
        case 1:
        {
            PreprocessingSharc<GraphType> spAlg_sh( G, levels, &algTimestamp, parts);
            runQueries( G, spAlg_sh, ids, queryFileName, logFileName);
        }   break;
        case 2:
        {
			Dijkstra<GraphType> spAlg_pd(G, &algTimestamp);
            runQueries( G, spAlg_pd, ids, queryFileName, logFileName);
        }   break;
        case 3:
        {
            BidirectionalDijkstra<GraphType> spAlg_bd(G, &algTimestamp);
            runQueries( G, spAlg_bd, ids, queryFileName, logFileName);
        }   break;
        case 4:
        {
            UniAstarEcl<GraphType> spAlg_ua(G, &algTimestamp);
            runQueries( G, spAlg_ua, ids, queryFileName, logFileName);
        }   break;
        case 5:
        {
            BidAstarAveEcl<GraphType> spAlg_ba(G, &algTimestamp);
            runQueries( G, spAlg_ba, ids, queryFileName, logFileName);
        }   break;

        default:
        {
            dijkstraVariant = 1;
            
            PreprocessingSharc<GraphType> spAlg1( G, levels, &algTimestamp, parts);
	        logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries( G, spAlg1, ids, queryFileName, logFileName);

            dijkstraVariant = 2;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            Dijkstra<GraphType> spAlg2(G, &algTimestamp);                
            runQueries( G, spAlg2, ids, queryFileName, logFileName);

            dijkstraVariant = 3;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            BidirectionalDijkstra<GraphType> spAlg3(G, &algTimestamp);            
            runQueries( G, spAlg3, ids, queryFileName, logFileName);

            dijkstraVariant = 4;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            UniAstarEcl<GraphType> spAlg4(G, &algTimestamp);            
            runQueries( G, spAlg4, ids, queryFileName, logFileName);

            dijkstraVariant = 5;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            BidAstarAveEcl<GraphType> spAlg5(G, &algTimestamp);
            runQueries( G, spAlg5, ids, queryFileName, logFileName);
        
            dijkstraVariant = 0;
        }   break;
    }
}


void checkCorrectness(std::string basePath, std::string map)
{
    const string queryFileName = basePath + "queries.dat";
    const string errorLogFileName = "./stats/" + map + "/" + string(graphTypeLabel[graphVariant]) + "/errors.dat";
    const string reliableAlgLogFileName = "./stats/" + map + "/" + string(graphTypeLabel[graphVariant]) + "/" + string(algLabel[reliableAlg]) + ".dat";
    const string algLogFileName = "./stats/" + map + "/" + string(graphTypeLabel[graphVariant]) + "/" + string(algLabel[dijkstraVariant]) + ".dat";

    //read the queries from a file
    ifstream queryFile( queryFileName.c_str());
    ifstream algLogFile( algLogFileName.c_str());
    ifstream reliableAlgLogFile( reliableAlgLogFileName.c_str());
    ofstream errorLogFile( errorLogFileName.c_str());

    queryFile.exceptions( std::ifstream::failbit | std::ifstream::badbit);
    algLogFile.exceptions( std::ifstream::failbit | std::ifstream::badbit);
    reliableAlgLogFile.exceptions( std::ifstream::failbit | std::ifstream::badbit);
    errorLogFile.exceptions( std::ifstream::failbit | std::ifstream::badbit);

    if (!queryFile)
    {
        std::cerr << "\nerror: unable to open file [" << queryFileName << "]\n";
        exit(1);
    }

    if (!algLogFile)
    {
        std::cerr << "\nerror: unable to open file [" << algLogFileName << "]\n";
        exit(1);
    }

    if (!reliableAlgLogFile)
    {
        std::cerr << "\nerror: unable to open file [" << reliableAlgLogFileName << "]\n";
        exit(1);
    }

    if (!errorLogFile)
    {
        std::cerr << "\nerror: unable to open file [" << errorLogFileName << "]\n";
        exit(1);
    }

    unsigned int numQueries = 0, numIncorrectResults = 0;
    Timer timer;
    timer.start();

    //get and drop the first line (pact : single-line comment)
    queryFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    queryFile >> numQueries;
    queryFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    queryFile.close();

    //get and drop the first three lines with the comments
    algLogFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
    algLogFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
    algLogFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    //get and drop the first three lines with the comments
    reliableAlgLogFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
    reliableAlgLogFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
    reliableAlgLogFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    ProgressBar show_progress( numQueries, std::string("\tComparing shortest path lengths"));

    double ignoredField;
    unsigned int SPLength1, SPLength2;

    try
    {
        for(unsigned int i = 0; i < numQueries; i++)
        {
            algLogFile >> ignoredField >> ignoredField >> ignoredField >> SPLength1;
            algLogFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

            reliableAlgLogFile >> ignoredField >> ignoredField >> ignoredField >> SPLength2;
            reliableAlgLogFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

            if( SPLength1 != SPLength2)
            {
                errorLogFile << algLabel[dijkstraVariant] << "[" << SPLength1 << "] - " 
                             << algLabel[reliableAlg] << "[" << SPLength2 << "] - " << "query[" << i << "]\n";

                numIncorrectResults++;
            }

            ++show_progress;
        }
    }

    catch (std::ifstream::failure e) 
    {
        std::cerr << "\nError comparing files: [" << algLogFileName << "] & [" << reliableAlgLogFileName << "]."
                  << "\nThe reason :" << e.what() << "n";
        return;
    }

    timer.stop();

    std::cout << "\tTime spent to compare the computed shortest path lengths of [" << algLabel[reliableAlg] 
              << "] and ["  << algLabel[dijkstraVariant] << "] :\t" << timer.getElapsedTime() << "sec\n"
              << "\tNumber of queries:\t" << numQueries << "\n"
              << "\tNumber of non-optimal paths:\t" << numIncorrectResults << "\n\n";
}


typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   pmaGraph;
typedef DynamicGraph< ForwardStarImpl, node, edge>         fsGraph;

typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;

typedef GraphReader< Graph> AdjReader;
typedef GraphReader< pmaGraph> PmaReader;
typedef GraphReader< fsGraph> FsReader;

int main( int argc, char* argv[])
{    
    std::string map ="luxembourg";
    std::string basePath = std::string(getenv("HOME")) + "/Projects/Graphs/DIMACS10/" + map + "/";
    std::string queryFileName = std::string(getenv("HOME")) + "/Projects/Graphs/DIMACS10/" + map + "/queries.dat";
    std::string file = "EdgesWithFlags_luxembourg.txt";

    Graph G;
    pmaGraph pmaG;
    fsGraph  fsG;
	AdjReader* reader = 0;
	PmaReader* pmaReader = 0;
    FsReader* fsReader = 0;
    unsigned int graphVariant = 0;
    
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("shortest path algorithm,a", po::value< unsigned int>(), "Choose: All[0], sharc[1], plain-dijkstra[2], plain-bidirectional[3], uni-A*-ecl[4], bid-A*-ecl-ave[5]. Default:0")
        ("graphtype,g", po::value< unsigned int>(), "graph type. All[0], Adjacency List[1], Packed Memory Graph[2], Forward Star [3]. Default:0")
        ("map,m", po::value< std::string>(), "input map. The name of the map to read. Maps should reside in '$HOME/Projects/Graphs/maps/' and should consist of 2 files, both with the same map name prefix, and suffixes 'osm.graph' and 'osm.xyz'. Default:'luxembourg'")
        ("queries,q", "The input queries are in the file queries.dat. The file must be placed in '$HOME/Projects/Graphs/DIMACS10/'map name'/'")
        ("check correctness,c", po::value< unsigned int>(), "Based on a reliable shortest path algorithm. The shortest path results of the queries of any selected shortest path algorithm from the above field -g will be compared with the ones of the reliable shortest path algorithm -c.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if (vm.empty()) {
        std::cout << desc << "\n";
        return 0;
    }

    if ( vm.count("shortest path algorithm"))
    {
        dijkstraVariant = vm["shortest path algorithm"].as<unsigned int>();
    }

    if (vm.count("graphtype"))
    {
        graphVariant = vm["graphtype"].as<unsigned int>();
    }

	if (vm.count("map"))
    {
        map = vm["map"].as<std::string>();
    }

    if (vm.count("check correctness"))
    {
        reliableAlg = vm["check correctness"].as<unsigned int>();

        if( dijkstraVariant == 0)
        {
            for( dijkstraVariant = 1; dijkstraVariant< 6; ++dijkstraVariant)
                if( dijkstraVariant != reliableAlg)
                    checkCorrectness( basePath, map);
                else
                    continue;
        }
        else
        {   
            if( dijkstraVariant != reliableAlg)
                    checkCorrectness( basePath, map);
        }
        return 0;
    }

    Timer timer;
        
    switch( graphVariant)
    {
        case 1:  //ADJ
        {
            if( dijkstraVariant == 1)
            {            
                reader = new DIMACS10Reader<Graph>( basePath + map + ".osm.graph.modified", basePath + map + ".osm.xyz.modified");
                timer.start();
                G.read(reader);
                std::cout << "\tGraph has " << (double)G.memUsage()/1048576.0 << " Mbytes. Time spent to read: " 
                          << timer.getElapsedTime() << "sec" << std::endl;
                giveFlagsAndWeightToEdges( G, file , reader->getIds(), levels);
				getGraphStats(G);
                runExperiments( G, reader->getIds(), map);
                reader->getIds().clear();
                G.clear();
            }
            else
            {
                reader = new DIMACS10Reader<Graph>( basePath + map + ".osm.graph", basePath + map + ".osm.xyz");
                timer.start();
                G.read(reader);
                std::cout << "\tGraph has " << (double)G.memUsage() / 1048576.0 << " Mbytes. Time spent to read: " 
                          << timer.getElapsedTime() << "sec" << std::endl;
                std::cout << "IDs container capacity: "
                          << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
                calcWeightsAndIds( G, reader->getIds());
				getGraphStats(G);
                runExperiments( G, reader->getIds(), map);
                reader->getIds().clear();
                G.clear();
            }
        }   break;
        case 2:  //PMG
        {
            if( dijkstraVariant == 1)
            { 
                pmaReader = new DIMACS10Reader<pmaGraph>( basePath + map + ".osm.graph.modified",basePath + map + ".osm.xyz.modified");
                timer.start();
                pmaG.read(pmaReader);
                pmaG.compress();
                std::cout << "\tGraph has " << (double)pmaG.memUsage()/1048576.0 << " Mbytes. Time spent to read: " 
                          << timer.getElapsedTime() << "sec" << std::endl;
                giveFlagsAndWeightToEdges( pmaG, file, pmaReader->getIds(), levels);
				getGraphStats(pmaG);
                runExperiments( pmaG, pmaReader->getIds(), map);
                pmaReader->getIds().clear();
                pmaG.clear();
            }
            else
            {
                pmaReader = new DIMACS10Reader<pmaGraph>( basePath + map + ".osm.graph",basePath + map + ".osm.xyz");
                timer.start();
                pmaG.read(pmaReader);
                pmaG.compress();
                std::cout << "Graph has " << (double)G.memUsage() / 1048576.0 << " Mbytes. Time spent to read:\t" 
                          << timer.getElapsedTime() << "sec" << std::endl;
                std::cout << "IDs container capacity: "
                          << (sizeof(NodeDescriptor) * pmaReader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
                calcWeightsAndIds( G, pmaReader->getIds());
				getGraphStats(pmaG);
                runExperiments( G, pmaReader->getIds(), map);
                pmaReader->getIds().clear();
                pmaG.clear();
            }
        }   break;
        case 3:  //FSG
        {
            if( dijkstraVariant == 1)
            { 
                fsReader = new DIMACS10Reader<fsGraph>( basePath + map + ".osm.graph.modified", basePath + map + ".osm.xyz.modified");
                timer.start();
                fsG.read(fsReader);
                std::cout << "\tGraph has " << (double)pmaG.memUsage()/1048576.0 << " Mbytes. Time spent to read: " 
                          << timer.getElapsedTime() << "sec" << std::endl;
                giveFlagsAndWeightToEdges( fsG, file, fsReader->getIds(), levels);
                runExperiments( fsG, fsReader->getIds(), map);
                fsReader->getIds().clear();
                fsG.clear();
            }
            else
            {
                fsReader = new DIMACS10Reader<fsGraph>( basePath + map + ".osm.graph", basePath + map + ".osm.xyz");
                timer.start();
                fsG.read(fsReader);
                std::cout << "Graph has " << (double)fsG.memUsage() / 1048576.0 << " Mbytes. Time spent to read: " 
                          << timer.getElapsedTime() << "sec" << std::endl;
                std::cout << "IDs container capacity: "
                          << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
                calcWeightsAndIds( fsG, fsReader->getIds());
                runExperiments( fsG, fsReader->getIds(), map);
                fsReader->getIds().clear();
                fsG.clear();
            }
        }   break;
        
        default:
        {
            if( dijkstraVariant == 1)
            { 
                //ADJ
                reader = new DIMACS10Reader<Graph>( basePath + map + ".osm.graph.modified", basePath + map + ".osm.xyz.modified");
                timer.start();
                G.read(reader);
                std::cout << "\tGraph has " << (double)G.memUsage()/1048576.0 << " Mbytes. Time spent to read: " 
                          << timer.getElapsedTime() << "sec" << std::endl;
                giveFlagsAndWeightToEdges( G, file, reader->getIds(), levels);
                runExperiments( G, reader->getIds(), map);
                reader->getIds().clear();
                G.clear();

                //PMG
                pmaReader = new DIMACS10Reader<pmaGraph>( basePath + map + ".osm.graph.modified",basePath + map + ".osm.xyz.modified");
                timer.start();
                pmaG.read(pmaReader);
                pmaG.compress();
                std::cout << "\tGraph has " << (double)pmaG.memUsage()/1048576.0 << " Mbytes. Time spent to read: " << timer.getElapsedTime() << "sec" << std::endl;
                giveFlagsAndWeightToEdges( pmaG, file, pmaReader->getIds(), levels);
                runExperiments( pmaG, pmaReader->getIds(), map);
                pmaReader->getIds().clear();
                pmaG.clear();

                //FSG
                fsReader = new DIMACS10Reader<fsGraph>( basePath + map + ".osm.graph.modified", basePath + map + ".osm.xyz.modified");
                timer.start();
                fsG.read(fsReader);
                std::cout << "\tGraph has " << (double)fsG.memUsage()/1048576.0 << " Mbytes. Time spent to read: " << timer.getElapsedTime() << "sec" << std::endl;
                giveFlagsAndWeightToEdges( fsG, file, fsReader->getIds(), levels);
                runExperiments( fsG, fsReader->getIds(), map);
                fsReader->getIds().clear();
                fsG.clear();
            }
            else
            {
                //ADJ
                reader = new DIMACS10Reader<Graph>( basePath + map + ".osm.graph", basePath + map + ".osm.xyz");
                timer.start();
                G.read(reader);
                std::cout << "Graph has " << (double)G.memUsage() / 1048576.0 << " Mbytes. Time spent to read:\t" 
                          << timer.getElapsedTime() << "sec" << std::endl;
                std::cout << "IDs container capacity: "
                          << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
                calcWeightsAndIds( G, reader->getIds());
                runExperiments( G, reader->getIds(), map);
                reader->getIds().clear();
                G.clear();
                
                //PMG
                pmaReader = new DIMACS10Reader<pmaGraph>( basePath + map + ".osm.graph",basePath + map + ".osm.xyz");
                timer.start();
                pmaG.read(pmaReader);
                pmaG.compress();
                std::cout << "Graph has " << (double)G.memUsage() / 1048576.0 << " Mbytes. Time spent to read:\t" 
                          << timer.getElapsedTime() << "sec" << std::endl;
                std::cout << "IDs container capacity: "
                          << (sizeof(NodeDescriptor) * pmaReader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
                calcWeightsAndIds( G, pmaReader->getIds());
                runExperiments( G, pmaReader->getIds(), map);
                pmaReader->getIds().clear();
                G.clear();
               
                //FSG
                fsReader = new DIMACS10Reader<fsGraph>( basePath + map + ".osm.graph", basePath + map + ".osm.xyz");
                timer.start();
                fsG.read(fsReader);
                std::cout << "Graph has " << (double)fsG.memUsage() / 1048576.0 << " Mbytes (without landmarks). Time spent to read:\t" 
                          << timer.getElapsedTime() << "sec" << std::endl;
                std::cout << "IDs container capacity: "
                          << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
                calcWeightsAndIds( fsG, fsReader->getIds());
                runExperiments( fsG, fsReader->getIds(), map);
                fsReader->getIds().clear();
                fsG.clear();
            }
        }   break;
    }
    
    std::cout << "\tExperiments at\n";
    std::cout << "\tmap:[" << map << "] graphType:[" << graphVariant << "] shortest path algorithm:[" << dijkstraVariant << "]" << std::endl;

    return 0;
}
