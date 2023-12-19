#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <cmath>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/forwardStarImpl.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <boost/utility/enable_if.hpp>
#include <iomanip>
#include <Utilities/geographic.h>
#include <Utilities/graphGenerators.h>
#include <Utilities/timer.h>
#include <boost/program_options.hpp>

#include <Algorithms/basicGraphAlgorithms.h>
#include <Algorithms/ShortestPath/Astar/uniDijkstra.h>
#include <Algorithms/ShortestPath/Astar/uniAstarEcl.h>
#include <Algorithms/ShortestPath/Astar/uniAstarLmk.h>
#include <Algorithms/ShortestPath/Astar/bidAstarAveLmk.h>
#include <Algorithms/ShortestPath/Astar/bidAstarMaxLmk.h>
#include <Algorithms/ShortestPath/Astar/bidAstarSymLmk.h>
#include <Algorithms/ShortestPath/Astar/bidAstarAveEcl.h>
#include <Algorithms/ShortestPath/Astar/bidAstarMaxEcl.h>
#include <Algorithms/ShortestPath/Astar/bidAstarSymEcl.h>
#include <Algorithms/ShortestPath/Astar/bidDijkstra.h>

//#include <Algorithms/ShortestPath/Astar/uniAstarLmk+Ecl.h>
//#include <Algorithms/ShortestPath/Astar/bidAstarAveLmk+Ecl.h>
//#include <Algorithms/ShortestPath/Astar/bidAstarMaxLmk+Ecl.h>
//#include <Algorithms/ShortestPath/Astar/bidAstarSymLmk+Ecl.h>

#include <Algorithms/ShortestPath/Astar/LandmarkGenerator/lmkGen.h>

namespace po = boost::program_options;

std::string basePath, map;

//the number of the active landmarks that will be used for each shortest path query
unsigned int numActiveLandmarks;

const char* algLabel[11] = { "", "uni", "bid", 
                                 "uniEcl", "bidSymEcl", "bidMaxEcl", "bidAveEcl",
                                 "uniLmk", "bidSymLmk", "bidMaxLmk", "bidAveLmk"};

const char* graphTypeLabel[4] = { "", "Adjacency", "PMA", "ForwardStar"};

unsigned int graphVariant, dijkstraVariant, reliableAlg;

unsigned int algTimestamp;

unsigned int format;

union LandmarkData
{
    unsigned int distanceToLandmark;
    unsigned int distanceFromLandmark;
};

struct node: DefaultGraphItem
{
	node():x(0),y(0),dist(std::numeric_limits<unsigned int>::max()),pqitem(0),pred(0),timestamp(0)
	{	
	}

    /*
    void print( std::ofstream& out)
    {
        out << "\\|" << dist;
        return;
    }

    void setProperty( const std::string& name, const std::string& value)
    {
        DefaultGraphItem::setProperty( name, value);
        if( !name.compare("x"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>x;
        }
        if( !name.compare("y"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>y;
        }
    }

    void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "x" << valueDelimiter << x << propertyDelimiter;
        out << "y" << valueDelimiter << y << propertyDelimiter;
    }
    
    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"x\":" << x << "," << "\"y\":" << y << "";
    }
    */

    /*TODO landmark heuristics <-> euclidean heuristics
    union
    {
        unsigned int x;
        unsigned int dist;
    };
    union
    {
        unsigned int y;
        unsigned int distBack;
    };*/

    unsigned int x, y;
    unsigned int dist, distBack;
    unsigned int pqitem, pqitemBack;

    void* pred;
    void* succ;
    unsigned int timestamp;
    
    std::vector<LandmarkData> landmark; 
    
};


struct edge: DefaultGraphItem
{
	edge():weight(0)
	{	
	}

    /*
    void print( std::ofstream& out)
    {
        out << weight;
        return;
    }

    void setProperty( const std::string& name, const std::string& value)
    {
        DefaultGraphItem::setProperty( name, value);
        if( !name.compare("weight"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>weight;
        }
    }

    void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "w" << valueDelimiter << weight << propertyDelimiter;
    }

    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"w\":" << weight << "";
    }
    */

    unsigned int weight;
};


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


template < typename GraphType>
void calcWeights( GraphType& G)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typedef typename GraphType::EdgeIterator EdgeIterator;
    typedef typename GraphType::InEdgeIterator InEdgeIterator;
    typedef typename GraphType::SizeType SizeType;

    NodeIterator u,v,lastnode;
    EdgeIterator e,lastedge;
    InEdgeIterator k;

    unsigned int max = 0;
    //unsigned int max_ux, max_uy, max_vx, max_vy, u_id, v_id;

    std::stringstream sstr;
    sstr << "Calculating weights of " << G.getNumEdges() << " edges";
    ProgressBar edge_progress( G.getNumEdges(),sstr.str());
    Timer timer; 
    timer.start();
    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
	    for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
	    {
            
            v = G.target(e);
            k = G.getInEdgeIterator(e);
            
            e->weight = euclideanDistance( u->x, u->y, v->x, v->y);
            k->weight = e->weight;

            if( e->weight > max)
            {
                //max_ux = u->x;
                //max_uy = u->y;  
                //max_vx = v->x;
                //max_vy = v->y;
                //u_id = G.getRelativePosition(u);
                //v_id = G.getRelativePosition(v);
                max = e->weight;
            }
            ++edge_progress;
            //std::cout << u->lat << " " << v->lat << std::endl;
        }
    }

    std::cout << "\tMax weight: " << max << "\tTime:\t" << timer.getElapsedTime() << std::endl;
    //std::cout << "( " << max_ux << ", " << max_uy << ") " << u_id << "->" << v_id << " ( " << max_vx << ", " << max_vy << ")\n";
}


template< typename shortestPathAlg, typename GraphType>
void runQueries( GraphType& G, const std::vector<typename GraphType::NodeDescriptor>& ids,
                const std::string& queryFileName, const std::string& logFileName)
{
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

    if(dijkstraVariant > 6)
        logFile << "  landmarks:[loaded:" << G.chooseNode()->landmark.size() 
                << ", activated:" << numActiveLandmarks << "]";

    logFile << "\n%time efficiency expansion SPLength NumSPNodes settledNodes visitedNodes\n";

    //run the selected shortest path algorithm
    shortestPathAlg spAlg(G, &algTimestamp);

    std::cout << "\nExperiment >  map:[" << map << "]  graphType:[" 
              << graphTypeLabel[graphVariant] << "]  alg:[" << algLabel[dijkstraVariant] << "]"
              << "  queries:[" << numQueries << "]";

    if(dijkstraVariant > 6)
        std::cout << "  landmarks:[loaded:" << G.chooseNode()->landmark.size() 
                  << ", activated:" << numActiveLandmarks << "]";

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

    logFile << "%Statisitcs:\n"
            << "%executionΤime(ms) - efficiency(%) - spreading(%) - distance(m) - numSPNodes - numSettledNodes - numVisitedNodes\n"
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


template< typename GraphType>
void runExperimentsAt( GraphType& G, std::vector<typename GraphType::NodeDescriptor>& ids)
{

    if(dijkstraVariant == 0 || dijkstraVariant > 6)
    {
        Timer timer;
        LandmarkGenerator<GraphType> landmarkGen(G, ids); 
        timer.start();
        landmarkGen.loadLandmarkDistances(basePath + "landmarks.dat");
        timer.stop();
        std::cout << "\tLandmark container capacity: "
                  << (G.getNumNodes() * sizeof(LandmarkData) * G.chooseNode()->landmark.capacity() / 1048576.0) << " Mbytes."
                  << "\n\tTime spent to read the landmarks: " << timer.getElapsedTime() << "sec\n";
    }

    //initilization : reset the status of each node
    for(typename GraphType::NodeIterator u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
        u->timestamp = 0;
        u->dist = std::numeric_limits<unsigned int>::max();
        u->distBack = std::numeric_limits<unsigned int>::max();
        u->pred = 0;
        u->succ = 0;
    }

    algTimestamp = 0;

    //check the feasibility of the heuristic - potential functions
    if(dijkstraVariant == 0 || (dijkstraVariant > 2 && dijkstraVariant < 7))
    {

        UniAstarEcl<GraphType> uniAstarEcl(G, &algTimestamp); 

        ProgressBar feasibleEcl(/*G.getNumNodes()*/1, std::string("Checking the feasibility of the euclidean potential functions"));

        typename GraphType::NodeIterator u = G.chooseNode(); 

        //full check
        //for(typename GraphType::NodeIterator u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
        {

            ++feasibleEcl;

            if(uniAstarEcl.hasFeasiblePotentials(u) == false)
            {
                std::cout << "Warning, feasibility isn't achieved! There is no guarantee for the validity of the results.\n";
                exit(1);
            }
        }

    }

    if(dijkstraVariant == 0 || dijkstraVariant > 6)
    {
        UniAstarLmk<GraphType> uniAstarLmk(G, &algTimestamp);

        ProgressBar feasibleLmk(/*G.getNumNodes()*/1, std::string("Checking the feasibility of the landmark-based potential functions"));

        typename GraphType::NodeIterator u = G.chooseNode(); 

        //full check
        //for(typename GraphType::NodeIterator u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
        {

            ++feasibleLmk;

            if(uniAstarLmk.hasFeasiblePotentials(u) == false)
            {
                std::cout << "Warning, feasibility isn't achieved! There is no guarantee for the validity of the results.\n";
                exit(1);
            }
        }
    }

    const std::string queryFileName = basePath + "queries.dat";
    std::string logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";

    switch( dijkstraVariant)
    {
        case 1:
            runQueries<Dijkstra<GraphType> >( G, ids, queryFileName, logFileName);
            break;
        case 2:
            runQueries<BidirectionalDijkstra<GraphType> >( G, ids, queryFileName, logFileName);
            break;
        case 3:
            runQueries<UniAstarEcl<GraphType> >( G, ids, queryFileName, logFileName);
            break;
        case 4:
            runQueries<BidAstarSymEcl<GraphType> >( G, ids, queryFileName, logFileName);
            break;
        case 5:
            runQueries<BidAstarMaxEcl<GraphType> >( G, ids, queryFileName, logFileName);
            break;
        case 6:
            runQueries<BidAstarAveEcl<GraphType> >( G, ids, queryFileName, logFileName);
            break;
        case 7:
            runQueries<UniAstarLmk<GraphType> >( G, ids, queryFileName, logFileName);
            break;
        case 8:
            runQueries<BidAstarSymLmk<GraphType> >( G, ids, queryFileName, logFileName);
            break;
        case 9:
            runQueries<BidAstarMaxLmk<GraphType> >( G, ids, queryFileName, logFileName);
            break;
        case 10:
            runQueries<BidAstarAveLmk<GraphType> >( G, ids, queryFileName, logFileName);
            break;

        default:

            dijkstraVariant = 1;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<Dijkstra<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 2;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<BidirectionalDijkstra<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 3;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<UniAstarEcl<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 4;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<BidAstarSymEcl<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 5;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<BidAstarMaxEcl<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 6;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<BidAstarAveEcl<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 7;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<UniAstarLmk<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 8;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<BidAstarSymLmk<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 9;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<BidAstarMaxLmk<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 10;
            logFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";
            runQueries<BidAstarAveLmk<GraphType> >( G, ids, queryFileName, logFileName);

            dijkstraVariant = 0;

            break;
    }
}

void checkCorrectness()
{
    const std::string queryFileName = basePath + "queries.dat";
    const std::string errorLogFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/errors.dat";
    const std::string reliableAlgLogFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[reliableAlg]) + ".dat";
    const std::string algLogFileName = "./stats/" + map + "/" + std::string(graphTypeLabel[graphVariant]) + "/" + std::string(algLabel[dijkstraVariant]) + ".dat";

    //read the queries from a file
    std::ifstream queryFile( queryFileName.c_str());
    std::ifstream algLogFile( algLogFileName.c_str());
    std::ifstream reliableAlgLogFile( reliableAlgLogFileName.c_str());
    std::ofstream errorLogFile( errorLogFileName.c_str(), std::ios::app);

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


typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   pmaGraph;
typedef DynamicGraph< ForwardStarImpl, node, edge>         fsGraph;
typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;

typedef GraphReader< Graph> AdjReader;
typedef GraphReader< pmaGraph> PmaReader;
typedef GraphReader< fsGraph> FsReader;

int main( int argc, char* argv[])
{

    dijkstraVariant = 0;
    graphVariant = 0;
    reliableAlg = 1;
    format = 0;

	AdjReader* reader = 0;
	PmaReader* pmaReader = 0;	
    FsReader* fsReader = 0;

    numActiveLandmarks = 0;
    
    //basePath = "/home/michai/Projects/Graphs/";
    //basePath = "/home/andreas/Projects/Graphs/";
    basePath = std::string(getenv("HOME")) + "/Projects/Graphs/";
    map = "luxembourg";

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("shortest path algorithm,a", po::value< unsigned int>(), "Choose: All[0], plain-dijkstra[1], plain-bidirectional[2], uni-A*-ecl[3], bid-A*-ecl-sym[4], bid-A*-ecl-max[5], bid-A*-ecl-ave[6], uni-A*-lmk[7], bid-A*-lmk-sym[8], bid-A*-lmk-max[9], bid-A*-lmk-ave[10], uni-A*-{lmk+ecl}[11], bid-A*-{lmk+ecl}-sym[12], bid-A*-{lmk+ecl}-max[13], bid-A*-{lmk+ecl}-ave[14]. Default:0")
        ("graphtype,g", po::value< unsigned int>(), "graph type. All[0], Adjacency List[1], Packed Memory Graph[2], forward Star[3]. Default:0")
		("format,f", po::value< unsigned int>(), "map format. DIMACS10[0], DIMACS9[1]. Default:0")
        ("map,m", po::value< std::string>(), "input map. The name of the map to read. Default:'luxembourg'. Maps should reside in '$HOME/Projects/Graphs/DIMACS{9,10}/' and should consist of 2 files, both with the same map name prefix, and suffixes 'osm.graph' and 'osm.xyz' in the case of DIMACS10 and, '.gr' and '.co' in the case of DIMACS9.")
        ("queries,q", "The input queries are in the file queries.dat. The file must be placed in '$HOME/Projects/Graphs/.../$MAP/'")
        ("check correctness,c", po::value< unsigned int>(), "Based on a reliable shortest path algorithm. The shortest path results of the queries of any selected shortest path algorithm from the above field -g will be compared with the ones of the reliable shortest path algorithm -c.")
        ("number of active landmarks,l", po::value< unsigned int>(), "The number of the active landmarks per shortest path query.")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if ( vm.empty()) 
    {
        std::cout << desc << "\n";
        return 0;
    }

    if ( vm.count("shortest path algorithm"))
    {
        dijkstraVariant = vm["shortest path algorithm"].as<unsigned int>();
    }

    if ( vm.count("number of active landmarks"))
    {
        numActiveLandmarks = vm["number of active landmarks"].as<unsigned int>();
    }

    if ( vm.count("graphtype"))
    {
        graphVariant = vm["graphtype"].as<unsigned int>();
    }
    
    if ( vm.count("map"))
    {
        map = vm["map"].as<std::string>();
    }

	if ( vm.count("format") && ( format = vm["format"].as<unsigned int>()) == 1)
    {
        basePath = basePath + "DIMACS9/" + map + "/";

        reader = new DIMACS9Reader<Graph>( basePath + map + ".gr",
                                           basePath + map + ".co");

        pmaReader = new DIMACS9Reader<pmaGraph>( basePath + map + ".gr",
                                                 basePath + map + ".co");

        fsReader = new DIMACS9Reader<fsGraph>( basePath + map + ".gr",
                                               basePath + map + ".co");
    }

    else
    {
        basePath = basePath + "DIMACS10/" + map + "/";

        reader = new DIMACS10Reader<Graph>( basePath + map + ".osm.graph",
                                            basePath + map + ".osm.xyz");

        pmaReader = new DIMACS10Reader<pmaGraph>( basePath + map + ".osm.graph",
                                                  basePath + map + ".osm.xyz");

        fsReader = new DIMACS10Reader<fsGraph>( basePath + map + ".osm.graph",
                                                  basePath + map + ".osm.xyz");
    }

    if (vm.count("check correctness"))
    {
        reliableAlg = vm["check correctness"].as<unsigned int>();

        if( dijkstraVariant == 0)
        {
            for( dijkstraVariant = 1; dijkstraVariant< 11; dijkstraVariant++)
                if( dijkstraVariant != reliableAlg)
                    checkCorrectness();
                else
                    continue;
        }
        else
            checkCorrectness();

        return 0;
    }

    Graph G;
    pmaGraph pmaG;
    fsGraph  fsG;
        
    if(dijkstraVariant == -1)
    {   
        G.read(reader);
        if(format == 0)
            calcWeights(G);
        getGraphStats(G);
        return 0;
    }

    Timer timer;

    switch( graphVariant)
    {
        case 1:
            timer.start();
            G.read(reader);
            timer.stop();
            std::cout << "Graph has " << (double)G.memUsage() / 1048576.0 << " Mbytes (without landmarks). Time spent to read:\t" 
                      << timer.getElapsedTime() << "sec" << std::endl;
            std::cout << "IDs container capacity: "
                      << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
            if(format == 0)
                calcWeights(G);
            runExperimentsAt( G, reader->getIds());
            reader->getIds().clear();
            G.clear();
            break;

        case 2:
            timer.start();
            pmaG.read(pmaReader);
            std::cout << "Graph has " << (double)pmaG.memUsage() / 1048576.0 << " Mbytes (without landmarks). Time spent to read:\t" 
                      << timer.getElapsedTime() << "sec" << std::endl;
            std::cout << "IDs container capacity: "
                      << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
            if(format == 0)
                calcWeights(pmaG);
            runExperimentsAt( pmaG, pmaReader->getIds());
            pmaReader->getIds().clear();
            pmaG.clear();
            break;

        case 3:
            timer.start();
            fsG.read(fsReader);
            std::cout << "Graph has " << (double)fsG.memUsage() / 1048576.0 << " Mbytes (without landmarks). Time spent to read:\t" 
                      << timer.getElapsedTime() << "sec" << std::endl;
            std::cout << "IDs container capacity: "
                      << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
            if(format == 0)
                calcWeights(fsG);
            runExperimentsAt( fsG, fsReader->getIds());
            fsReader->getIds().clear();
            fsG.clear();
            break;

        default:
            graphVariant = 1;
            timer.start();
            G.read(reader);
            timer.stop();
            std::cout << "Graph has " << (double)G.memUsage()/1048576.0 << " Mbytes (without landmarks). Time spent to read:\t" 
                      << timer.getElapsedTime() << "sec" << std::endl;
            std::cout << "IDs container capacity: "
                      << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
            if(format == 0)
                calcWeights(G);
            runExperimentsAt( G, reader->getIds());
            reader->getIds().clear();
            G.clear();

            graphVariant = 2;
            timer.start();
            pmaG.read(pmaReader);
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576.0 << " Mbytes (without landmarks). Time spent to read:\t" 
                      << timer.getElapsedTime() << "sec" << std::endl;
            std::cout << "IDs container capacity: "
                      << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
            if(format == 0)
                calcWeights(pmaG);
            runExperimentsAt( pmaG, pmaReader->getIds());
            pmaReader->getIds().clear();
            pmaG.clear();

            graphVariant = 3;
            timer.start();
            fsG.read(fsReader);
            std::cout << "Graph has " << (double)fsG.memUsage() / 1048576.0 << " Mbytes (without landmarks). Time spent to read:\t" 
                      << timer.getElapsedTime() << "sec" << std::endl;
            std::cout << "IDs container capacity: "
                      << (sizeof(NodeDescriptor) * reader->getIds().capacity() / 1048576.0) << " Mbytes." << std::endl;
            if(format == 0)
                calcWeights(fsG);
            runExperimentsAt( fsG, fsReader->getIds());
            fsReader->getIds().clear();
            fsG.clear();

            graphVariant = 0;
            break;
    }

    return 0;
}
