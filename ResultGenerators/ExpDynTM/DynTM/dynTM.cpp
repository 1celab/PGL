#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include <Utilities/timer.h>

#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/forwardStarImpl.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>

#include "Structs/tspNetwork.h"
#include "Readers/GTFSReader.h"
#include "Readers/GTFS2Reader.h"
#include "Readers/INTReader.h"
#include "Readers/tspPreprocessor.h"

#include "Algs/tspRegDijkstra.h"
#include "Algs/tspOptDijkstra.h"
#include "Algs/tspMultiDijkstra.h"
#include "Algs/tspALT.h"
#include "Algs/tspMultiALT.h"
#include "Algs/tspDelay.h"

using namespace DynTM;

//graph Node
struct NodeInfo : DefaultGraphItem, TspNode, DjkNode
{};

//graph Edge
struct EdgeInfo : DefaultGraphItem, TspEdge
{};

struct TransitInfo
{
    TransitInfo(): transit(0)
    {}

    bool transit;
};

//graph InEdge
struct InEdgeInfo : DefaultGraphItem
{};

TTL algTimestamp = 0;

/**
 * @brief Executes a shortest path query.
 * @param net The transport network.
 */
template<typename GraphType>
void runQuery( TspNetwork<GraphType>& net, int algorithm, std::string& basePath)
{
    typedef TspNetwork<GraphType> TspNetwork;
    Timer timer;

    StationID sourceStationId, targetStationId;
    Time startTime;

    while(1)
    {
        //stations ids
        std::cout << "\nStation Ids:[0," << net.stations.size()-1 << "] Time:[0,1440)\n";

        //set source station
        std::cout << "Departure station:_\b";
        std::cin >> sourceStationId;

        //set target station
        std::cout << "Arrival station:_\b";
        std::cin >> targetStationId;

        //set start time at source station
        std::cout << "Start time:_\b";
        std::cin >> startTime;

        //set the number of used vehicle types
        unsigned int numVehicles;
        std::cout << "Number of vehicle types (<=" << net.vehicles.size() << ", 0 or " << net.vehicles.size() << ": All)\n";
        std::cin >> numVehicles;
        if( numVehicles >= net.vehicles.size()) numVehicles = 0;

        std::cout << "Selected vehicle types:[0," << net.vehicles.size()-1 << "]\n";
        std::vector<VehicleTypeID> vehicleTypes;
        while( numVehicles > 0)
        {
            int vhType;
            std::cin >> vhType;

            if( vhType < 0)
                continue;

            vehicleTypes.push_back( vhType);
            --numVehicles;
        }

        //regular Dijkstra
        if( algorithm == 1)
        {
            TspRegDijkstra<TspNetwork> alg( net.G, net.stations, &algTimestamp);
            //run shortest path query
            timer.start();
            alg.runQuery( sourceStationId, targetStationId, startTime);
            timer.stop();

            //print the discovered shortest path
            alg.printShortestPath();
        }

        //optimized Dijkstra
        if( algorithm == 2)
        {
            TspOptDijkstra<TspNetwork> alg( net.G, net.stations, &algTimestamp);

            //run shortest path query
            timer.start();
            alg.runQuery( sourceStationId, targetStationId, startTime);
            timer.stop();

            //print the discovered shortest path
            alg.printShortestPath();
        }

        //ALT
        else if( algorithm == 3)
        {
            TspALT<TspNetwork> alg( net.G, net.stations, &algTimestamp, basePath + "lowBnds.dat");

            //run shortest path query
            timer.start();
            alg.runQuery( sourceStationId, targetStationId, startTime);
            timer.stop();

            //print the discovered shortest path
            alg.printShortestPath();
        }

        //optimized multi Dijkstra
        else if( algorithm == 4)
        {
            TspMultiDijkstra<TspNetwork> alg( net.G, net.stations, net.vehicles, &algTimestamp);

            //run shortest path query
            timer.start();
            alg.runQuery( sourceStationId, targetStationId, startTime, vehicleTypes);
            timer.stop();

            //print the discovered shortest path
            alg.printShortestPath();
        }

        //optimized multi ALT
        else if( algorithm == 5)
        {
            TspMultiALT<TspNetwork> alg( net.G, net.stations, net.vehicles, &algTimestamp, basePath + "lowBnds.dat");
            //run shortest path query
            timer.start();
            alg.runQuery( sourceStationId, targetStationId, startTime, vehicleTypes);
            timer.stop();

            //print the discovered shortest path
            alg.printShortestPath();
        }

        else
            std::cout << "unknown option on algorithm";
        
        std::cout << "\nQuery time: "<< (timer.getElapsedTime()*1000) << "ms\n\n";
    }
}


/**
 * @brief Executes a delay-update query.
 * @param net The transport network.
 */
template<typename TspNetwork>
void update( TspNetwork& net)
{
    Timer timer;

    StationID stationId;
    Time startTime, delay = 0;

    //stations ids
    std::cout << "\nStation Ids:[0," << net.stations.size()-1 << "] Time:[0,1440)\n";

    //set station
    std::cout << "Station:_\b";
    std::cin >> stationId;

    //set start time from source-station
    std::cout << "Start time:_\b";
    std::cin >> startTime;

    //set delay
    //(pact: the delay is applied to the ealiest departure in the station, taking as a reference the startTime)
    std::cout << "Delay:_\b";
    std::cin >> delay;

    unsigned int algTimestamp = 0;
    DelayPropagator<TspNetwork> delayPro( net.G, net.stations, &algTimestamp);

    typename TspNetwork::NodeIterator depNode = delayPro.getAffectedDepNode( stationId, startTime);

    timer.start();
    delayPro.update( stationId, depNode, delay);
    timer.stop();

    std::cout << "\nNum Updated Stations: " << delayPro.getNumUpdatedStations()
              << "\nNum Updated Edges: "<< delayPro.getNumUpdatedEdges()
              << "\nUpdate time: "<< ( timer.getElapsedTime() * 1000) << "ms\n\n";
}

struct QueryType
{
    char op;
    Time startTime;
    union{ StationID sourceId, stationId; };
    union{ StationID targetId; Time delay; };
    union{ Distance distance; unsigned int updates; };
    float exeTime;
    std::vector<VehicleTypeID> vehicleTypes;
};


/**
 * @brief Reads and executes a sequence of random queries from a file.
 * @param net The rail network.
 */
template<typename GraphType>
void runQueriesFromFile( TspNetwork<GraphType>& net, int algorithm, 
std::string queryFileName, std::string resultFileName, std::string& basePath, bool skipDelayUpdates = false)
{
    typedef  TspNetwork<GraphType> TspNetwork;

    //queries (read mode)
    std::ifstream in( queryFileName.c_str());
    std::cout << "\nReading queries from " << queryFileName << "\n" << std::flush;
    if( !in)
    {
        std::cerr << "error: unable to open file [" << queryFileName << "]\n" << std::flush;
        exit(-1);
    }

    //results (write mode)
    std::ofstream out( resultFileName.c_str());
    std::cout << "Writing results to " << resultFileName << "\n" << std::flush;
    if( !out)
    {
        std::cerr << "error: unable to open file [" << resultFileName << "]\n" << std::flush;
        exit(-1);
    }

    //remove the first line (pact : single-line comment)
    in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    unsigned int numQueries;
    unsigned int numSPQueries = 0;
    unsigned int numDLQueries = 0; 
    unsigned int numSkippedQueries = 0;

    in >> numQueries;
    in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    //remove the third line (pact : single-line comment)
    in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

    ProgressStream query_progress( numQueries);
    query_progress.label() << "\tRunning " << numQueries << " queries";

    //timer
    Timer timer;

    //statistics
    double avgTouchedEdges = 0, avgTouchedNodes = 0, avgSettledNodes = 0, avgUpdatedStations = 0, avgUpdatedEdges = 0;
    double exeTime, avgQueryTime = 0, avgUpdateTime = 0;

    unsigned int touchedEdges, touchedNodes, settledNodes, updatedStations, updatedEdges;
    Distance distance;

    //algorithms
    TspRegDijkstra<TspNetwork> rdj( net.G, net.stations, &algTimestamp);
    TspOptDijkstra<TspNetwork> odj( net.G, net.stations, &algTimestamp);
    TspALT<TspNetwork> alt( net.G, net.stations, &algTimestamp);
    TspMultiDijkstra<TspNetwork> mdj( net.G, net.stations, net.vehicles, &algTimestamp);
    TspMultiALT<TspNetwork> malt( net.G, net.stations, net.vehicles, &algTimestamp);
    DelayPropagator<TspNetwork> delayPro( net.G, net.stations, &algTimestamp);

    //heuristic data
    std::string lowBndFileName = basePath + "lowBnds.dat";
    if( algorithm == 3)
        alt.loadHeuristicData( lowBndFileName);
    else if( algorithm == 5)
        malt.loadHeuristicData( lowBndFileName);        

    //prepare queries
    std::vector<QueryType> queries;
    for( unsigned int i=0; i < numQueries; ++i)
    {
        QueryType query;
        
        //$(operation) $(source-station) $(target-station or delay) $(startTime) $(vehile types size) $(vehilce type id sequence)
        in >> query.op >> query.sourceId >> query.targetId >> query.startTime;

        if( query.op == 'q')
        {
            unsigned int numVehicles;
            in >> numVehicles;
            if( numVehicles == 0)
                query.vehicleTypes.clear();
            else
            {
                std::vector<VehicleTypeID>& vehicleTypes = query.vehicleTypes;
                vehicleTypes.resize( numVehicles);
                for( unsigned int j=0; j<numVehicles; ++j)
                    in >> vehicleTypes[j];
            }
        }

        queries.push_back( query);
    }

    out << "Stations: " << net.stations.size() 
        << "  Nodes: " << net.G.getNumNodes() << "  Edges: " << net.G.getNumEdges()
        << "\nQueries[q]  Source-St  Target-St  StartTime  TouchedEdges  TouchedNodes  SettledNodes  ExeTime(ms)  Distance"
        << "\nUpdates[d]  Station  Delay  StartTime  ExeTime(μs)  UpdatedEdges  UpdatedStations \n" << std::flush;

    std::cout.setf( std::ios::fixed, std::ios::floatfield);
    out.setf( std::ios::fixed, std::ios::floatfield);
    std::cout.precision(4);
    out.precision(4);

    //execute queries
    for( unsigned int i=0; i < numQueries; ++i)
    {
        //shortest path query
        if( queries[i].op == 'q')
        {
            //regular Dijkstra
            if( algorithm == 1)
            {
                timer.start();
                rdj.runQuery( queries[i].sourceId, queries[i].targetId, queries[i].startTime);
                timer.stop();

                distance = rdj.getDistance();
                exeTime = timer.getElapsedTime();
                settledNodes = rdj.getSettledNodes();
                touchedNodes = rdj.getTouchedNodes();
                touchedEdges = rdj.getTouchedEdges();
            }

            //optimized Dijkstra
            if( algorithm == 2)
            {
                timer.start();
                odj.runQuery( queries[i].sourceId, queries[i].targetId, queries[i].startTime);
                timer.stop();

                distance = odj.getDistance();
                exeTime = timer.getElapsedTime();
                settledNodes = odj.getSettledNodes();
                touchedNodes = odj.getTouchedNodes();
                touchedEdges = odj.getTouchedEdges();
            }

            //ALT
            else if( algorithm == 3)
            {
                timer.start();
                alt.runQuery( queries[i].sourceId, queries[i].targetId, queries[i].startTime);
                timer.stop();

                distance = alt.getDistance();
                exeTime = timer.getElapsedTime();
                settledNodes = alt.getSettledNodes();
                touchedNodes = alt.getTouchedNodes();
                touchedEdges = alt.getTouchedEdges();
            }

            //optimized multi Dijkstra  
            else if( algorithm == 4)
            {
                timer.start();
                mdj.runQuery( queries[i].sourceId, queries[i].targetId, queries[i].startTime, queries[i].vehicleTypes);
                timer.stop();

                distance = mdj.getDistance();
                exeTime = timer.getElapsedTime();
                settledNodes = mdj.getSettledNodes();
                touchedNodes = mdj.getTouchedNodes();
                touchedEdges = mdj.getTouchedEdges();
            }

            //multi ALT
            else if( algorithm == 5)
            {
                timer.start();
                malt.runQuery( queries[i].sourceId, queries[i].targetId, queries[i].startTime, queries[i].vehicleTypes);
                timer.stop();

                distance = malt.getDistance();
                exeTime = timer.getElapsedTime();
                settledNodes = malt.getSettledNodes();
                touchedNodes = malt.getTouchedNodes();
                touchedEdges = malt.getTouchedEdges();
            }

            //sh path query time - convert to ms
            exeTime = 1000.0 * exeTime;

            //skip the query, only if target-station is not reachable from source-station
            if( distance == std::numeric_limits<Distance>::max())
                numSkippedQueries++;

            else
            {
                numSPQueries++;
                avgQueryTime    += exeTime;
                avgSettledNodes += settledNodes;
                avgTouchedNodes += touchedNodes;
                avgTouchedEdges += touchedEdges;
            }

            out << "q " << queries[i].sourceId << " " << queries[i].targetId << " " << queries[i].startTime
                << " " << touchedEdges << " " << touchedNodes << " " << settledNodes 
                << " " << exeTime << " " << distance << "\n" << std::flush;
        }

        //update-delay query
        else if( skipDelayUpdates == false)
        {
            updatedStations = 0;
            updatedEdges = 0;

            typename TspNetwork::NodeIterator depNode = delayPro.getAffectedDepNode( queries[i].stationId, queries[i].startTime);

            timer.start();
            delayPro.update( queries[i].stationId, depNode, queries[i].delay);
            timer.stop();

            numDLQueries++;
            updatedStations = delayPro.getNumUpdatedStations();
            updatedEdges = delayPro.getNumUpdatedEdges();

            //delay update time, convert to μs
            exeTime = 1000000.0 * timer.getElapsedTime();

            avgUpdateTime += exeTime;
            avgUpdatedStations += delayPro.getNumUpdatedStations();
            avgUpdatedEdges += delayPro.getNumUpdatedEdges();

            out << "d " << queries[i].stationId << " " << queries[i].delay << " " << queries[i].startTime 
                << " " << exeTime << " " << updatedEdges << " " << updatedStations << "\n" << std::flush;
        }

        ++query_progress;
    }

    //check network topology validity
    /*for( StationID j=0, size=net.stations.size(); j<size; j++)
        if( net.isValid( j) == false)
            std::cout << "\nproblem with station:" << j;*/

    //statistics
    avgQueryTime    /= numSPQueries;
    avgSettledNodes /= numSPQueries;
    avgTouchedNodes /= numSPQueries;
    avgTouchedEdges /= numSPQueries;

    avgUpdateTime      /= numDLQueries;
    avgUpdatedEdges    /= numDLQueries;
    avgUpdatedStations /= numDLQueries;

    
    //sh Path queries (ms)
    if( avgQueryTime < 0.01)
    {
        out.precision(3);
        std::cout.precision(3);
    }
    else
    {
        out.precision(2);
        std::cout.precision(2);
    }

    //shortest path results
    out << "Avg Query Time: " << avgQueryTime
        << "ms  Avg touched Edges: " << avgTouchedEdges
        << "  Avg touched Nodes: " << avgTouchedNodes 
        << "  Avg Settled Nodes: " << avgSettledNodes
        << "  Queries:" << numSPQueries;

    std::cout << "Avg Query Time: " << avgQueryTime
        << "ms  Avg touched Edges: " << avgTouchedEdges
        << "  Avg touched Nodes: " << avgTouchedNodes 
        << "  Avg Settled Nodes: " << avgSettledNodes
        << "  Queries:" << numSPQueries;


    //delay queries (μs)
    if( avgUpdateTime < 0.01)
    {
        out.precision(3);
        std::cout.precision(3);
    }
    else
    {
        out.precision(2);
        std::cout.precision(2);
    }

    //update results
    out << "\nAvg Update Time: "<< avgUpdateTime << "μs "
        << "  Avg updated stations: "<< avgUpdatedStations << "  Avg updated edges: " 
        << avgUpdatedEdges << "  Queries:" << numDLQueries << "\n" << std::flush;

    std::cout << "\nAvg Update Time: "<< avgUpdateTime << "μs "
        << "  Avg updated stations: "<< avgUpdatedStations << "  Avg updated edges: " 
        << avgUpdatedEdges << "  Queries:" << numDLQueries << "\n" << std::flush;
}

/**
 * @brief Interface.
 */
template<typename GraphType>
void shell( int& format, int& mod, int& algorithm, std::string& basePath, 
            std::string& qFile, std::string& rFile, std::string& netName, bool walkEnabled, bool evEnabled)
{
    typedef TspNetwork<GraphType> TspNetwork;
    typedef typename GraphType::NodeIterator NodeIterator;

    Timer timer;
    timer.start();

    TspNetwork net;

    if( format == 1)
    {
        INTReader<TspNetwork> data( net, basePath + "stations.int", basePath + "connections.int");
        data.read();
    }

    else if( format == 2)
    {
        GTFSReader<TspNetwork> data( net, basePath + "stops.csv", basePath + "connections.csv", basePath + "vehicles.csv");
        data.read();
    }

    else
    {
        GTFS2Reader<TspNetwork> data( net, basePath + "routes.txt", basePath + "trips.txt", 
                                           basePath + "stops.txt", basePath + "stop_times.txt", basePath + "transfers.txt");
        data.read();
    }
    
    net.printVehicleStats();
    
    if( evEnabled)
    {
        TspPreprocessor<TspNetwork> tpr( net.stations);
        tpr.readData( basePath + "car.csv");
        tpr.setEVPaths( 3600); //travel time: at most 1h, speed: 50km/h, EV-stations: 10% of total stations
    }

    if( walkEnabled)
    {
        TspPreprocessor<TspNetwork> tpr( net.stations);
        tpr.readData( basePath + "pedestrian.csv");
        tpr.setWalkPaths( 600); //travel time: at most 10 min, speed: 1m/s
    }

    std::cout << "Graph has " << (double) net.G.memUsage() / 1048576.0
              << " Mbytes. Time spent to read:\t"
              << timer.getElapsedTime() << "secs\n";

    //reset timestamps
    algTimestamp = 0;
    for( NodeIterator v = net.G.beginNodes(), lastnode=net.G.endNodes(); v!=lastnode; v++)
        v->timestamp = 0;

    if( mod == 1)
        runQuery( net, algorithm, basePath);
    else if( mod == 2)
        update( net);
    else
    {
        if( algorithm == 0)
        {
            /*algorithm = 1;
            rFile = "./stats/" + netName + ".DTM.RDJ.results";
            runQueriesFromFile( net, algorithm, qFile, rFile, basePath, true);*/

            algorithm = 2;
            rFile = "./stats/" + netName + ".DTM.ODJ.results";
            runQueriesFromFile( net, algorithm, qFile, rFile, basePath, true);
    
            algorithm = 3;
            rFile = "./stats/" + netName + ".DTM.ALT.results";
            runQueriesFromFile( net, algorithm, qFile, rFile, basePath, true);

            algorithm = 4;
            rFile = "./stats/" + netName + ".DTM.MDJ.results";
            runQueriesFromFile( net, algorithm, qFile, rFile, basePath, true);

            algorithm = 5;
            rFile = "./stats/" + netName + ".DTM.MALT.results";
            runQueriesFromFile( net, algorithm, qFile, rFile, basePath, false);
        }

        else
            runQueriesFromFile( net, algorithm, qFile, rFile, basePath);
    }

    net.clear();
}

typedef DynamicGraph< PackedMemoryArrayImpl, NodeInfo, EdgeInfo, InEdgeInfo>   pmaGraph;
typedef DynamicGraph< ForwardStarImpl, NodeInfo, EdgeInfo, InEdgeInfo>         fsGraph;
typedef DynamicGraph< AdjacencyListImpl, NodeInfo, EdgeInfo, InEdgeInfo>       Graph;

int main( int argc, char* argv[])
{
    //graph structures => ADJ:adjacency list, DynFS:Dynamic Forward Star, PMG:Packed-Memory Graph, sFS:Static Forward Star
    int graphVariant = 3;
    int algorithm = 2;
    int format = 1;
    int mod = 1;
    bool walkEnabled = false;
    bool evEnabled = false;

    //graph data file
    std::string homePath, basePath, netName;
    
    //query result file
    std::string qFile, rFile;

    //TODO warning - getenv has compatibility issues
    //basePath = "/home/paraskevop/Projects/Graphs/";
    homePath = std::string( getenv("HOME"));
    basePath = homePath + "/Projects/Data/Graphs/";

    netName = "a0i";

    //declare the supported options
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("net,n", po::value<std::string>(), "Name of transport network. Default:'a0i'.")
        ("format,f", po::value<int>(), "Format. INT[1], GTFS[2], GTFS2[3]. Default:1.")
        ("graph structure,g", po::value<int>(), "Graph structure. Adjacency List[1], Dynamic Forward Star[2], Packed Memory Graph[3], Static Forward Star[4]. Default:3.")
        ("algorithm,a", po::value<int>(), "Algorithm. Reg Dijkstra[1], Opt Dijkstra[2], A*[3], Multi Dijkstra[4], Multi A*[5], All[0]. Default:2.")
        ("mod,o", po::value<int>(), "Run shpath queries manually[1], delay-update manually[2], random queries from file[3]. Default:1")
        ("qFile,q", po::value<std::string>(), "Query file path. Default:./netName.queries")
        ("rFile,r", po::value<std::string>(), "Result file path. Default:./netName.RTE.results")
        ("walk,w", po::value<bool>(), "Enable walking. Default:0")
        ("ev,e", po::value<bool>(), "Enable electric car. Default:0")
        ;

    po::variables_map vm;
    po::store( po::parse_command_line(argc, argv, desc), vm);
    po::notify( vm);    

    if ( vm.empty()) 
    {
        std::cout << desc << "\n";
        return 0;
    }

    if ( vm.count("graph structure"))
        graphVariant = vm["graph structure"].as<int>();

    if ( vm.count("algorithm"))
        algorithm = vm["algorithm"].as<int>();

    if ( vm.count("net"))
        netName = vm["net"].as<std::string>();

    if ( vm.count("format"))
        format = vm["format"].as<int>();

    if( format == 1)
        basePath = basePath + "TspNet/" + netName + "/";
    else
        basePath = basePath + "GTFS/" + netName + "/";

    std::cout << "basePath:" << basePath << std::endl;

    qFile = "./" + netName + ".queries";
    rFile = "./" + netName + ".DTM.results";

    //query VS experiment
    if ( vm.count("mod"))
        mod = vm["mod"].as<int>();

    //query file
    if ( vm.count("qFile"))
        qFile = vm["qFile"].as<std::string>();

    //result file
    if ( vm.count("rFile"))
        rFile = vm["rFile"].as<std::string>();

    //enable walking
    if ( vm.count("walk"))
        walkEnabled = vm["walk"].as<bool>();

    //enable ev
    if ( vm.count("ev"))
        evEnabled = vm["ev"].as<bool>();

    switch( graphVariant)
    {
        //ADJ
        case 1:
            shell<Graph>( format, mod, algorithm, basePath, qFile, rFile, netName, walkEnabled, evEnabled);
            break;

        //dynFS
        case 2:
            shell<fsGraph>( format, mod, algorithm, basePath, qFile, rFile, netName, walkEnabled, evEnabled);
            break;

        //PMG
        case 3:
            shell<pmaGraph>( format, mod, algorithm, basePath, qFile, rFile, netName, walkEnabled, evEnabled);
            break;

        //sFS
        case 4:
            shell<pmaGraph>( format, mod, algorithm, basePath, qFile, rFile, netName, walkEnabled, evEnabled);
            break;
    }

    return 0;
}
