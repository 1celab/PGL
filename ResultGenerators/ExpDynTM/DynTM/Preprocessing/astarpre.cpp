#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include <Utilities/timer.h>

#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/forwardStarImpl.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>

#include "../Structs/tspNetwork.h"
#include "../Readers/GTFSReader.h"
#include "../Readers/GTFS2Reader.h"
#include "../Readers/INTReader.h"

#include "../Algs/astarHeur.h"

using namespace DynTM;

//graph node
struct NodeInfo : DefaultGraphItem, TspNode
{};

//graph edge
struct EdgeInfo : DefaultGraphItem, TspEdge
{};

//graph inEdge
struct InEdgeInfo : DefaultGraphItem
{};

template< typename GraphType>
void run( int& format, std::string& basePath)
{
    typedef TspNetwork<GraphType> TspNetwork;

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

    if( evEnabled)
    {
        TspPreprocessor<TspNetwork> tpr( net.stations);
        tpr.readData( basePath + "car.csv");
        tpr.setEVPaths( 60, 5); //2h radius 
    }

    if( walkEnabled)
    {
        TspPreprocessor<TspNetwork> tpr( net.stations);
        tpr.readData( basePath + "pedestrian.csv");
        tpr.setWalkPaths( 15, 5); //15 mins radius
    }


    std::cout << "Graph has " << (double) net.G.memUsage() / 1048576.0
              << " Mbytes. Time spent to read:\t"
              << timer.getElapsedTime() << "secs\n";

    timer.start();

    //compute and write A* heuristics
    TTL algTimestamp = 0;
    AstarHeur<TspNetwork> astarHeur( net.G, net.stations, &algTimestamp);
    astarHeur.buildTrees();
    astarHeur.writeHeuristics( basePath + "lowBnds.dat");

    std::cout.precision(2);
    std::cout << "Time spent on preprocessing: "
              << (timer.getElapsedTime() / 60.0) << "mins\n";

    net.clear();
}

typedef DynamicGraph< PackedMemoryArrayImpl, NodeInfo, EdgeInfo, InEdgeInfo>   pmaGraph;
typedef DynamicGraph< ForwardStarImpl, NodeInfo, EdgeInfo, InEdgeInfo>         fsGraph;
typedef DynamicGraph< AdjacencyListImpl, NodeInfo, EdgeInfo, InEdgeInfo>       Graph;

int main( int argc, char* argv[])
{
    //graph structures => ADJ:adjacency list, DynFS:Dynamic Forward Star, PMG:Packed-Memory Graph, sFS:Static Forward Star
    int graphVariant = 1;
    int format = 1;
    bool walkEnabled = false;
    bool evEnabled = false;

    //graph data file
    std::string homePath, basePath, netName;

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
        ("graph structure,g", po::value<int>(), "Graph structure. Adjacency List[1], Dynamic Forward Star[2], Packed Memory Graph[3], Static Forward Star[4]. Default:1.")
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

    if ( vm.count("net"))
        netName = vm["net"].as<std::string>();

    if ( vm.count("format"))
        format = vm["format"].as<int>();

    if ( vm.count("graph structure"))
        graphVariant = vm["graph structure"].as<int>();

    if( format == 1)
        basePath = basePath + "TspNet/" + netName + "/";
    else
        basePath = basePath + "GTFS/" + netName + "/";

    if ( vm.count("walk"))
        walkEnabled = vm["walk"].as<bool>();

    if ( vm.count("ev"))
        evEnabled = vm["ev"].as<bool>();

    Timer timer;

    switch( graphVariant)
    {
        //ADJ
        case 1:
            run<Graph>( format, basePath);
            break;

        //dynFS
        case 2:
            run<fsGraph>( format, basePath);
            break;

        //PMG
        case 3:
            run<pmaGraph>( format, basePath);
            break;

        //sFS
        case 4:
            run<pmaGraph>( format, basePath);
            break;
    }

    return 0;
}
