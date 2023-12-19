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
#include "Algs/scc.h"

using namespace DynTM;

//graph Node
struct NodeInfo : DefaultGraphItem, TspNode, SccNode
{};

//graph Edge
struct EdgeInfo : DefaultGraphItem, TspEdge
{};

//graph InEdge
struct InEdgeInfo : DefaultGraphItem
{};


/**
 * @brief Generates a sequence of random shortest path queries.
 * @param net The transport network.
 */
template<typename TspNetwork>
void generateRndSPQueries( TspNetwork& net, unsigned int numQueries, std::string queryFileName, int multi, std::vector<int>& vehTypes)
{
    boost::mt19937 gen(time(NULL));
    boost::uniform_real<> dist(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random(gen, dist);    

    std::ofstream out( queryFileName.c_str());
    std::cout << "\nWriting queries to " << queryFileName << "\n" << std::flush;
    if (!out)
    {
        std::cerr << "error: unable to open file [" << queryFileName << "]\n" << std::flush;
        exit(-1);
    }

    out << "$queries[SP]\n";
    out << numQueries << "\n";
    out << "$source-station $target-station $startTime $vehicleTypes\n";

    typename TspNetwork::GraphImpl::NodeIterator v;
    StationID sourceId, targetId;
    Time startTime;

    ProgressStream query_progress( numQueries);
    query_progress.label() << "Generating " << numQueries << " queries";

    std::vector<bool> selectedVehicles(net.vehicles.size(), false);
    const unsigned int totalVehicles = net.vehicles.size() * multi;

    for( unsigned int i=0, vsize=vehTypes.size(); i<vsize; i++)
         selectedVehicles[vehTypes[i]]= true;

    if( vehTypes.size() > 0)
    {
        for( unsigned int i = 0; i < numQueries; i++)
        {
            do
            {
                //select a random source station (with at least one departure)
                bool stop = false;
                do{ sourceId = random() * net.stations.size();

                    typename TspNetwork::Station& station = net.stations[sourceId];

                    std::vector< std::vector<VehicleTypeID> > outVehicles;
                    station.setOutgoingVehicles( net.G, outVehicles);
                    for( unsigned int i=0; i<outVehicles.size(); i++)
                        for( unsigned int j=0; j<outVehicles[i].size(); j++)
                            if( selectedVehicles[outVehicles[i][j]] == true)
                            {
                                stop = true;
                                i = outVehicles.size();
                                break;
                            }
                }
                while( stop == false);


                stop = false;
                do{ targetId = random() * net.stations.size();

                    typename TspNetwork::Station& station = net.stations[targetId];

                    std::vector< std::vector<VehicleTypeID> > inVehicles;
                    station.setIncomingVehicles( net.G, inVehicles);
                    for( unsigned int i=0; i<inVehicles.size(); i++)
                        for( unsigned int j=0; j<inVehicles[i].size(); j++)
                            if( selectedVehicles[inVehicles[i][j]] == true)
                            {
                                stop = true;
                                i = inVehicles.size();
                                break;
                            }
                }
                while( sourceId == targetId || stop == false);

            } while( net.G.getNodeIterator( net.stations[sourceId].stNode)->compId != net.G.getNodeIterator( net.stations[targetId].stNode)->compId);

            //select a random start time [0, 1339] at the source station
            startTime = random() * 1440;

            out << "q " << sourceId << " " << targetId << " " << startTime << " " << vehTypes.size() << " ";

            for( unsigned int i=0; i<vehTypes.size(); i++)
                    out << vehTypes[i] << " ";

            out << "\n";
            ++query_progress;
        }

        return;
    }


    for( unsigned int i = 0; i < numQueries; i++)
    {
        do
        {
            //select a random source station (with at least one departure)
            do{ sourceId = random() * net.stations.size();
                v = net.G.getNodeIterator( net.stations[sourceId].stNode); }
            while( net.G.outdeg( v) == 0);

            //select a random target station (!= source station)
            do{ targetId = random() * net.stations.size();}
            while( sourceId == targetId);

        } while( net.G.getNodeIterator( net.stations[sourceId].stNode)->compId != net.G.getNodeIterator( net.stations[targetId].stNode)->compId);

        //select a random start time [0, 1339] at the source station
        startTime = random() * 1440;

        out << "q " << sourceId << " " << targetId << " " << startTime << " ";

        //select a random number of used vehicle types [1, all]
        unsigned int numVehicles = 1 + random() * totalVehicles;

        if( numVehicles >= totalVehicles)
            out << "0";
        else
        {
            std::vector<bool> isSelected( net.vehicles.size(), false);
            out << numVehicles << " ";
            //select random vehicle types
            while( numVehicles > 0)
            {
                VehicleTypeID vhTypeId = (VehicleTypeID) ( random() * net.vehicles.size());
                if( isSelected[vhTypeId] == true) continue;
                isSelected[vhTypeId] = true;
                out << vhTypeId << " ";
                --numVehicles;
            }
        }

        out << "\n";
        ++query_progress;
    }
}


/**
 * @brief Generates a sequence of random delay-update queries.
 * @param net The transport network.
 */
template<typename TspNetwork>
void generateRndDLQueries( TspNetwork& net, unsigned int numQueries, std::string queryFileName)
{
    SccProcessor<typename TspNetwork::GraphImpl> scc( net.G);
    scc.computeComponents();

    boost::mt19937 gen(time(NULL));
    boost::uniform_real<> dist(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random(gen, dist);    

    std::ofstream out( queryFileName.c_str());
    std::cout << "\nWriting queries to " << queryFileName << "\n" << std::flush;
    if (!out)
    {
        std::cerr << "error: unable to open file [" << queryFileName << "]\n" << std::flush;
        exit(-1);
    }

    out << "$queries[DL]\n";
    out << numQueries << "\n";
    out << "$station $delay $startTime\n";

    StationID stationId;
    Time startTime, delay;

    ProgressStream query_progress( numQueries);
    query_progress.label() << "Generating " << numQueries << " delay-queries";

    for( unsigned int i = 0; i < numQueries; i++)
    {
        //select a random station (with at least one departure)
        do{ stationId = random() * net.stations.size();}
        while( scc.getLargestCompId() != net.G.getNodeIterator( net.stations[stationId].stNode)->compId);

        //select a random start time [0, 1339] at the station
        startTime = random() * 1440;

        //select a random delay [1, 360] 
        //(pact: the delay is applied to the ealiest departure in the station, taking as a reference the startTime)
        delay = 1 + random() * 360;

        out << "d " << stationId << " " << delay << " " << startTime << "\n";
        ++query_progress;
    }
}


/**
 * @brief Generates a sequence of random shortest path and delay-update queries.
 * @param net The transport network.
 */
template<typename TspNetwork>
void generateRndMXQueries( TspNetwork& net, unsigned int numQueries, std::string queryFileName, int multi, std::vector<int>& vehTypes)
{
    SccProcessor<typename TspNetwork::GraphImpl> scc( net.G);
    scc.computeComponents();

    boost::mt19937 gen(time(NULL));
    boost::uniform_real<> dist(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random(gen, dist);    

    std::ofstream out( queryFileName.c_str());
    std::cout << "\nWriting queries to " << queryFileName << "\n" << std::flush;
    if (!out)
    {
        std::cerr << "error: unable to open file [" << queryFileName << "]\n" << std::flush;
        exit(-1);
    }

    out << "$queries[MX]\n";
    out << numQueries << "\n";
    out << "$operation $source-station $target-station\\departure $delay\\deptime $vehicleTypes\n";

    typename TspNetwork::GraphImpl::NodeIterator v;
    StationID stationId, sourceId, targetId;
    Time startTime, delay;

    //update query per sp query
    const float ratio = 1.0 / 100.0;
    unsigned int numUpdates = ratio * numQueries;

    ProgressStream query_progress( numQueries);
    query_progress.label() << "Generating " << numQueries << " delay-queries";

    const unsigned int totalVehicles = net.vehicles.size() * multi;

    for( unsigned int i = 1; i <= numQueries; i++)
    {
        if( i % numUpdates == 0)
        {
            //select a random station (with at least one departure)
            do{ stationId = random() * net.stations.size();}
            while( scc.getLargestCompId() != net.G.getNodeIterator( net.stations[stationId].stNode)->compId);

            //select a random start time [0, 1339] at the source station
            startTime = random() * 1440;

            //select a random delay [1, 360] 
            //(pact: the delay is applied to the ealiest departure in the station, taking as a reference the startTime)
            delay = 1 + random() * 360;

            out << "d " << stationId << " " << delay << " " << startTime << "\n";
        }

        else
        {
            do
            {
                //select a random source station (with at least one departure)
                do{ sourceId = random() * net.stations.size();
                    v = net.G.getNodeIterator( net.stations[sourceId].stNode); }
                while( net.G.outdeg( v) == 0);

                //select a random target station (!= source station)
                do{ targetId = random() * net.stations.size();}
                while( sourceId == targetId);

            } while( net.G.getNodeIterator( net.stations[sourceId].stNode)->compId != net.G.getNodeIterator( net.stations[targetId].stNode)->compId);

            //select a random start time [0, 1339] at the source station
            startTime = random() * 1440;

            out << "q " << sourceId << " " << targetId << " " << startTime << " ";

            //select a random number of used vehicle types [1, all]
            unsigned int numVehicles = 1 + random() * totalVehicles;

            if( numVehicles >= totalVehicles)
                out << "0";
            else
            {
                std::vector<bool> isSelected( net.vehicles.size(), false);
                out << numVehicles << " ";
                //select random vehicle types
                while( numVehicles > 0)
                {
                    VehicleTypeID vhTypeId = (VehicleTypeID) ( random() * net.vehicles.size());
                    if( isSelected[vhTypeId] == true) continue;
                    isSelected[vhTypeId] = true;
                    out << vhTypeId << " ";
                    --numVehicles;
                }
            }

            out << "\n";
        }

        ++query_progress;
    }
}


/**
 * @brief Generates a sequence of random shortest path and delay-update queries.
 * @param net The transport network.
 */
template<typename TspNetwork>
void generateRndDisQueries( TspNetwork& net, unsigned int numQueries, std::string queryFileName, int multi, std::vector<int>& vehTypes)
{
    SccProcessor<typename TspNetwork::GraphImpl> scc( net.G);
    scc.computeComponents();

    boost::mt19937 gen(time(NULL));
    boost::uniform_real<> dist(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random(gen, dist);    

    std::ofstream out( queryFileName.c_str());
    std::cout << "\nWriting queries to " << queryFileName << "\n" << std::flush;
    if (!out)
    {
        std::cerr << "error: unable to open file [" << queryFileName << "]\n" << std::flush;
        exit(-1);
    }

    out << "$queries[DIS]\n";
    out <<  (2*numQueries) << "\n";
    out << "$operation $source-station $target-station\\departure $delay\\deptime $vehicleTypes\n";

    typename TspNetwork::GraphImpl::NodeIterator v;
    StationID stationId, sourceId, targetId;
    Time startTime, delay;

    ProgressStream query_progress( 2 * numQueries);
    query_progress.label() << "Generating " << numQueries << " sPath and " << numQueries << " delay queries";

    std::vector<bool> selectedVehicles( net.vehicles.size(), false);
    const unsigned int totalVehicles = net.vehicles.size() * multi;


    std::cout << "\nvehTypes.size():" << vehTypes.size();

    for( unsigned int i=0, vsize=vehTypes.size(); i<vsize; i++)
         selectedVehicles[vehTypes[i]]= true;

    //via selected vehicle types
    if( vehTypes.size() > 0)
    {
        //queries
        for( unsigned int i = 0; i < numQueries; i++)
        {
            do
            {
                //select a random source station (with at least one departure)
                bool stop = false;
                do{ sourceId = random() * net.stations.size();

                    typename TspNetwork::Station& station = net.stations[sourceId];

                    std::vector< std::vector<VehicleTypeID> > outVehicles;
                    station.setOutgoingVehicles( net.G, outVehicles);
                    for( unsigned int i=0; i<outVehicles.size(); i++)
                        for( unsigned int j=0; j<outVehicles[i].size(); j++)
                            if( selectedVehicles[outVehicles[i][j]] == true)
                            {
                                stop = true;
                                i = outVehicles.size();
                                break;
                            }
                }
                while( stop == false);


                stop = false;
                do{ targetId = random() * net.stations.size();

                    typename TspNetwork::Station& station = net.stations[targetId];

                    std::vector< std::vector<VehicleTypeID> > inVehicles;
                    station.setIncomingVehicles( net.G, inVehicles);
                    for( unsigned int i=0; i<inVehicles.size(); i++)
                        for( unsigned int j=0; j<inVehicles[i].size(); j++)
                            if( selectedVehicles[inVehicles[i][j]] == true)
                            {
                                stop = true;
                                i = inVehicles.size();
                                break;
                            }
                }
                while( sourceId == targetId || stop == false);

            } while( net.G.getNodeIterator( net.stations[sourceId].stNode)->compId != net.G.getNodeIterator( net.stations[targetId].stNode)->compId);

            //select a random start time [0, 1339] at the source station
            startTime = random() * 1440;

            out << "q " << sourceId << " " << targetId << " " << startTime << " " << vehTypes.size() << " ";

            for( unsigned int i=0; i<vehTypes.size(); i++)
                    out << vehTypes[i] << " ";

            out << "\n";
            ++query_progress;
        }


        //updates
        for( unsigned int i = 0; i < numQueries; i++)
        {
            //select a random station (with at least one departure)
            //select a random source station (with at least one departure)
            bool stop = false;
            do{ stationId = random() * net.stations.size();

                typename TspNetwork::Station& station = net.stations[stationId];

                std::vector< std::vector<VehicleTypeID> > outVehicles;
                station.setOutgoingVehicles( net.G, outVehicles);
                for( unsigned int i=0; i<outVehicles.size(); i++)
                    for( unsigned int j=0; j<outVehicles[i].size(); j++)
                        if( selectedVehicles[outVehicles[i][j]] == true)
                        {
                            stop = true;
                            i = outVehicles.size();
                            break;
                        }
            }
            while( stop == false);

            //select a random start time [0, 1339] at the station
            startTime = random() * 1440;

            //select a random delay [1, 360] 
            //(pact: the delay is applied to the ealiest departure in the station, taking as a reference the startTime)
            delay = 1 + random() * 360;

            out << "d " << stationId << " " << delay << " " << startTime << "\n";
            ++query_progress;
        }

        return;
    }

    //queries
    for( unsigned int i = 0; i < numQueries; i++)
    {
        do
        {
            //select a random source station (with at least one departure)
            do{ sourceId = random() * net.stations.size();
                v = net.G.getNodeIterator( net.stations[sourceId].stNode); }
            while( net.G.outdeg( v) == 0);

            //select a random target station (!= source station)
            do{ targetId = random() * net.stations.size();}
            while( sourceId == targetId);

        } while( net.G.getNodeIterator( net.stations[sourceId].stNode)->compId != net.G.getNodeIterator( net.stations[targetId].stNode)->compId);

        //select a random start time [0, 1339] at the source station
        startTime = random() * 1440;

        out << "q " << sourceId << " " << targetId << " " << startTime << " ";

        //select a random number of used vehicle types [1, all]
        unsigned int numVehicles = 1 + random() * totalVehicles;

        if( numVehicles >= totalVehicles)
            out << "0";
        else
        {
            std::vector<bool> isSelected( net.vehicles.size(), false);
            out << numVehicles << " ";
            //select random vehicle types
            while( numVehicles > 0)
            {
                VehicleTypeID vhTypeId = (VehicleTypeID) ( random() * net.vehicles.size());
                if( isSelected[vhTypeId] == true) continue;
                isSelected[vhTypeId] = true;
                out << vhTypeId << " ";
                --numVehicles;
            }
        }

        out << "\n";
        ++query_progress;
    }

    //updates
    for( unsigned int i = 0; i < numQueries; i++)
    {
        //select a random station (with at least one departure)
        do{ stationId = random() * net.stations.size();}
        while( scc.getLargestCompId() != net.G.getNodeIterator( net.stations[stationId].stNode)->compId);

        //select a random start time [0, 1339] at the station
        startTime = random() * 1440;

        //select a random delay [1, 360] 
        //(pact: the delay is applied to the ealiest departure in the station, taking as a reference the startTime)
        delay = 1 + random() * 360;

        out << "d " << stationId << " " << delay << " " << startTime << "\n";
        ++query_progress;
    }

}

/**
 * @brief Interface.
 */
template<typename GraphType>
void shell( int& format, int& mod, unsigned int& numQueries, std::string& basePath, std::string& qFile, int multi, std::vector<int>& vehTypes)
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

    std::cout << "Graph has " << (double) net.G.memUsage() / 1048576.0
              << " Mbytes. Time spent to read:\t"
              << timer.getElapsedTime() << "secs\n";

    if( mod == 1)
        generateRndSPQueries( net, numQueries, qFile, multi, vehTypes);
    else if( mod == 2)
        generateRndDLQueries( net, numQueries, qFile);
    else if( mod == 3)
        generateRndMXQueries( net, numQueries, qFile, multi, vehTypes);
    else
        generateRndDisQueries( net, numQueries, qFile, multi, vehTypes);

    net.clear();
}

typedef DynamicGraph< PackedMemoryArrayImpl, NodeInfo, EdgeInfo, InEdgeInfo>   pmaGraph;
typedef DynamicGraph< ForwardStarImpl, NodeInfo, EdgeInfo, InEdgeInfo>        fsGraph;
typedef DynamicGraph< AdjacencyListImpl, NodeInfo, EdgeInfo, InEdgeInfo>      Graph;

int main( int argc, char* argv[])
{
    //graph structures => ADJ:adjacency list, DynFS:Dynamic Forward Star, PMG:Packed-Memory Graph, sFS:Static Forward Star
    unsigned int numQueries = 1000;
    int graphVariant = 1;
    int format = 1;
    int mod = 1;
    int multi = 0;
    std::vector<int> vehTypes;

    //graph data file
    std::string homePath, basePath, netName;
    
    //query result file
    std::string qFile;

    //TODO warning - getenv has compatibility issues
    //basePath = "/home/paraskevop/Projects/Graphs/";
    homePath = std::string( getenv("HOME"));

    if( homePath == "/home/paraskevop" || homePath == "/home/titan")
        basePath = homePath + "/Projects/Data/Graphs/";
    else
        basePath = homePath + "/Scrivania/pgl/";

    netName = "a0i";

    //declare the supported options
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("net,n", po::value<std::string>(), "Name of transport network. Default:'a0i'.")
        ("format,f", po::value<int>(), "Format. INT[1], GTFS[2], GTFS2[3]. Default:1.")
        ("queries,q", po::value<unsigned int>(), "Number of queries. Default:1000.")
        ("multi,a", po::value<int>(), "Vehicles All[0]/Multi[1]. Default:0.")
        ("types,t", po::value<std::vector<int> >()->multitoken(), "Vehicles types.")
        ("mod,o", po::value<int>(), "Type of Query. Shortest Path[1], Delay-Update[2], Full-Mixed[3], Half-Mixed[4]. Default:1.")
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

    if ( vm.count("queries"))
        numQueries = vm["queries"].as<unsigned int>();

    if ( vm.count("mod"))
        mod = vm["mod"].as<int>();

    if ( vm.count("multi"))
        multi = vm["multi"].as<int>();

    if ( vm.count("format"))
        format = vm["format"].as<int>();

    if ( vm.count("types"))
        vehTypes = vm["types"].as<std::vector<int> >();

    if( format == 1)
        basePath = basePath + "TspNet/" + netName + "/";
    else
        basePath = basePath + "GTFS/" + netName + "/";

    qFile = "./" + netName + ".queries";

    Timer timer;

    switch( graphVariant)
    {
        //ADJ
        case 1:
            shell<Graph>( format, mod, numQueries, basePath, qFile, multi, vehTypes);
            break;

        //dynFS
        case 2:
            shell<fsGraph>( format, mod, numQueries, basePath, qFile, multi, vehTypes);
            break;

        //PMG
        case 3:
            shell<pmaGraph>( format, mod, numQueries, basePath, qFile, multi, vehTypes);
            break;

        //sFS
        case 4:
            shell<pmaGraph>( format, mod, numQueries, basePath, qFile, multi, vehTypes);
            break;
    }

    return 0;
}
