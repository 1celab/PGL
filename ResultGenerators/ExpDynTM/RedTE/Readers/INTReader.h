#ifndef REDTE_INT_READER_H
#define REDTE_INT_READER_H

#include "../Structs/definitions.h"
#include "netBuilder.h"

namespace RedTE
{

/**
 * @class INTReader
 *
 * @brief This class implements a reader for the RedTE graph model. 
 * Its use is for loading data files (INT format), containing time expanded transportation network info.
 * @tparam TspNetwork The time expanded data structure used for transporation network representation.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename TspNetwork>
class INTReader : RedTEBuilder<TspNetwork>
{

 public:

    typedef typename TspNetwork::GraphImpl                        GraphType;
    typedef typename TspNetwork::GraphImpl::SizeType              SizeType;

    typedef typename GraphType::NodeIterator                      NodeIterator;
    typedef typename GraphType::NodeDescriptor                    NodeDescriptor;
    typedef typename GraphType::EdgeIterator                      EdgeIterator;
    typedef typename GraphType::EdgeDescriptor                    EdgeDescriptor;
    typedef typename GraphType::InEdgeIterator                    InEdgeIterator;

    typedef typename TspNetwork::StationContainer                 StationContainer;
    typedef typename TspNetwork::Station                          Station;

    typedef typename TspNetwork::VehicleContainer                 VehicleContainer;
    typedef typename TspNetwork::Vehicle                          Vehicle;

    /**
     * @brief Constructor. Creates an instance of INTReader.
     * @param tspNetwork The time expanded network.
     * @param stationFilename The file with the stations info.
     * @param connectionFilename The file with the connections info.
     * @param realMod The realistic flag.
     **/
    INTReader( TspNetwork& tspNetwork, const std::string& stationFilename, const std::string& connectionFilename, bool realMod = true) :
    RedTEBuilder<TspNetwork>( tspNetwork, realMod),
    m_stationFilename( stationFilename), m_connectionFilename( connectionFilename)
    {}

    /**
     * @brief Reads the data and creates the network.
     **/
    void read()
    {
        clear();

        importStations();
        importConnections();

        buildNetwork();
    }

    /**
     * @brief Returns the node ids.
     * @return A vector with the node ids.
     **/
    std::vector<NodeDescriptor>& getIds()
    {
        return m_ids;
    }

    /**
     * @brief Clears the data.
     **/
    void clear()
    {
        tspNet.clear();
        m_ids.clear();
    }

 private:

    std::string m_stationFilename;
    std::string m_connectionFilename;

    //station {global id (from data) -> local id (in vector)}
    std::map<StationID, StationID> index;

    /**
     * @brief Imports the stations data info.
     **/
    void importStations()
    {
        std::ifstream in;
        std::string line;

        //open file "stations.int"
        in.open( m_stationFilename.c_str());
        std::cout << "\nReading stations from " << m_stationFilename << " ... " << std::flush;
        if( !in)
        {
            std::cerr << "\nError opening " << m_stationFilename << std::endl;
            exit(-1);
        }
        
        //station ids:0,1,2,..
        StationID stationId = 0;

        //read stations
        while( std::getline( in, line))
        {
            //read a line from file (station info)
            std::stringstream data( line);

            //create a station
            Station& station = tspNet.addStation();

            //set station info (stationId, transitTime (mins), coords (lon, lat), name)
            data >> station.id >> station.transitTime
                 >> station.lon >> station.lat >> std::ws;

            //name of station
            std::string sname( (std::istreambuf_iterator<char>(data)), std::istreambuf_iterator<char>());
            station.name = sname;

            //if the time expanded graph model is not realistic then the transit time (inside the station) is zero
            if( isRealistic == false)
                station.transitTime = 0;

            //add station to index
            index[station.id] = stationId;
            stationId++;
        }

        Vehicle& vehicle = tspNet.addVehicle();
        vehicle.typeId = 0;
        vehicle.name = "vehicle(unimodal)";

        in.close();

        std::cout << "done!\n"
                  << "\tstations:" << stations.size() << std::endl;
    }

    /**
     * @brief Imports the connections data info.
     * @param timeEvents The time events.
     **/
    void importConnections()
    {
        std::ifstream in;
        std::string line;

        //open file "connections.int"
        in.open( m_connectionFilename.c_str());
        std::cout << "Reading connections from " << m_connectionFilename << " ... " << std::flush;
        if( !in) 
        {
            std::cerr << "\nError opening " << m_connectionFilename << std::endl;
            exit(-1);
        }

        timeEvents.resize( stations.size());
        numConnections = 0;

        StationID depStationId, arrStationId;
        VehicleID vehicleId;
        Time depTime, arrTime;
        Distance travelTime;
        DepEvent depEvent;
        ArrEvent arrEvent;

        //read connections
        while( std::getline( in, line))
        {
            //read a line from file (connection info)
            std::stringstream data( line);
            char token;
            data >> token;

            //skip comment-lines
            if( token == '%' || token == '*')
                continue;

            //read the rest connection info
            else
            {
                data.seekg( 0, std::ios::beg);

                //connection info :
                //vehicleId, departure-stationId, arrival-stationId,
                //departureTime (from departure-station), 
                //arrivalTime (to arrival-station), 
                //travelTime (from departure-station to arrival-station)
                data >> vehicleId >> depStationId >> arrStationId 
                     >> depTime >> arrTime >> travelTime;

                assert( depTime < 1440);
                assert( arrTime < 1440);

                //local depStation Id
                depStationId = index[depStationId];

                //local arrStation Id
                arrStationId = index[arrStationId];

                //add the connected adjacent arrival stations of depStation
                Station& depStation = stations[depStationId];
                depStation.addAdjacentStation( arrStationId);
     
                //add the arrival time event to the arrStation
                arrEvent.time = arrTime;
                arrEvent.arrTime = depTime + travelTime;
                arrEvent.vehicleId = vehicleId;
                arrEvent.vehicleTypeId = 0;
                timeEvents[arrStationId].arrivals.push_back( arrEvent);

                //add the departure time event to the depStation
                depEvent.time = depTime;
                depEvent.vehicleId = vehicleId;
                depEvent.vehicleTypeId = 0;
                depEvent.arrStId = arrStationId;
                depEvent.arrIt = (--timeEvents[arrStationId].arrivals.end());
                timeEvents[depStationId].departures.push_back( depEvent);

                numConnections++;
            }
        }

        in.close();

        std::cout << "done!"
                  << "\n\tstations:" << stations.size() << " cnxEdges:" << numConnections
                  << "\n\tdepNodes:" << numConnections << " arrNodes:" << numConnections
                  << std::endl;
    }

 protected:

    typedef RedTEBuilder<TspNetwork> Base;
    using Base::G;
    using Base::m_ids;
    using Base::tspNet;
    using Base::stations;
    using Base::timeEvents;
    using Base::buildNetwork;
    using Base::numConnections;
    using Base::isRealistic;
    using typename Base::DepEvent;
    using typename Base::ArrEvent;

};

};

#endif
