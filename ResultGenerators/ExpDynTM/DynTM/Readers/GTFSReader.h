#ifndef DYNTM_GTFS_READER_H
#define DYNTM_GTFS_READER_H

#include "../Structs/definitions.h"
#include "netBuilder.h"

namespace DynTM
{

/**
 * @class GTFSReader
 *
 * @brief This class implements a reader for the DynTM graph model. 
 * Its use is for loading data files (GTFS format), containing time expanded transportation network info.
 * @tparam TspNetwork The time expanded data structure used for transporation network representation.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename TspNetwork>
class GTFSReader : public DynTMBuilder<TspNetwork>
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

    typedef typename TspNetwork::VehicleContainer                   VehicleContainer;
    typedef typename TspNetwork::Vehicle                            Vehicle;

    /**
     * @brief Constructor. Creates an instance of GTFSReader.
     * @param tspNetwork The time expanded network.
     * @param stopFilename The file with the stops info.
     * @param connectionFilename The file with the connections info.
     * @param realMod The realistic flag.
     **/
    GTFSReader( TspNetwork& tspNetwork, const std::string& stopFilename, const std::string& connectionFilename, const std::string& vehicleFilename, bool realMod = true) :
    DynTMBuilder<TspNetwork>( tspNetwork, realMod),
    m_stopFilename( stopFilename), m_connectionFilename( connectionFilename), m_vehicleFilename(vehicleFilename)
    {}

    /**
     * @brief Reads the data and creates the network.
     **/
    void read()
    {
        clear();

        importStops();
        importVehicles();
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

    std::string m_stopFilename;
    std::string m_connectionFilename;
    std::string m_vehicleFilename;

    //vehicle type id (global) to vehicle type id (local)
    std::map<VehicleTypeID, VehicleTypeID> vehTypes;

    /**
     * @brief Imports stop data.
     **/
    void importStops()
    {
        std::ifstream in;
        std::string line;

        //open file "stops.csv"
        in.open( m_stopFilename.c_str());
        std::cout << "\nReading stations from " << m_stopFilename << " ... " << std::flush;

        if( !in)
        {
            std::cerr << "\nError opening " << m_stopFilename << std::endl;
            exit(-1);
        }
       
        //station ids:0,1,2,..
        StationID stationId = 0;
        Time minTransferTime;
        std::string name;
        float lon, lat;
        int temp;

        //ignore first (comment) line
        in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

        //read stations
        while( std::getline( in, line))
        {
            std::replace( line.begin(), line.end(), ' ', '_'); //only for string name
            std::replace( line.begin(), line.end(), ',', ' ');

            //std::cout << "\nline:" << line;

            //read a line from file (info for a station)
            std::stringstream data( line);

            data >> temp >> temp >> temp
                 >> lon >> lat >> minTransferTime >> temp >> name;

            //std::cout << "\nstopID:" << stopID << " stationID:" 
            //            << stId << " minTransferTime:" << minTransferTime << " name:" << name;
            //{int a; std::cin>>a;}

            assert( minTransferTime < 1440);
            assert( fabs(lat) <= 90);
            assert( fabs(lon) <= 180);

            if( isRealistic == false)
                minTransferTime = 0;

            Station& station = tspNet.addStation();
            station.id = stationId;
            station.lon = lon;
            station.lat = lat;
            station.minTransferTime = minTransferTime;
            station.name = name;
            stationId++;
        }

        in.close();

        std::cout << "done!\n";
    }


    /**
     * @brief Imports vehicle data.
     **/
    void importVehicles()
    {
        std::ifstream in;
        std::string line;
        
        //open file "vehicles.csv"
        in.open( m_vehicleFilename.c_str());
        std::cout << "\nReading vehicle types from " << m_vehicleFilename << " ... " << std::flush;

        if( !in)
        {
            std::cerr << "\nError opening " << m_vehicleFilename << std::endl;
            exit(-1);
        }
      
        //ignore first (comment) line
        in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
       
        //vehicle type names (basic route types)        
        std::map<VehicleTypeID, std::string> vehicleName;
        std::map<VehicleTypeID, std::string> vehicleGroupName;
        VehicleTypeID vehTypeId;
        std::string vehName;

        //read vehicles
        while( std::getline( in, line))
        {
            //read a line from file (info for a connection)
            std::stringstream data( line);
            std::string temp;            

            std::getline( data, temp , ',');
            vehTypeId = atoi( temp.c_str());
            std::getline( data, temp, ',');
            std::getline( data, temp, ',');
            std::getline( data, vehName, '\n');
            vehicleName[vehTypeId] = vehName;
            vehTypes[vehTypeId];
        }
        
        in.close();
		    
        std::cout << "done!" << std::endl;

        //group vehicle types
        /*vehicleGroupName[0] = "train"; 
        vehicleGroupName[1] = "bus";
        vehicleGroupName[2] = "ferry";
        vehicleGroupName[3] = "tram";

        for( unsigned int j=0; j<4; j++)
        {
            Vehicle& vehicle = tspNet.addVehicle();
            vehicle.name = vehicleGroupName[j];
            vehicle.typeId = j;
        }

        //set grouped vehicle types
        std::map<VehicleTypeID, VehicleTypeID>::iterator it;
        for( it = vehTypes.begin(); it != vehTypes.end(); ++it)
        {
            if( it->first == 0 || it->first == 11)
                vehTypes[it->first] = 0;
            else if( it->first < 11) 
                vehTypes[it->first] = 1;
            else if( it->first < 14) 
                vehTypes[it->first] = 2;
            else
                vehTypes[it->first] = 3;
        }
        */

        //set single vehicle types
        unsigned int i = 0;
        std::map<VehicleTypeID, VehicleTypeID>::iterator it;
        for( it = vehTypes.begin(); it != vehTypes.end(); ++it)
        {
            Vehicle& vehicle = tspNet.addVehicle();
            vehicle.name = vehicleName[it->first];
            vehTypes[it->first] = i;
            vehicle.typeId = it->first;
            i++;
        }


        std::cout << "\nVehicle types:\n";
	    for(int i = 0; i < tspNet.vehicles.size(); i++)
		    std::cout << "TypeID:" << tspNet.vehicles[i].typeId  << " Name:" << tspNet.vehicles[i].name << "\n";

        std::cout << std::endl;
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

        StopID stopFromId, stopToId;
        VehicleID vehicleId, routeId, tripId;
        VehicleTypeID vehicleTypeId;
        Time depTime, arrTime;
        Distance travelTime;
        DepEvent depEvent;
        int temp;

        //number of stations (stops)
        const SizeType numStations = stations.size();

        //ignore first (comment) line
        in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

        //read connections
        while( std::getline( in, line))
        {
            std::replace( line.begin(), line.end(), ',', ' ');

            //read a line from file (info for a connection)
            std::stringstream data( line);
          
            //std::cout << "\nline:" << line;

            //set connection info :
            //vehicleId, departure-stationId, arrival-stationId,
            //departureTime (from departure-station), 
            //arrivalTime (to arrival-station), 
            //travelTime (from departure-station to arrival-station)
              
            data >> temp >> tripId >> routeId >> stopFromId >> stopToId >> temp >> depTime >> travelTime 
                 >> temp >> temp >> temp >> temp >> temp >> temp >> temp >> vehicleTypeId;
                 
            //vehicleId = routeId;
            vehicleId = routeId * 10000 + tripId;
            //std::cout << "\nvehicleId:" << vehicleId << " stopFromId:" 
            //            << stopFromId << " stopToId:" << stopToId << " depTime:" << depTime << " duration:" << travelTime;
            //{int a; std::cin>>a;}

            //skip invalid vehicle types
            if( vehicleTypeId > vehicles.size())
               continue;

            //skip inner station connection
            if( stopFromId == stopToId)
                continue;

            //convert to mins
            depTime = int( round( ( depTime % 86400) / 60.0)) % 1440;
            travelTime = round( travelTime / 60.0);
            arrTime = ( depTime + travelTime) % 1440;
            assert( arrTime < 1440);
            assert( depTime < 1440);

            //TODO skip zero travel time connections
            //if( travelTime == 0) continue;

            //add the connected adjacent arrival station of the depStation
            Station& depStation = stations[stopFromId];
            depStation.addAdjacentStation( stopToId);
            
            //add the departure time event to the depStation
            depEvent.time = depTime;
            depEvent.arrTime = depTime + travelTime;
            depEvent.arrStId = stopToId;
            depEvent.vehicleId = vehicleId;
            depEvent.vehicleTypeId = vehTypes[vehicleTypeId];
            timeEvents[stopFromId].departures.push_back( depEvent);

            numConnections++;
        }

        in.close();

        std::map<VehicleTypeID, VehicleTypeID> empty2;
        vehTypes.swap( empty2);

        std::cout << "done!"
                  << "\n\tstations:" << stations.size() << " cnxEdges:" << numConnections
                  << "\n\tdepNodes:" << numConnections << " arrNodes:" << numConnections
                  << std::endl;
    }

 protected:

    typedef DynTMBuilder<TspNetwork> Base;
    using Base::G;
    using Base::m_ids;
    using Base::tspNet;
    using Base::stations;
    using Base::vehicles;
    using Base::timeEvents;
    using Base::buildNetwork;
    using Base::numConnections;
    using Base::isRealistic;
    using typename Base::DepEvent;
};

};

#endif
