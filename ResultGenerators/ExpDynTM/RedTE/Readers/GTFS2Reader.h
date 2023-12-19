#ifndef REDTE_GTFS2_READER_H
#define REDTE_GTFS2_READER_H

#include "../Structs/definitions.h"
#include "netBuilder.h"

namespace RedTE
{

/**
 * @class GTFS2Reader
 *
 * @brief This class implements a reader for the DynTM time-expanded graph model. 
 * Its use is for loading data files (GTFS format), containing time expanded transportation network info.
 * @tparam TspNetwork The time expanded data structure used for transporation network representation.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename TspNetwork>
class GTFS2Reader : public RedTEBuilder<TspNetwork>
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
     * @brief Constructor. Creates an instance of GTFS2Reader.
     * @param tspNetwork The time expanded network.
     * @param stationFilename The file with the stations info.
     * @param connectionFilename The file with the connections info.
     * @param realMod The realistic flag.
     **/
    GTFS2Reader( TspNetwork& tspNetwork, const std::string& routesFilename, const std::string& tripsFilename, const std::string& stopFilename, 
                 const std::string& stopTimesFilename, const std::string& transfersFilename, bool realMod = true) :
    RedTEBuilder<TspNetwork>( tspNetwork, realMod),
    m_routesFilename( routesFilename), m_tripsFilename( tripsFilename),
    m_stopFilename( stopFilename), m_stopTimesFilename( stopTimesFilename), m_transfersFilename( transfersFilename) 
    {}

    /**
     * @brief Reads the data and creates the network.
     **/
    void read()
    {
        clear();

        importStations();
        importVehicles();
        importTrips();
        importTransfers();
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

    std::string m_routesFilename;
    std::string m_tripsFilename;
    std::string m_stopFilename;
    std::string m_stopTimesFilename;
    std::string m_transfersFilename;

    //stop ids (global) to station ids (local)
    std::map<StationID, StationID> stIndex;
    //vehicle type id (global) to vehicle type id (local)
    std::map<VehicleTypeID, VehicleTypeID> vehTypes;
    //trip id to its correspoding route id
    std::map<std::string, std::string> trip2route;
    //route id to its correspoding vehicle type id (global)
    std::map<std::string, VehicleTypeID> route2vehicle;

    /**
     * @brief Imports station data.
     **/
    void importStations()
    {
        std::ifstream in;
        std::string line;
        in.open( m_stopFilename.c_str());
        std::cout << "\nReading stops from " << m_stopFilename << " ... " << std::flush;
        if( !in)
        {
            std::cerr << "\nError opening " << m_stopFilename << std::endl;
            exit(-1);
        }

        //station ids:0,1,2,..
        StationID stopId, stationId = 0;
        std::string name, temp;
        unsigned int transitTime = 0;
        float lon, lat;
        
        //ignore first (comment) line
        in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

        //read stop transit times
        while( std::getline( in, line))
        {
            //std::cout << "\nline:" << line;
            //read a line from file (info for a station)
            std::stringstream data( line);

            ///stopId
            std::getline( data, temp, ',');
            stopId = atoi( temp.c_str());

            //stop_code
            std::getline( data, temp, ',');

            //stop name
            std::getline( data, temp, ',');
            name = temp;
            if( temp[0] == '"' && *(temp.end()-1) != '"')
            { 
                std::getline( data, temp, '"');
                name += temp;
                std::getline( data, temp, ',');
            }

            ///lat
            std::getline( data, temp, ',');
            lat = atof( temp.c_str());

            ///lon
            std::getline( data, temp, ',');
            lon = atof( temp.c_str());

            assert( fabs(lat) <= 90);
            assert( fabs(lat) <= 180);

            //std::cout << "\nstopID:" << stopID << " stationName:" << name  << " lon:" << lon << " lat:" << lat;
            //{int a; std::cin>>a;}

            stIndex[stopId] = stationId;

            //create a station
            Station& station = tspNet.addStation();
            station.id = stationId;
            station.lon = lon; //at first stop
            station.lat = lat;
            station.name = name;
            station.transitTime = transitTime;
            stationId++;

            //std::cout << "\nstationID:" << station.id 
            //            << " stationName:" << station.name  
            //            << " transitTime:" << station.transitTime;
        }

        in.close();

        std::cout << "done!\n"
                  << "\tstations:" << stationId << std::endl;
    }


    /**
     * @brief Imports vehicle data.
     **/
    void importVehicles()
    {
        std::ifstream in;
        std::string line;

        in.open( m_routesFilename.c_str());
        std::cout << "\nReading vehicle types from " << m_routesFilename << " ... " << std::flush;
        if( !in)
        {
            std::cerr << "\nError opening " << m_routesFilename << std::endl;
            exit(-1);
        }
        
        VehicleTypeID vehicleTypeId;
        std::string routeId;
        std::string temp;
        
        //ignore first (comment) line
        in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
        
        //map RouteID -> VehicleTypeID
        while( std::getline( in, line))
        {          
            //read a line from file
            std::stringstream data( line);
 
            std::getline( data, routeId, ',');
            std::getline( data, temp, ',');
            std::getline( data, temp, ',');
            std::getline( data, temp, ',');
            if( temp[0] == '"' && *(temp.end()-1) != '"')
            { 
                std::getline( data, temp, '"');
                std::getline( data, temp, ',');
            }
            std::getline( data, temp, ',');
            std::getline( data, temp, ',');
            vehicleTypeId = atoi( temp.c_str()); 
            
            route2vehicle[routeId] = vehicleTypeId;
            vehTypes[vehicleTypeId];
        }
        
        //vehicle type names (basic route types)
        std::map<VehicleTypeID, std::string> vehicleName;
        vehicleName[0] = std::string("light rail");
        vehicleName[1] = std::string("subway");
        vehicleName[2] = std::string("rail");
        vehicleName[3] = std::string("bus");
        vehicleName[4] = std::string("ferry");
        vehicleName[5] = std::string("cable car");
        vehicleName[6] = std::string("gondola");
        vehicleName[7] = std::string("funicular");

        //vehicle type names (extented route types)
        vehicleName[1000] = std::string("water Transport");
        vehicleName[900] = std::string("tram");
        vehicleName[800] = std::string("trolleybus");
        vehicleName[700] = std::string("bus");
        vehicleName[600] = std::string("underground");
        vehicleName[500] = std::string("metro");
        vehicleName[400] = std::string("urban railway");
        vehicleName[300] = std::string("suburban railway");
        vehicleName[200] = std::string("coach");
        vehicleName[100] = std::string("railway");
 	    vehicleName[109] = std::string("suburban railway");

        //create vehicle types
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

        std::cout << "done!"<<std::endl;

        std::cout << "\nVehicle types:\n";
	    for(int i = 0; i < tspNet.vehicles.size(); i++)
		    std::cout << "TypeID:" << tspNet.vehicles[i].typeId  << " Name:" << tspNet.vehicles[i].name << "\n";
    }
    
    
    /**
     * @brief Imports trip data.
     **/
    void importTrips()
    {
        std::ifstream in;
        std::string line;

        in.open( m_tripsFilename.c_str());
        std::cout << "\nReading trips from " << m_tripsFilename << " ... " << std::flush;
        if( !in)
        {
            std::cerr << "\nError opening " << m_tripsFilename << std::endl;
            exit(-1);
        }
        
        std::string tripId;
        std::string routeId;
        int temp;
        
        //ignore first (comment) line
        in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
        
        //read stop transit times
        while( std::getline( in, line))
        {
            std::replace( line.begin(), line.end(), ' ', '_');
            std::replace( line.begin(), line.end(), ',', ' ');

            //std::cout << "\nline:" << line;

            //read a line from file (info for a station)
            std::stringstream data( line);

            data >> routeId >> temp >> tripId;

            trip2route[tripId] = routeId;
        }
    }
 
    /**
     * @brief Imports the connections data info.
     * @param timeEvents The time events.
     **/
    void importConnections()
    {
        std::ifstream in;
        std::string line;
        in.open( m_stopTimesFilename.c_str());
        std::cout << "\nReading stop times from " << m_stopTimesFilename << " ... " << std::flush;
        if( !in)
        {
            std::cerr << "\nError opening " << m_stopTimesFilename << std::endl;
            exit(-1);
        }

        timeEvents.resize( tspNet.stations.size());
        numConnections = 0;

        StationID depStationId, arrStationId, stopId;
        Time depTime, arrTime;
        DepEvent depEvent;
        ArrEvent arrEvent;
        int hr, min, sec;
        VehicleID vehicleId = 0;
        std::string currTripId, nextTripId;
 
        //ignore first (comment) line
        in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

        //read first line
        std::getline( in, line);
        std::replace( line.begin(), line.end(), ':', ' ');
        std::replace( line.begin(), line.end(), ',', ' ');
        std::stringstream data( line);

        data >> currTripId;

        data >> hr >> min >> sec;
        data >> hr >> min >> sec;
        depTime = ( hr * 60 + min) % 1440;

        data >> stopId;
        assert( stIndex.find( stopId) != stIndex.end());
        depStationId = stIndex[stopId];

        //read stop transit times
        while( std::getline( in, line))
        {
            std::replace( line.begin(), line.end(), ':', ' ');
            std::replace( line.begin(), line.end(), ',', ' ');

            //read a line from file (info for a station)
            std::stringstream data( line);

            //std::cout << "\nline:" << line;
            data >> nextTripId;

            data >> hr >> min >> sec;
            data >> hr >> min >> sec;
            arrTime = ( hr * 60 + min);
 
            if( arrTime < depTime)
                arrTime += 1440;

            data >> stopId;
            assert( stIndex.find( stopId) != stIndex.end());
            arrStationId = stIndex[stopId];

            if( nextTripId == currTripId)
            {
                //add the connected adjacent arrival stations of depStation
                Station& depStation = tspNet.stations[depStationId];
                depStation.addAdjacentStation( arrStationId);

                //add the arrival time event to the arrStation
                arrEvent.time = arrTime % 1440;
                arrEvent.arrTime = arrTime;
                arrEvent.vehicleId = vehicleId;
                arrEvent.vehicleTypeId = vehTypes[ route2vehicle[ trip2route[currTripId] ] ];
                timeEvents[arrStationId].arrivals.push_back( arrEvent);

                //add the departure time event to the depStation
                depEvent.time = depTime;
                depEvent.arrIt = (--timeEvents[arrStationId].arrivals.end());
                depEvent.arrStId = arrStationId;
                depEvent.vehicleId = vehicleId; 
                depEvent.vehicleTypeId = vehTypes[ route2vehicle[ trip2route[currTripId] ] ];
                timeEvents[depStationId].departures.push_back( depEvent);

                numConnections++;

                //std::cout << "\nfrom [" << depStationId << "] dep" << depTime << " to [" << arrStationId << "] arrTime:" << arrTime;
                //{int a; std::cin>>a;}
            }

            else
            {
                vehicleId++;
                currTripId = nextTripId;
            }

            depStationId = arrStationId;
            depTime = arrTime % 1440;
        }

        //clear auxilary containers
        std::map<StationID, StationID> empty1;
        stIndex.swap( empty1);

        std::map<VehicleTypeID, VehicleTypeID> empty2;
        vehTypes.swap( empty2);

        std::map<std::string, std::string> empty3;
        trip2route.swap( empty3);

        std::map<std::string, VehicleTypeID> empty4;
        route2vehicle.swap( empty4);

        in.close();

        std::cout << "done!"
                  << "\n\tstations:" << stations.size() << " cnxEdges:" << numConnections
                  << "\n\tdepNodes:" << numConnections << " arrNodes:" << numConnections
                  << std::endl;
    }

    /**
     * @brief Imports the stops for any vehicle.
     **/
    void importTransfers()
    {

        if( isRealistic == false)
            return;

        std::ifstream in;
        std::string line;
        in.open( m_transfersFilename.c_str());
        std::cout << "\nReading transfers from " << m_transfersFilename << " ... " << std::flush;
        if( !in)
        {
            std::cerr << "\nFailed opening " << m_transfersFilename << " - transfers are skipped" << std::endl;
            return;
        }

        StationID stationId, stopId;
        Time transitTime;
        int temp;

        //ignore first (comment) line
        in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

        //read stop transit times
        while( std::getline( in, line))
        {
            std::replace( line.begin(), line.end(), ',', ' ');

            //std::cout << "\nline:" << line;

            //read a line from file (info for a station)
            std::stringstream data( line);

            data >> stopId >> temp >> temp >> transitTime;

            stationId = stIndex[stopId];
            Station& station = tspNet.stations[stationId];

            transitTime = transitTime/60;
            if ( station.transitTime < transitTime)
                station.transitTime = transitTime;

            assert( station.transitTime < 1440);

            //std::cout << "\nstopID:" << stopID << " transitTime:" << station.transitTime;
            //{int a; std::cin>>a;}
        }

        in.close();
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
