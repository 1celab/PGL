#ifndef DYNTM_GTFS2_READER_H
#define DYNTM_GTFS2_READER_H

#include "../Structs/definitions.h"
#include "netBuilder.h"

#if !defined(DLOG)
    #define DLOG(...) ;
#endif

#if !defined(ILOG)
    #define ILOG(...) ;
#endif

#if !defined(DBG)
    #define DBG(...) ;
#endif

namespace DynTM
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
class GTFS2Reader : public DynTMBuilder<TspNetwork>
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
    DynTMBuilder<TspNetwork>( tspNetwork, realMod),
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
    std::map<std::string, StationID> stIndex;
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
        StationID stationId = 0;
        std::string temp;
        unsigned int minTransferTime = 0;


        //read file's column labels (first line)
        std::vector<std::string> fields;
        std::getline( in, line);
        fields = explode( line);

        int stopIdIndex = findColumn( fields, "stop_id");
        if( stopIdIndex == fields.size()) stopIdIndex = 0;
        int nameIndex = findColumn( fields, "stop_name");
        if( nameIndex == fields.size()) nameIndex = 2;
        int latIndex = findColumn( fields, "stop_lat");
        if( latIndex == fields.size()) latIndex = 4;
        int lonIndex = findColumn( fields, "stop_lon");
        if( lonIndex == fields.size()) lonIndex = 5;

        ILOG("[*] Filtering fields: (%s, %s, %s, %s)\n", fields[stopIdIndex].c_str(), fields[nameIndex].c_str(), fields[latIndex].c_str(), fields[lonIndex].c_str());

        //stop id in [0,numStops-1]
        std::string name, stopIdStr;
        double lon, lat;

        //read stops
        while( std::getline( in, line))
        {
            fields = explode( line);
            stopIdStr = fields[stopIdIndex];
            name = fields[nameIndex];
            lon = str2Float(fields[latIndex]);
            lat = str2Float(fields[lonIndex]);

            DBG
            (
                if( fabs(lat) > 90.0 || fabs(lon) > 180.0)
                {
                    DLOG("[-] Invalid coords: (%f,%f)\n", lat, lon);
                }
            )

            stIndex[stopIdStr] = stationId;

            //create a station
            Station& station = tspNet.addStation();
            station.id = stationId;
            station.lon = lon; //at first stop
            station.lat = lat;
            station.name = name;
            station.minTransferTime = minTransferTime;
            stationId++;
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

        std::vector<std::string> fields;
        //read file's column labels (first line)
        std::getline( in, line);
        fields = explode( line);

        int routeIdIndex = findColumn( fields, "route_id");
        if( routeIdIndex == fields.size()) routeIdIndex = 0;
        int routeTypeIndex = findColumn( fields, "route_type");
        if( routeTypeIndex == fields.size()) routeTypeIndex = 4;

        ILOG("[*] Filtering fields: (%s, %s)\n", fields[routeIdIndex].c_str(), fields[routeTypeIndex].c_str());


        //map GTFS route id -> GTFS vehicle type id
        while( std::getline( in, line))
        {
            fields = explode( line);

            routeId = fields[routeIdIndex];
            vehicleTypeId = str2Int(fields[routeTypeIndex]);

            route2vehicle[routeId] = vehicleTypeId;
            vehTypes[vehicleTypeId];
        }


        //create vehicle types
        unsigned int i = 0;
        std::map<VehicleTypeID, VehicleTypeID>::iterator it;
        for( it = vehTypes.begin(); it != vehTypes.end(); ++it)
        {
            Vehicle& vehicle = tspNet.addVehicle();
            vehicle.name = transportModes[it->first];
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

        std::vector<std::string> fields;
        std::getline( in, line);
        fields = explode( line);

        int routeIdIndex = findColumn( fields, "route_id");
        if( routeIdIndex == fields.size()) routeIdIndex = 0;
        int tripIdIndex = findColumn( fields, "trip_id");
        if( tripIdIndex == fields.size()) tripIdIndex = 2;

        ILOG("[*] Filtering fields: (%s, %s)\n", fields[routeIdIndex].c_str(), fields[tripIdIndex].c_str());

        std::string tripId;
        std::string routeId;

        //read stop transit times
        while( std::getline( in, line))
        {
            fields = explode( line);

            routeId = fields[routeIdIndex];
            tripId = fields[tripIdIndex];

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
        int hr, min, sec;
        VehicleID vehicleId = 0;
        std::string currTripId, nextTripId;
        std::vector<std::string> depStr;

        //ignore first (comment) line
        //in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

        {
            std::vector<std::string> fields;
            std::getline( in, line);
            fields = explode( line);

            int tripIdIndex = findColumn( fields, "route_id");
            if( tripIdIndex == fields.size()) tripIdIndex = 0;
            int arrTimeIndex = findColumn( fields, "arrival_time");
            if( arrTimeIndex == fields.size()) arrTimeIndex = 1;
            int depTimeIndex = findColumn( fields, "departure_time");
            if( depTimeIndex == fields.size()) depTimeIndex = 2;
            int stopIdIndex = findColumn( fields, "stop_id");
            if( stopIdIndex == fields.size()) stopIdIndex = 3;

            ILOG("[*] Filtering fields: (%s, %s, %s, %s)\n", fields[tripIdIndex].c_str(), fields[arrTimeIndex].c_str(), fields[depTimeIndex].c_str(), fields[stopIdIndex].c_str());

            StationID depStopId, arrStopId;
            std::string stopId;
            Time depTime, arrTime;
            Distance travelTime;
            int hr, min, sec;
            std::string currTripId="";
            std::string nextTripId="";
            VehicleID vehicleId;
            VehicleTypeID tspModeId;

            vehicleId = 0;

            //read stop transit times
            while( std::getline( in, line))
            {
                fields = explode( line);

                nextTripId = fields[tripIdIndex];
                depStr = explode(fields[arrTimeIndex], ':', ':');
                hr = str2Int(depStr[0]);
                min = str2Int(depStr[1]);
                sec = str2Int(depStr[2]);//TODO enable seconds
                stopId = fields[stopIdIndex];

                arrTime = ( hr * 60 + min);
                if( arrTime < depTime)
                {
                    travelTime = (1440 + arrTime) - depTime;
                }
                else
                {
                    travelTime = arrTime - depTime;
                    arrTime = arrTime % 1440;
                }

                arrStopId = stIndex[stopId];

                //trip connections
                if( nextTripId == currTripId)
                {
                    //skip self loop connection
                    if( arrStopId != depStopId)
                    {

                        Station& depStation = tspNet.stations[depStopId];
                        depStation.addAdjacentStation( arrStopId);

                        depEvent.time = depTime;
                        depEvent.arrTime = arrTime;
                        depEvent.arrStId = arrStopId;
                        depEvent.vehicleId = vehicleId;
                        depEvent.vehicleTypeId = vehTypes[ route2vehicle[ trip2route[currTripId] ] ];
                        timeEvents[depStationId].departures.push_back( depEvent);
                        numConnections++;
                    }
                }

                else
                {
                    vehicleId++;
                    currTripId = nextTripId;
                    tspModeId = vehTypes[route2vehicle[trip2route[currTripId]]];
                }

                depStopId = arrStopId;
                depTime = arrTime % 1440;
            }
        }

        //clear auxilary containers
        std::map<std::string, StationID> empty1;
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
            isRealistic = false;
            return;
        }

        StationID stationId, stopId;
        int temp;

        std::vector<std::string> fields;

        std::getline( in, line);
        fields = explode( line);

        //from_stop_id, to_stop_id, transfer_type, min_transfer_time
        int fromStopIdIndex = findColumn( fields, "from_stop_id");
        if( fromStopIdIndex == fields.size()) fromStopIdIndex = 0;
        int toStopIdIndex = findColumn( fields, "to_stop_id");
        if( toStopIdIndex == fields.size()) toStopIdIndex = 1;
        int tsfTypeIndex = findColumn( fields, "transfer_type");
        if( tsfTypeIndex == fields.size()) tsfTypeIndex = 2;
        int tsfTimeIndex = findColumn( fields, "min_transfer_time");
        if( tsfTimeIndex == fields.size()) tsfTimeIndex = 3;

        ILOG("[*] Filtering fields: (%s, %s, %s, %s)\n", fields[0].c_str(), fields[1].c_str(), fields[2].c_str(), fields[3].c_str());

        std::string fromStopId, toStopId;
        Time minTransferTime;
        int transferType;

        //read stop transit times
        while( std::getline( in, line))
        {
            fields = explode( line);

            fromStopId = fields[fromStopIdIndex];
            toStopId = fields[toStopIdIndex];
            transferType = str2Int(fields[tsfTypeIndex]);
            minTransferTime = str2Int(fields[tsfTimeIndex]) / 60.0;

            if( transferType != 2 || fromStopId == toStopId) continue;

            stopId = stIndex[fromStopId];
            Station& station = tspNet.stations[stopId];
            if ( station.minTransferTime < minTransferTime)
                station.minTransferTime = minTransferTime;

            DBG
            (
                if( minTransferTime > 1440)
                    DLOG("[-] Invalid transfer time (%f>1440)\n", minTransferTime);
            )
        }

        in.close();
    }

   /**
     * @brief Return the column which has the given label.
     * @param fields A vector of strings representing the columns.
     * @param columnLabel A label string.
     * @return The column index.
     **/
    int findColumn( std::vector<std::string>& fields, const char* columnLabel)
    {
        return std::distance( fields.begin(), std::find( fields.begin(), fields.end(), columnLabel));
    }

    /**
     * @brief Returns a vector of strings, each of which is a substring of string
     * formed by splitting it on boundaries formed by the string delimiter.
     * @param str The input string.
     * @param firstDelim The first delimiter-character.
     * @param secondDelim The second delimiter-character.
     * @return A vector separating each string token by the specified delimiter.
     **/
    std::vector<std::string> explode( const std::string& str, const char firstDelim='"', const char secondDelim=',')
    {
        std::vector<std::string> tokens;
        std::istringstream iss(str);

        for( std::string token; std::getline(iss, token, secondDelim);)
        {
            tokens.push_back(std::move(token));
            if( tokens.back().empty() == false && tokens.back()[0] == firstDelim)
            {
                if( tokens.back().back() == firstDelim)
                    tokens.back() = tokens.back().substr(1, tokens.back().size()-2);
                else
                {
                    tokens.back().erase(0,1);
                    std::getline(iss, token, firstDelim);
                    tokens.back() += token;
                }
            }
        }

        return tokens;
    }

    /**
     * @brief Parses the string, interpreting its content as a floating point number and returns its value.
     * @param str The input string.
     * @return The float number.
     **/
    float str2Float( const std::string& str)
    {
        return atof( str.c_str());
    }

    /**
     * @brief Parses the string, interpreting its content as an integer number and returns its value.
     * @param str The input string.
     * @return The integer number.
     **/
    int str2Int( const std::string& str)
    {
        return atoi( str.c_str());
    }

    //extended GTFS Route Types
    std::map<unsigned int, std::string> transportModes = {
    {0, "light rail"},
    {1, "subway"},
    {2, "rail"},
    {3, "bus"},
    {4, "ferry"},
    {5, "cable car"},
    {6, "gondola"},
    {7, "funicular"},
    {100, "Railway Service"},
    {101, "High Speed Rail Service"},
    {102, "Long Distance Trains"},
    {103, "Inter Regional Rail Service"},
    {104, "Car Transport Rail Service"},
    {105, "Sleeper Rail Service"},
    {106, "Regional Rail Service"},
    {107, "Tourist Railway Service"},
    {108, "Rail Shuttle (Within Complex)"},
    {109, "Suburban Railway"},
    {110, "Replacement Rail Service"},
    {111, "Special Rail Service"},
    {112, "Lorry Transport Rail Service"},
    {113, "All Rail Services"},
    {114, "Cross-Country Rail Service"},
    {115, "Vehicle Transport Rail Service"},
    {116, "Rack and Pinion Railway"},
    {117, "Additional Rail Service"},
    {200, "Coach Service"},
    {201, "International Coach Service"},
    {202, "National Coach Service"},
    {203, "Shuttle Coach Service"},
    {204, "Regional Coach Service"},
    {205, "Special Coach Service"},
    {206, "Sightseeing Coach Service"},
    {207, "Tourist Coach Service"},
    {208, "Commuter Coach Service"},
    {209, "All Coach Services"},
    {300, "Suburban Railway Service"},
    {400, "Urban Railway Service"},
    {401, "Metro Service"},
    {402, "Underground Service"},
    {403, "Urban Railway Service"},
    {404, "All Urban Railway Services"},
    {405, "Monorail"},
    {500, "Metro Service"},
    {600, "Underground Service"},
    {700, "Bus Service"},
    {701, "Regional Bus Service"},
    {702, "Express Bus Service"},
    {703, "Stopping Bus Service"},
    {704, "Local Bus Service"},
    {705, "Night Bus Service"},
    {706, "Post Bus Service"},
    {707, "Special Needs Bus"},
    {708, "Mobility Bus Service"},
    {709, "Mobility Bus for Registered Disabled"},
    {710, "Sightseeing Bus"},
    {711, "Shuttle Bus"},
    {712, "School Bus"},
    {713, "School and Public Service Bus"},
    {714, "Rail Replacement Bus Service"},
    {715, "Demand and Response Bus Service"},
    {716, "All Bus Services"},
    {800, "Trolleybus Service"},
    {900, "Tram Service"},
    {901, "City Tram Service"},
    {902, "Local Tram Service"},
    {903, "Regional Tram Service"},
    {904, "Sightseeing Tram Service"},
    {905, "Shuttle Tram Service"},
    {906, "All Tram Services"},
    {1000, "Water Transport Service"},
    {1001, "International Car Ferry Service"},
    {1002, "National Car Ferry Service"},
    {1003, "Regional Car Ferry Service"},
    {1004, "Local Car Ferry Service"},
    {1005, "International Passenger Ferry Service"},
    {1006, "National Passenger Ferry Service"},
    {1007, "Regional Passenger Ferry Service"},
    {1008, "Local Passenger Ferry Service"},
    {1009, "Post Boat Service"},
    {1010, "Train Ferry Service"},
    {1011, "Road-Link Ferry Service"},
    {1012, "Airport-Link Ferry Service"},
    {1013, "Car High-Speed Ferry Service"},
    {1014, "Passenger High-Speed Ferry Service"},
    {1015, "Sightseeing Boat Service"},
    {1016, "School Boat"},
    {1017, "Cable-Drawn Boat Service"},
    {1018, "River Bus Service"},
    {1019, "Scheduled Ferry Service"},
    {1020, "Shuttle Ferry Service"},
    {1021, "All Water Transport Services"},
    {1100, "Air Service"},
    {1101, "International Air Service"},
    {1102, "Domestic Air Service"},
    {1103, "Intercontinental Air Service"},
    {1104, "Domestic Scheduled Air Service"},
    {1105, "Shuttle Air Service"},
    {1106, "Intercontinental Charter Air Service"},
    {1107, "International Charter Air Service"},
    {1108, "Round-Trip Charter Air Service"},
    {1109, "Sightseeing Air Service"},
    {1110, "Helicopter Air Service"},
    {1111, "Domestic Charter Air Service"},
    {1112, "Schengen-Area Air Service"},
    {1113, "Airship Service"},
    {1114, "All Air Services"},
    {1200, "Ferry Service"},
    {1300, "Telecabin Service"},
    {1301, "Telecabin Service"},
    {1302, "Cable Car Service"},
    {1303, "Elevator Service"},
    {1304, "Chair Lift Service"},
    {1305, "Drag Lift Service"},
    {1306, "Small Telecabin Service"},
    {1307, "All Telecabin Services"},
    {1400, "Funicular Service"},
    {1401, "Funicular Service"},
    {1402, "All Funicular Service"},
    {1500, "Taxi Service"},
    {1501, "Communal Taxi Service"},
    {1502, "Water Taxi Service"},
    {1503, "Rail Taxi Service"},
    {1504, "Bike Taxi Service"},
    {1505, "Licensed Taxi Service"},
    {1506, "Private Hire Service Vehicle"},
    {1507, "All Taxi Services"},
    {1600, "Self Drive"},
    {1601, "Hire Car"},
    {1602, "Hire Van"},
    {1603, "Hire Motorbike"},
    {1604, "Hire Cycle"},
    {1700, "Miscellaneous Service"},
    {1701, "Cable Car"},
    {1702, "Horse-drawn Carriage"}
    };

 protected:

    typedef DynTMBuilder<TspNetwork> Base;
    using Base::G;
    using Base::m_ids;
    using Base::tspNet;
    using Base::stations;
    using Base::timeEvents;
    using Base::buildNetwork;
    using Base::numConnections;
    using Base::isRealistic;
    using typename Base::DepEvent;



};

};

#endif
