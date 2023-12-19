#ifndef TRANSPORT_NETWORK_H
#define TRANSPORT_NETWORK_H

#include <iostream>
#include <sstream>
#include <fstream>

#include <algorithm>
#include <vector>
#include <string>
#include <cmath>
#include <list>
#include <map>

#include "definitions.h"

//graph tsp node
struct TspNode
{
    TspNode() : stId(0), vhId(0), tpId(0), label('-'), time(0)
    {}

    //network info
    NodeID id;
    StationID stId;
    VehicleID vhId;
    VehicleTypeID tpId;

    //time event info ('d':departure, 'a':arrival, '-':undefined)
    char label;
    Time time;

    bool isDeparture() 
    {
        return ( label == 'd');
    }

    bool isArrival() 
    {
        return ( label == 'a');
    }

    bool operator <( const TspNode& node) const
    {
        if( time < node.time)
            return true;
        else
            return false;
    }
};

//graph tsp edge
struct TspEdge
{
    TspEdge(): weight(0)
    {}

    union{ Distance travelTime, weight; };
};

/**
 * @class TspNetwork
 *
 * @brief Time-expanded transport network.
 *
 * This class supports building a transport network, as a set of departure, arrival, stay and transfer events through the
 * stations.
 *
 * @tparam GraphType The graph data structure.
 * @author Andreas Paraskevopoulos
 *
 */
template<class GraphType>
class TspNetwork
{

 public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef typename GraphType::EdgeDescriptor                      EdgeDescriptor;
    typedef typename GraphType::InEdgeIterator                      InEdgeIterator;
    typedef typename GraphType::SizeType                            SizeType;
    typedef GraphType                                               GraphImpl;

    struct Station;
    typedef typename std::vector<Station> StationContainer;

    struct Vehicle;
    typedef typename std::vector<Vehicle> VehicleContainer;

    /**
     * @brief Constructor. Creates an instance of a (rail, bus, ...) transport network.
     **/
    TspNetwork()
    {}


    /**
     * @brief Clears the network.
     **/
    void clear()
    {
        stations.clear();
        G.clear();
    }

    /**
     * @brief Reserves space for the time-expanded graph.
     * @param numNodes The number of nodes.
     * @param numEdges The number of edges.
     **/
    void reserve( SizeType numNodes, SizeType numEdges)
    {
        G.reserve( numNodes, numEdges);
    }

    /**
     * @brief Returns the number of stations.
     * @return The number of stations.
     **/
    SizeType getNumStations() const
    {
        return stations.size();
    }

    /**
     * @brief Returns the number of time event-nodes.
     * @return The number of nodes.
     **/
    SizeType getNumNodes() const
    {
        return G.getNumNodes();
    }

    /**
     * @brief Returns the number of edges.
     * @return The number of edges.
     **/
    SizeType getNumEdges() const
    {
        return G.getNumEdges();
    }


    /**
     * @brief Adds a station in the network. 
     * @return The created station.
     **/
    Station& addStation()
    {
        stations.resize( stations.size()+1);
        return stations.back();
    }

    /**
     * @brief Adds a vehicle type in the network.
     * @return The vehicle type.
     **/
    Vehicle& addVehicle()
    {
        vehicles.resize( vehicles.size()+1);
        return vehicles.back();
    }

    /**
     * @brief Adds a departure time event in a station, where a vehicle begins its route.
     * @param stationId The station.
     * @param time The time that the departure event occurs.
     * @param vehicleId The vehicle that departs from the station.
     * @return The node (depNode) which represents the departure time event.
     **/
    /*NodeDescriptor addDepEvent( const StationID& stationId, const Time& time, const VehicleID& vehicleId)
    {
        const NodeDescriptor vD = G.insertNode();
        const NodeIterator v = G.getNodeIterator( vD);

        v->time = time;
        v->label = 'd';

        v->stId = stationId;
        v->vhId = vehicleId;

        return vD;
    }*/


    /**
     * @brief Adds an arrival time event in a station, where a vehicle ends its route.
     * @param stationId The station.
     * @param time The time that the arrival event occurs.
     * @param vehicleId The vehicle that arrives to the station.
     * @return The node (arrNode) which represents the arrival time event.
     **/
    /*NodeDescriptor addArrEvent( const StationID& stationId, const Time& time, const VehicleID& vehicleId)
    {
        const NodeDescriptor vD = G.insertNode();
        const NodeIterator v = G.getNodeIterator( vD);

        v->time = time;
        v->label = 'a';

        v->stId = stationId;
        v->vhId = vehicleId;

        return vD;
    }*/


    /**
     * @brief Adds a transfer/connection edge between two node - time events.
     * @param uD The tail node.
     * @param vD The head node.
     * @param travelTime The travel time through the edge.
     **/
    /*void insertEdge( NodeDescriptor uD, NodeDescriptor vD, Distance travelTime)
    {
        const EdgeDescriptor eD = G.insertEdge( uD, vD);
        const EdgeIterator e = G.getEdgeIterator( eD);
        e->travelTime = travelTime;
    }*/

    /**
     * @brief Prints the departure and arrival events of a station.
     * @param stationId The station.
     **/
    void printStationInfo(const StationID stationId)
    {
        const Station& station = stations[stationId];
        NodeIterator u, v, lastnode;
        EdgeIterator e, lastedge;

        std::cout << "\n\nSTATION:[" << stationId << "," << station.name <<"] transitTime=" << station.transitTime << "\n";
        std::cout << "numDeps:" << station.numDepEvents << "\n";

        if( station.numDepEvents > 0)
            std::cout << "firstDepNode:" << G.getNodeIterator( station.firstDepNode)->time << "\n";

        for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; u++)
        {
            if( u->stId == stationId)
            {
                if( u->isDeparture())
                {
                    std::cout << "\nDEP(" << u->time << ")" << "vh(" << u->vhId << ")";

                    for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; e++)
                    {
                        v = G.target( e);
                        if( v->isDeparture())
                            std::cout << "\n  ->dep(" << v->time << ")" << "vh(" << v->vhId << ")wt(" << e->weight << ")";
                        else
                            std::cout << "\n  ->arr(" << v->time << ")" << "vh(" << v->vhId << ")wt(" << e->weight << ")";
                    }
                }

                else
                {        
                    std::cout << "\nARR(" << u->time << ")" << "vh(" << u->vhId << ")";

                    for( e = G.beginEdges( u), lastedge = G.endEdges( u); e != lastedge; ++e)
                    {
                        v = G.target( e);
                        if( v->isDeparture())
                            std::cout << "\n  ->dep(" << v->time << ")" << "vh(" << v->vhId << ")wt(" << e->weight << ")";
                        else
                            std::cout << "\n  ->arr(" << v->time << ")" << "vh(" << v->vhId << ")wt(" << e->weight << ")";
                    }
                }
            }
        }
    }
 
    /**
     * @brief Checks if the time-expanded network is valid.
     **/
    bool isValid( const StationID stationId)
    {
        const Station& station = stations[stationId];
        NodeIterator u, v, lastnode;
        EdgeIterator e, lastedge;

        Distance travelTime;
        unsigned int numDepEvents = 0;
        NodeIterator minDepNode = G.endNodes();   
        Time minDepTime = 1440;

        NodeIterator firstDepNode = G.endNodes();       
        if( station.numDepEvents > 0)
            firstDepNode = G.getNodeIterator( station.firstDepNode);

        bool isValid = true;

        for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; u++)
        {
            if( u->stId == stationId)
            {
                //departure tail
                if( u->isDeparture())
                {
                    numDepEvents++;

                    if( u->time < minDepTime)
                    {
                        minDepNode = u;   
                        minDepTime = u->time;     
                    }

                    for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; e++)
                    {
                        v = G.target( e);

                        //departure head
                        if( v->isDeparture())
                        {
                            if( u == v)
                            {
                                std::cout << "\nerror: dep self loop";
                                isValid = false;
                            }

                            if( v == firstDepNode)
                            {
                                if( v->time > u->time)
                                {
                                    std::cout << "\nerror: unsorted depEvents (firstDepNode)"
                                              << "\ndep(" << v->time << ") > dep(" << u->time << ")";
                                    isValid = false;
                                }

                                else
                                {
                                    travelTime = ( 1440 - u->time) + v->time;
                                    if( travelTime != e->weight)
                                    {
                                        std::cout << "\nerror: wrong weight"
                                                  << "\ndep(" << v->time << ")->dep(" << u->time << ")["<< e->weight <<"] should be"
                                                  << " dep(" << v->time << ")->dep(" << u->time << ")["<< travelTime <<"]\n";

                                        isValid = false;
                                    }
                                }
                            }

                            else
                            {   
                                if( v->time < u->time) 
                                {
                                    travelTime = u->time - v->time;
                                    std::cout << "\nerror: unsorted depEvents (intermediate)"
                                              << "\ndep(" << u->time << ")->dep(" << v->time << ")["<< e->weight <<"] should be"
                                              << " dep(" << v->time << ")->dep(" << u->time << ")["<< travelTime <<"]\n";
                                    isValid = false;
                                }

                                else
                                {
                                    travelTime = v->time - u->time;
                                    if( travelTime != e->weight )
                                    {
                                        std::cout << "\nerror: wrong weightX"
                                                  << "\ndep(" << u->time << ")->dep(" << v->time << ")["<< e->weight <<"] should be"
                                                  << " dep(" << u->time << ")->dep(" << v->time << ")["<< travelTime <<"]\n";

                                        isValid = false;
                                    }
                                }
                            }
                        }

                        //arrival head
                        else
                        {
                            if( u->stId == v->stId)
                            {
                                std::cout << "\nerror: dep-arr within the station";
                                isValid = false;
                            }

                            if( u->vhId != v->vhId)
                            {
                                std::cout << "\nerror: wrong dep-arr connection";
                                isValid = false;
                            }

                            Time arrTime = (u->time + e->weight) % 1440;

                            if( v->time != arrTime)
                            {
                                std::cout << "\nerror: wrong weight or arr time"
                                          << "\ndep(" << u->time << ")->arr(" << v->time << ")["<< e->weight <<"] should be"
                                          << " dep(" << u->time << ")->arr(" << arrTime << ")["<< e->weight <<"]\n";

                                isValid = false;
                            }
                        }
                    }
                }

                //arrival tail
                else
                {
                    for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; e++)
                    {
                        v = G.target( e);

                        //departure head
                        if( v->isDeparture())
                        {
                            if( station.numDepEvents > 1)
                            {
                                bool depDetected = false;
                                NodeIterator depNode = firstDepNode;
                                for( unsigned int j = 0; j < station.numDepEvents; j++)
                                {
                                    //arrival-departure edge
                                    if( depNode->time >= ( station.transitTime + u->time))
                                    {
                                        //compute the time a passenger needs to reach the next earliest departure
                                        //diff >= (station's transit time for changing train)
                                        travelTime = depNode->time - u->time;

                                        if( v != depNode)
                                        {
                                            std::cout << "\nerror: wrong arr-dep connection"
                                                      << "\narr(" << u->time << ")->dep(" << v->time << ")["<< e->weight <<"] should be"
                                                      << " arr(" << u->time << ")->dep(" << depNode->time << ")["<< travelTime <<"]\n";


                                            isValid = false;
                                        }
                    

                                        else if( e->weight != travelTime)
                                        {
                                            std::cout << "\nerror: wrong weight at arr-dep connection"
                                                      << "\narr(" << u->time << ")->dep(" << v->time << ")["<< e->weight <<"] should be"
                                                      << " arr(" << u->time << ")->dep(" << depNode->time << ")["<< travelTime <<"]\n";

                                            isValid = false;
                                        }

                                        depDetected = true;
                                        break;
                                    }

                                    depNode = G.target(--G.endEdges( depNode));

                                    if( depNode->isArrival())
                                    {
                                        std::cout << "\nerror: invalid descend";
                                        isValid = false;
                                    }
                                }

                                //special case: if there is no departure after the arrival until the midnight
                                if( depDetected == false)
                                {
                                    depNode = firstDepNode;
                                    for( unsigned int j = 0; j < station.numDepEvents; j++)
                                    {
                                        //arrival-departure edge
                                        if( ( ( 1440 - u->time) + depNode->time) >= station.transitTime)
                                        {
                                            travelTime = (1440 + depNode->time) - u->time;

                                            if( v != depNode)
                                            {
                                                std::cout << "\nerror: wrong arr-dep connection"
                                                          << "\narr(" << u->time << ")->dep(" << v->time << ") should be"
                                                          << " arr(" << u->time << ")->dep(" << depNode->time << ")\n";
 
                                                isValid = false;
                                            }

                                            else if( e->weight != travelTime)
                                            {
                                                std::cout << "\nerror: wrong weight at arr-dep connection";
                                                isValid = false;
                                            }

                                            depDetected = true;
                                            break;
                                        }

                                        depNode = G.target(--G.endEdges( depNode));

                                        if( depNode->isArrival())
                                        {
                                            std::cout << "\nerror: invalid descend";
                                            isValid = false;
                                        }
                                    }
                                }

                                if( depDetected == false)
                                {
                                    std::cout << "\nerror: orphan arrNode";
                                    isValid = false;
                                }
                            }

                            else
                            {
                                if( v->time >= ( station.transitTime + u->time))
                                    travelTime = v->time - u->time;
                                else
                                    travelTime = ( 1440 + v->time) - u->time;

                                if( e->weight != travelTime)
                                {
                                    std::cout << "\nerror: wrong weight"
                                              << "\narr(" << u->time << ")->dep(" << v->time << ")["<< e->weight <<"] should be"
                                              << " arr(" << u->time << ")->dep(" << v->time << ")["<< travelTime <<"]\n";

                                    isValid = false;
                                }
                            }
                        }

                        //arrival head
                        else
                        {
                            if( u->stId == v->stId)
                            {
                                std::cout << "\nerror: arr-arr within the station";
                                isValid = false;
                            }

                            if( u->vhId != v->vhId)
                            {
                                std::cout << "\nerror: wrong arr-arr connection";
                                isValid = false;
                            }

                            Time arrTime = (u->time + e->weight) % 1440;

                            if( v->time != arrTime)
                            {
                                std::cout << "\nerror: wrong weight or arr time"
                                          << "\narr(" << u->time << ")->arr(" << v->time << ")["<< e->weight <<"] should be"
                                          << " dep(" << u->time << ")->arr(" << arrTime << ")["<< e->weight <<"]\n";

                                isValid = false;
                            }
                        }
                    }
                }
            }
        }

        if( firstDepNode->time != minDepNode->time)
        {
            std::cout << "\nerror: wrong first dep"
                      << "\nfirst dep is " << minDepNode->time << " and no " << firstDepNode->time << "\n";
            isValid = false;
        }

        if( station.numDepEvents != numDepEvents)
        {
            std::cout << "\nerror: wrong num dep nodes"
                      << "\ndep nodes are " << numDepEvents << " and no " << station.numDepEvents << "\n";
            isValid = false;
        }

        return isValid;
    }

    /**
     * @brief Checks the edge weights.
     **/
    bool checkWeights()
    {
        for(NodeIterator u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; u++)
        {
            for( EdgeIterator e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
            {
                NodeIterator v = G.target( e);
                Distance vtime = (u->time + e->weight) % 1440;
                if( vtime != v->time)
                {
                    std::cout << "\nerror: wrong node times / edge weights!" << std::endl;
                    return false;
                }
            }
        }

        return true;
    }

    /**
     * @brief Checks station info.
     **/
    bool checkStationInfo()
    {
        for( StationID stId = 0; stId < stations.size(); stId++)
        {
            unsigned int counter = 0;
            for( NodeIterator v = G.beginNodes(), lastNode = G.endNodes(); v != lastNode; v++)
            {
                if( v->isDeparture() && v->stId == stId)
                    counter++;
            }

            if( counter != stations[stId].numDepEvents)
            {
                std::cout << "\nerror: numDepEvents" << std::endl;
                return false;
            }

            if( G.getNodeIterator( stations[stId].firstDepNode)->stId != stId)
            {
                std::cout << "\nerror: first Dep event"  << std::endl;
                return false;
            }
        }

        return true;
    }

    StationContainer stations;
    VehicleContainer vehicles;
    GraphType G;

    //stations
    struct Station
    {
        Station(): id(0), name("undefined"), firstDepNode(0), numDepEvents(0), timestamp(0)
        { optArrNodes.reserve(10); tnsArrNodes.reserve(100);}

        //basic info
        union{double x, lon;};
        union{double y, lat;};
        StationID id;
        std::string name;

        NodeDescriptor firstDepNode;
        unsigned int numDepEvents;

        Time transitTime;
        std::vector<StationID> adjStations;
        std::vector<VehicleTypeID> vehicleTypes;

        //shortest path routing
        TTL timestamp;
        bool isSettled;
        Distance minDist;
        std::vector<NodeIterator> optArrNodes;
        std::vector<NodeIterator> tnsArrNodes;

        inline void addAdjacentStation( StationID id)
        {
            if( std::find( adjStations.begin(), adjStations.end(), id) == adjStations.end())
                adjStations.push_back( id);
        }
    };

    //vehicles
    struct Vehicle
    {
        Vehicle(): typeId(0), name("undefined")
        {}
        
        VehicleTypeID typeId;
        std::string name;
        
        inline bool operator==( const Vehicle& veh) const
        {
            if( typeId == veh.typeId)
                return true;
            else
                return false;
        }
    };
};

#endif //TRANSPORT_NETWORK_H
