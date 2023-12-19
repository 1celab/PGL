#ifndef REDTE_NETBUILDER_H
#define REDTE_NETBUILDER_H

#include <typeinfo>
#include <cassert>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <map>

#include <Utilities/progressBar.h>
#include "../Structs/definitions.h"
#include "../Algs/scc.h"

namespace RedTE
{

/**
 * @class RedTEBuilder
 *
 * @brief This class implements a reader for the RedTE graph model. 
 * Its use is for loading data files (GTFS format), containing time expanded transportation network info.
 * @tparam TspNetwork The time expanded data structure used for transporation network representation.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename TspNetwork>
class RedTEBuilder
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
     * @brief Constructor. Creates an instance of DynTMBuilder.
     * @param tspNetwork The time expanded network.
     * @param stopFilename The file with the stops info.
     * @param connectionFilename The file with the connections info.
     * @param realMod The realistic flag.
     **/
    RedTEBuilder( TspNetwork& tspNetwork, bool realMod = true) :
    tspNet(tspNetwork), stations(tspNet.stations), vehicles(tspNet.vehicles), G(tspNet.G),
    isRealistic(realMod)
    {}

 protected:

    TspNetwork& tspNet;
    StationContainer& stations;
    VehicleContainer& vehicles;
    GraphType& G;

    unsigned int numConnections;
    bool isRealistic;

    //node ids
    std::vector<NodeDescriptor> m_ids;

    //edge containers (due to special structure)
    struct EdgeData;
    std::vector<EdgeData> netEdges;

    //time event container
    struct TimeEvents;
    std::vector<TimeEvents> timeEvents;

    /**
     * @brief Extracts the node events of network.
     **/
    void createNodeEvents()
    {
        //estimate the overall space for the time-expanded graph
        const SizeType numNodes = numConnections * 2;
        const SizeType numEdges = numConnections * 4;
        G.reserve( numNodes, numEdges);

        //create the nodes
        for( SizeType i = 0; i < numNodes; ++i)
            G.insertNode();

        //node ids:1,2,...
        m_ids.clear();
        m_ids.reserve( numNodes + 1);
        m_ids.push_back( 0);
            
        //create the ids
        for( NodeIterator u = G.beginNodes(), end = G.endNodes(); u != end; ++u)
            m_ids.push_back( G.getNodeDescriptor( u));
    }

    /**
     * @brief Sorts the departure and arrival events, at all stations, in ascending order of time.
     **/
    void sortTimeEvents()
    {
        const SizeType numStations = stations.size();
        ProgressStream st_progress( numStations);
        DepIter iDep, endDep;

        numConnections = 0;

        st_progress.label() << "\tSorting the time events for " << numStations << " stations";

        for( StationID stationId = 0; stationId < numStations; stationId++)
        {
            std::vector<DepEvent>& depEvents = timeEvents[stationId].departures;
            std::list<ArrEvent>& arrEvents = timeEvents[stationId].arrivals;

            std::sort( depEvents.begin(), depEvents.end());
            arrEvents.sort();

            //remove duplicates
            if( depEvents.size() != 0)
                for( iDep = depEvents.begin(), endDep = depEvents.end()-1; iDep != endDep; ++iDep)
                {
                    const DepEvent& currDepEvent = *(iDep);
                    const DepEvent& nextDepEvent = *(iDep+1);

                    if( currDepEvent == nextDepEvent)
                        timeEvents[nextDepEvent.arrStId].arrivals.erase( nextDepEvent.arrIt);
                }

            depEvents.erase( std::unique( depEvents.begin(), depEvents.end()), depEvents.end());

            numConnections += depEvents.size();

            ++st_progress;
        }
    }

    /**
     * @brief Extracts the largest strongly connected component of network.
     **/
    void computeSCC()
    {
        //station ids
        SizeType numStations = stations.size();
        StationContainer savedStations;
        savedStations.reserve( numStations);

        std::vector<bool> isErased( numStations, true);
        std::vector<StationID> newIds( numStations, 0);
        std::vector<StNodeDescriptor> stNode( numStations);

        StationID stId, newStId, stopFromId, stopToId;

        //create station graph nodes
        stG.clear();
        for( stId = 0; stId < numStations; stId++)
        {
            stNode[stId] = stG.insertNode();
            stG.getNodeIterator( stNode[stId])->stId = stId;
        }

        //create station graph edges
        for( stopFromId = 0; stopFromId < numStations; stopFromId++)
        {
            std::vector<StationID>& adjStations = stations[stopFromId].adjStations;

            for( SizeType i = 0, end = adjStations.size(); i<end; i++)
            {
                stopToId = adjStations[i];
                stG.insertEdge( stNode[stopFromId], stNode[stopToId]);
            }
        }

        //perform the scc pruning
        SccProcessor<StGraph> scp( stG);
        scp.makeStronglyConnected();

        //take the scc stations
        for( StNodeIterator u = stG.beginNodes(), endNode = stG.endNodes(); u != endNode; ++u)
        {
            isErased[u->stId] = false;
            savedStations.push_back( stations[u->stId]);
        }

        //set the new ids
        for( newStId = 0, stId = 0; stId < numStations; stId++)
            if( isErased[stId] == false)
            {
                newIds[stId] = newStId;
                newStId++;
            }

        //remove unattached time events
        for( stId = 0; stId < numStations; stId++)
            if( isErased[stId] == false)
            {
                bool isUpdated = false;
                std::vector<StationID>& adjStations = stations[stId].adjStations;
                for( SizeType i = 0, end = adjStations.size(); i<end; i++)
                {
                    if( isErased[adjStations[i]] == true)
                    {
                        DepContainer& departures = timeEvents[stId].departures;
                        DepContainer savedDepartures;
                        
                        for( SizeType j = 0, size = departures.size(); j<size; j++)
                            if( isErased[departures[j].arrStId] == false)
                            {
                                departures[j].arrStId = newIds[departures[j].arrStId];
                                savedDepartures.push_back( departures[j]);
                            }

                        departures.swap( savedDepartures);
                        isUpdated = true;
                        break;
                    }
                }

                if( isUpdated == false)
                {
                    DepContainer& departures = timeEvents[stId].departures;
                    
                    for( SizeType j = 0, size = departures.size(); j<size; j++)
                        departures[j].arrStId = newIds[departures[j].arrStId];
                }
            }

            else
            {
                DepContainer& departures = timeEvents[stId].departures;
  
                for( SizeType j = 0, size = departures.size(); j<size; j++)
                    if( isErased[departures[j].arrStId] == false)
                        timeEvents[departures[j].arrStId].arrivals.erase(departures[j].arrIt);
            }

        stations.swap( savedStations);
        numStations = stations.size();

        //update time events info
        std::vector<TimeEvents> savedTimeEvents;
        savedTimeEvents.reserve( numStations);
        stId = 0;
        numConnections = 0;
        for( StNodeIterator u = stG.beginNodes(), endNode = stG.endNodes(); u != endNode; ++u)
        {
            savedTimeEvents.resize( savedTimeEvents.size()+1);
            savedTimeEvents.back().departures.swap( timeEvents[u->stId].departures);
            savedTimeEvents.back().arrivals.swap( timeEvents[u->stId].arrivals);
            u->stId = stId;
            stId++;
            numConnections += savedTimeEvents.back().arrivals.size();
        }

        timeEvents.swap( savedTimeEvents);

        //update connected adjacent destination info
        for( StNodeIterator u = stG.beginNodes(), endNode = stG.endNodes(); u != endNode; ++u)
        {
            Station& depStation = stations[u->stId];
            depStation.adjStations.clear();
            for( StEdgeIterator e = stG.beginEdges(u), endEdge = stG.endEdges(u); e != endEdge; ++e)
                depStation.addAdjacentStation( stG.target( e)->stId);
        }

        std::cout << "\n\tstations:" << stations.size() << " cnxEdges:" << numConnections
                  << "\n\tdepNodes:" << numConnections << " arrNodes:" << numConnections
                  << std::endl;
    }

	//graph station Node
	struct StNodeInfo : DefaultGraphItem
	{
        StNodeInfo() : compId(0), isMarked(0)
        {}

        StationID stId;
        unsigned int compId;
        bool isMarked;
	};

	//graph station Edge
	struct StEdgeInfo : DefaultGraphItem
	{};

	//graph station InEdge
	struct StInEdgeInfo : DefaultGraphItem
	{};

	//typedef DynamicGraph< PackedMemoryArrayImpl, StNodeInfo, StEdgeInfo, StInEdgeInfo>   StGraph;
	//typedef DynamicGraph< ForwardStarImpl, StNodeInfo, StEdgeInfo, StInEdgeInfo>         StGraph;
	typedef DynamicGraph< AdjacencyListImpl, StNodeInfo, StEdgeInfo, StInEdgeInfo>         StGraph;
    typedef typename StGraph::NodeIterator                     StNodeIterator;
    typedef typename StGraph::NodeDescriptor                   StNodeDescriptor;
    typedef typename StGraph::EdgeIterator                     StEdgeIterator;
    typedef typename StGraph::EdgeDescriptor                   StEdgeDescriptor;
    StGraph stG;


    /**
     * @brief Adds a departure time event in a station, where a vehicle begins its route.
     * @param vD The node (depNode) which represents the departure time event.
     * @param stationId The station.
     * @param time The time that the departure event occurs.
     * @param vehicleId The vehicle that departs from the station.
     **/
    void setDepEvent( const NodeDescriptor& vD, const StationID& stationId, const Time& time, const VehicleID& vehicleId, const VehicleTypeID& vehicleTypeId)
    {
        const NodeIterator v = G.getNodeIterator( vD);

        v->time = time;
        v->label = 'd';

        v->stId = stationId;
        v->vhId = vehicleId;
        v->tpId = vehicleTypeId;
    }

    /**
     * @brief Adds an arrival time event in a station, where a vehicle ends its route.
     * @param vD The node (arrNode) which represents the arrival time event.
     * @param stationId The station.
     * @param time The time that the arrival event occurs.
     * @param vehicleId The vehicle that arrives to the station.
     **/
    void setArrEvent( const NodeDescriptor&vD, const StationID& stationId, const Time& time, const VehicleID& vehicleId, const VehicleTypeID& vehicleTypeId)
    {
        const NodeIterator v = G.getNodeIterator( vD);

        v->time = time;
        v->label = 'a';

        v->stId = stationId;
        v->vhId = vehicleId;
        v->tpId = vehicleTypeId;
    }

    /**
     * @brief Builds the transport network.
     **/
    void buildNetwork()
    {
        Distance travelTime;

        //sort time events
        sortTimeEvents();

        //prune graph
        computeSCC();

        //create node events
        createNodeEvents();

        const SizeType numStations = stations.size();
        ProgressStream st_progress( 2 * numStations);
        st_progress.label() << "\tBuilding local transfer network for " << numStations << " stations";

        //create edge containers
        netEdges.reserve( G.getNumNodes() * 3);

        //pacts
        //outdeg of depNodes <= 2
        //outdeg of arrNodes >= 1
        //depNode:
        //first outgoing edge has head to an arrival node
        //second outgoing edge has head to a departure node

        SizeType nodeId = 1;
        StationID stationId = 0;
        DepIter iDep, endDep;
        ArrIter iArr, endArr;

        //insert the departure and the arrival nodes, for each station, in the graph
        for( stationId = 0; stationId < numStations; stationId++)
        {
            DepContainer& depEvents = timeEvents[stationId].departures;
            ArrContainer& arrEvents = timeEvents[stationId].arrivals;

            //set the number of departure events of station
            Station& station = stations[stationId];
            station.numDepEvents = depEvents.size();

            //mark the departure node-events of station
            if( depEvents.size() != 0)
            {
                //set the first (earliest) departure event of station
                station.firstDepNode = m_ids[nodeId];
      
                std::map<VehicleTypeID,bool> vehTypeIndex;

                //mark (unlabeled) neutral nodes as departure events of station
                for( iDep = depEvents.begin(), endDep = depEvents.end(); iDep != endDep; ++iDep)
                {
                    iDep->node = m_ids[nodeId];
                    setDepEvent( iDep->node, stationId, iDep->time, iDep->vehicleId, iDep->vehicleTypeId);
                    ++nodeId;

                    vehTypeIndex[iDep->vehicleTypeId] = true;
                }

                //set station vehicle types
                std::map<VehicleTypeID,bool>::iterator it;
                for( it = vehTypeIndex.begin(); it != vehTypeIndex.end(); ++it)
                    station.vehicleTypes.push_back( it->first);

                std::sort( station.vehicleTypes.begin(), station.vehicleTypes.end());
            }

            else
                station.firstDepNode = G.nilNodeDescriptor();

            //mark (unlabeled) neutral nodes as arrival events of station
            for( iArr = arrEvents.begin(), endArr = arrEvents.end(); iArr != endArr; ++iArr)
            {
                iArr->node = m_ids[nodeId];
                setArrEvent( iArr->node, stationId, iArr->time, iArr->vehicleId, iArr->vehicleTypeId);
                ++nodeId;
            }

            ++st_progress;
        }

        //build the inner transfer network of the station
        for( stationId = 0; stationId < numStations; stationId++)
        {
            ++st_progress;
            Station& station = stations[stationId];

            DepContainer& depEvents = timeEvents[stationId].departures;
            ArrContainer& arrEvents = timeEvents[stationId].arrivals;        

            const SizeType numDepEvents = depEvents.size();

            //special case station with no dep node
            if( numDepEvents == 0)
                continue;

            //insert the travel-connection edges between the station and its neigbor-stations
            for( iDep = depEvents.begin(), endDep = depEvents.end(); iDep != endDep; ++iDep)
            {
                const DepEvent& depEvent = *(iDep);
                const ArrEvent& toArrEvent = *(iDep->arrIt);

                travelTime = getTimeDiff( toArrEvent.arrTime, depEvent.time);

                netEdges.push_back( EdgeData( depEvent.node, toArrEvent.node, travelTime));
            }

            //insert the stay and transfer edges between the time events in the station
            for( iArr = arrEvents.begin(), endArr = arrEvents.end(); iArr != endArr; ++iArr)
            {
                const ArrEvent& arrEvent = *(iArr);
                DepEvent const * toDepEvPt = 0;

                //A) staying in the same vehicle - route without transit 
                if( numDepEvents > 1)
                {
                    //insert edge routes without transit within the day
                    for( iDep = depEvents.begin(), endDep = depEvents.end(); iDep != endDep; ++iDep)
                    {
                        const DepEvent& depEvent = *(iDep);

                        if( arrEvent.vehicleId == depEvent.vehicleId)
                        {
                            if( arrEvent.time <= depEvent.time)
                            {
                                toDepEvPt = &(depEvent);
                                break;
                            }

                            else if( toDepEvPt == 0)
                                toDepEvPt = &(depEvent);
                        }
                    }

                    //insert edge (from-ArrEvent) -> (to-ArrEvent)
                    if( toDepEvPt != 0)
                    {
                        travelTime = getTimeDiff( toDepEvPt->time, arrEvent.time) + (toDepEvPt->arrIt)->arrTime - toDepEvPt->time;
                        netEdges.push_back( EdgeData( arrEvent.node, (toDepEvPt->arrIt)->node, travelTime));
                    }
                }

                //B) changing vehicle - route with transits

                //special case station with one dep node
                else if( numDepEvents == 1)
                {
                    const DepEvent& depEvent = *(depEvents.begin());

                    if( arrEvent.vehicleId == depEvent.vehicleId)
                    {
                        station.transitTime = 0;
                        //insert edge (ArrEvent) -> (DepEvent)
                        travelTime = getTimeDiff( depEvent.time, arrEvent.time);
                        netEdges.push_back( EdgeData( arrEvent.node, depEvent.node, travelTime));
                        continue;
                    }
                }


                else
                    continue;


                bool depConnected = false;

                for( iDep = depEvents.begin(), endDep = depEvents.end(); iDep != endDep; ++iDep)
                {
                    const DepEvent& depEvent = *(iDep);

                    //insert edge (ArrEvent) -> (DepEvent)
                    if( depEvent.time >= ( station.transitTime + arrEvent.time))
                    {
                        //compute the time a passenger needs to reach the next earliest departure
                        //diff >= (station's transit time for changing train)
                        travelTime = getTimeDiff( depEvent.time, arrEvent.time);
                        netEdges.push_back( EdgeData( arrEvent.node, depEvent.node, travelTime));
                        depConnected = true;
                        break;
                    }
                }

                //special case: if there is no departure event after the arrival event until the midnight
                if( depConnected == false)
                {
                    for( iDep = depEvents.begin(), endDep = depEvents.end(); iDep != endDep; ++iDep)
                    {
                        const DepEvent& depEvent = *(iDep);

                        //insert edge (ArrEvent) -> (DepEvent)
                        if( ( ( 1440 + depEvent.time) - arrEvent.time ) >= station.transitTime)
                        {
                            travelTime = (1440 + depEvent.time) - arrEvent.time;
                            netEdges.push_back( EdgeData( arrEvent.node, depEvent.node, travelTime));
                            break;
                        }
                    }
                }
            }

            //C) stay edges, between the departure events
            for( iDep = depEvents.begin(), endDep = depEvents.end()-1; iDep != endDep; ++iDep)
            {
                const DepEvent& prevDepEvent = *(iDep);
                const DepEvent& currDepEvent = *(iDep+1);

                travelTime = currDepEvent.time - prevDepEvent.time;

                //add a stay edge between each pair of successive departure nodes
                netEdges.push_back( EdgeData( prevDepEvent.node, currDepEvent.node, travelTime));
            }

            if( numDepEvents > 1)
            {
                //get the first and last departure event-nodes
                const DepEvent& firstDepEvent = depEvents.front();
                const DepEvent& lastDepEvent = depEvents.back();

                travelTime = ( 1440 + firstDepEvent.time) - lastDepEvent.time;

                //complete the cycle betweeen the departure nodes
                //connect the last departure node with the the first departure node,
                //in order to enable transit over the time period
                netEdges.push_back( EdgeData( lastDepEvent.node, firstDepEvent.node, travelTime));
            }

            //clear the depEvents
            depEvents.clear();
            DepContainer empty;
            depEvents.swap( empty);
        }

        //clear the timeEvents
        std::vector<TimeEvents> empty;
        timeEvents.swap( empty);

        //insert the edges
        const std::string pmg("PackedMemoryArrayImpl");
        const std::string graphType( typeid(G).name());

        //PMG
        if( graphType.find(pmg) != std::string::npos)
            insertSortedEdges();

        //ADJ, dynFS
        else
            insertEdges();
    }

    /**
     * @brief Returns the time-distance between two events.
     **/
    Distance getTimeDiff( const Time& time1, const Time& time2)
    {
        if( time1 >= time2)
            return time1 - time2;
        else
            return (1440 + time1) - time2;
    }

    /**
     * @brief Inserts the edges in graph.
     **/
    void insertEdges()
    {
        EdgeDescriptor eD;
        EdgeIterator e;

        ProgressStream edge_progress( netEdges.size());
        edge_progress.label() << "\tInserting " << netEdges.size() << " edges";

        //insert the first edges
        for( SizeType i=0, size=netEdges.size(); i<size; i++)
        {
            eD = G.insertEdge( netEdges[i].uD, netEdges[i].vD);
            e = G.getEdgeIterator( eD);
            e->weight = netEdges[i].weight;
            ++edge_progress;
        }

        //delete container
        {netEdges.clear(); std::vector<EdgeData> empty; netEdges.swap( empty);}

        //insert the rest edges
        for( SizeType i=0, size=netEdges.size(); i<size; i++)
        {
            eD = G.insertEdge( netEdges[i].uD, netEdges[i].vD);
            e = G.getEdgeIterator( eD);
            e->weight = netEdges[i].weight;
            ++edge_progress;
        }

        //delete container
        {netEdges.clear(); std::vector<EdgeData> empty; netEdges.swap( empty);}
    }

    /**
     * @brief Inserts the sorted edges in graph.
     **/
    void insertSortedEdges()
    {
        EdgeDescriptor eD;
        EdgeIterator e;

        ProgressStream edge_progress( netEdges.size());
        edge_progress.label() << "\tSorting and inserting " << netEdges.size() << " edges";

        PackedMemoryArray<EdgeList> edgeLists;
        edgeLists.clear();
        typename PackedMemoryArray<EdgeList>::Iterator it;
        edgeLists.reserve( netEdges.size());

        //sort the first edges
        for( SizeType i=0, size=netEdges.size(); i<size; i++)
            edgeLists.optimalInsert( EdgeList( &(netEdges[i])));

        //insert the first edges
        while( !edgeLists.empty())
        {
            it = edgeLists.chooseCell();

            if( it->eData == 0)  
            {
                //warning: there are duplicates
                std::cout << "\nwarning: zero edge" << std::endl;
                continue;
            }
            
            const EdgeData& edge = *(it->eData);

            if( G.hasEdge( edge.uD, edge.vD) == true)
            {
                //warning: there are duplicates
                std::cout << "\nwarning: there are duplicates" << std::endl;
                continue;
            }

            eD = G.insertEdge( edge.uD, edge.vD);
            e = G.getEdgeIterator( eD);
            e->weight = edge.weight;

            edgeLists.erase( it);
            ++edge_progress;
        }

        //delete container
        {netEdges.clear(); std::vector<EdgeData> empty; netEdges.swap( empty);}

        //TODO needs clear because it is not "empty"...!
        edgeLists.clear();

        G.compress();
        correctOrdering();
    }


   void correctOrdering()
   {
        //reorder edges (arrival head is first)
        for( NodeIterator v = G.beginNodes(), endNode = G.endNodes(); v != endNode; ++v)
        {
            if( v->isDeparture())
            {
                assert( G.outdeg( v) <= 2);
                EdgeIterator edge1 = G.beginEdges( v);
                EdgeIterator edge2 = edge1;
                ++edge2;

                if( edge2 != G.endEdges( v) && G.target( edge1)->isDeparture())
                    G.swapEdges( edge1, edge2);
            }
        }
    }


    /**
     * @brief Returns the node that corresponds to id.
     * @param id The (unique) id of a node.
     * @return A node.
     **/
    NodeDescriptor id2Desc( SizeType id)
    {
        return m_ids[id];
    }

    //edge data list
    struct EdgeList
    {
        EdgeList( unsigned int init = 0): eData(0)
        {}

        EdgeList( EdgeData* data): eData(data)
        {}

        bool operator== ( const EdgeList& other)
        {
            return ( eData == other.eData);
        }

        bool operator!= ( const EdgeList& other)
        {
            return ( eData != other.eData);
        }

        EdgeData* eData;
    };

    //edge data
    struct EdgeData
    {
        EdgeData( NodeDescriptor uD, NodeDescriptor vD, Distance weight)
        {
            this->uD = uD;
            this->vD = vD;
            this->weight = weight;
        }

        NodeDescriptor uD;
        NodeDescriptor vD;
        Distance weight;
    };

    //arr events
    struct ArrEvent
    {
        NodeDescriptor node;
        VehicleID vehicleId;
        VehicleTypeID vehicleTypeId;

        Time time;
        Time arrTime;

        inline bool operator <( const ArrEvent& ev) const
        {
            if( time < ev.time)
                return true;
            else
                return false;
        }

        inline bool operator ==( const ArrEvent& ev) const
        {
            if( vehicleId == ev.vehicleId && time == ev.time)
                return true;
            else
                return false;
        }
    };

    typedef typename std::list<ArrEvent> ArrContainer;
    typedef typename ArrContainer::iterator ArrIter;

    //dep events
    struct DepEvent
    {
        NodeDescriptor node;
        VehicleID vehicleId;
        VehicleTypeID vehicleTypeId;

        Time time;

        StationID arrStId;

        typename ArrContainer::iterator arrIt;
    
        inline bool operator <( const DepEvent& ev) const
        {
            if( time < ev.time)
                return true;
            else
                return false;
        }

        inline bool operator ==( const DepEvent& ev) const
        {
            if( vehicleId == ev.vehicleId && time == ev.time)
                return true;
            else
                return false;
        }

        static std::vector<TimeEvents> * tevs;
    };

    typedef typename std::vector<DepEvent> DepContainer;
    typedef typename DepContainer::iterator DepIter;

	//time events
	struct TimeEvents
	{
        DepContainer departures;
        ArrContainer arrivals;
	};

};

};

#endif
