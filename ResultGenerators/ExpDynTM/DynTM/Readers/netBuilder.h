#ifndef DYNTM_NETBUILDER_H
#define DYNTM_NETBUILDER_H

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

namespace DynTM
{

/**
 * @class DynTMBuilder
 *
 * @brief This class implements a reader for the DynTM graph model. 
 * Its use is for loading data files (GTFS format), containing time expanded transportation network info.
 * @tparam TspNetwork The time expanded data structure used for transporation network representation.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename TspNetwork>
class DynTMBuilder
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
    DynTMBuilder( TspNetwork& tspNetwork, bool realMod = true) :
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
        const SizeType numNodes = stations.size() + numConnections;
        const SizeType numEdges = numConnections * 3;
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
     * @brief Sorts the departure events, at all stations, in ascending order of arrival time.
     **/
    void sortTimeEvents()
    {
        const SizeType numStations = stations.size();
        ProgressStream st_progress( numStations);
        st_progress.label() << "\tSorting the time events for " << numStations << " stations";

        numConnections = 0;

        for( StationID stationId = 0; stationId < numStations; stationId++)
        {
            std::vector<DepEvent>& depEvents = timeEvents[stationId].departures;

            //remove duplicates
            std::sort( depEvents.begin(), depEvents.end(), DepEvent::sortByDep);
            depEvents.erase( std::unique( depEvents.begin(), depEvents.end()), depEvents.end());

            std::sort( depEvents.begin(), depEvents.end(), DepEvent::sortByArr);

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
        std::vector<bool> isErased( numStations, true);
        StationContainer savedStations;
        savedStations.reserve( numStations);
        std::vector<StationID> newIds( numStations, 0);

        //create station graph nodes
        std::vector<StNodeDescriptor> stNode( numStations);
        stG.clear();
        StationID stId, newStId, stopFromId, stopToId;
        for( stId = 0; stId < numStations; stId++)
        {
            stNode[stId] = stG.insertNode();
            stG.getNodeIterator( stNode[stId])->stId = stId;
        }
       
        //create station graph edges
        for( stId = 0; stId < numStations; stId++)
        {
            std::vector<StationID>& adjStations = stations[stId].adjStations;             
            stopFromId = stId;

            for( SizeType i = 0, endSt = adjStations.size(); i<endSt; i++)
            {
                stopToId = adjStations[i];
                if( !stG.hasEdge( stNode[stopFromId], stNode[stopToId]))
                    stG.insertEdge( stNode[stopFromId], stNode[stopToId]);
            }
        }

        //perform the scc pruning
        SccProcessor<StGraph> scp( stG);
        scp.makeStronglyConnected();

        //take the survived scc stations
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
                for( SizeType i = 0, endSt = adjStations.size(); i<endSt; i++)
                {
                    if( isErased[adjStations[i]] == true)
                    {
                        DepContainer& departures = timeEvents[stId].departures;
                        DepContainer savedDepartures;
                        
                        for( SizeType j = 0, dsize = departures.size(); j<dsize; j++)
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

            u->stId = stId;
            stId++;
            numConnections += savedTimeEvents.back().departures.size();
        }

        timeEvents.swap( savedTimeEvents);

        //update connected adjacent destination info
        for( StNodeIterator u = stG.beginNodes(), endNode = stG.endNodes(); u != endNode; ++u)
        {
            Station& depStation = stations[u->stId];
            depStation.adjStations.clear();
            for( StEdgeIterator e = stG.beginEdges(u), endEdge = stG.endEdges(u); e != endEdge; ++e)
                depStation.addAdjacentStation( stG.target(e)->stId);
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
     * @brief Sorts the departure events, at all stations, in ascending order of departure time.
     **/
    void sortByDep()
    {
        const SizeType numStations = stations.size();
        ProgressStream st_progress( numStations);
        st_progress.label() << "\tSorting the time events for " << numStations << " stations";

        for( StationID stationId = 0; stationId < numStations; stationId++)
        {
            std::vector<DepEvent>& depEvents = timeEvents[stationId].departures;

            std::sort( depEvents.begin(), depEvents.end(), DepEvent::sortByDep);

            ++st_progress;
        }
    }

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
    void setSwitchEvent( const NodeDescriptor&vD, const StationID& stationId, const Time& time, const VehicleID& vehicleId)
    {
        const NodeIterator v = G.getNodeIterator( vD);

        v->time = 0;
        v->label = 's';

        v->stId = stationId;
        v->vhId = 0;
    }

    void addWalkPaths()
    {
        const SizeType numStations = stations.size();

        for( StationID uStId = 0; uStId < numStations; uStId++)
        {
            Station& uSt = stations[uStId];
            for( StationID vStId = uStId+1; vStId < numStations; vStId++)
            {
                Station& vSt = stations[vStId];
                double dist = greatCircle( uSt.lat, uSt.lon, vSt.lat, vSt.lon) * 1.609344 / 1000.0;

                if( dist < 1000)
                {
                    double travelTime = ceil( ( dist / 1.5) / 60.0);
                    typename TspNetwork::WalkPath wp;
                    wp.travelTime = travelTime;

                    wp.stId = vStId;
                    uSt.walkPaths.push_back( wp);

                    wp.stId = uStId;
                    vSt.walkPaths.push_back( wp);
                }
                
                /*if( dist < 10000)
                {
                    double travelTime = ceil( ( dist / 5.5) / 60.0);
                    EvPath ep;
                    ep.stId = vStId;
                    ep.travelTime = travelTime;
                    uSt.evPaths.push_back( ep);
                }*/
            }
        }
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
        netEdges.reserve( 2 * ( G.getNumNodes() - numStations) + numConnections);

        //pacts
        //outdeg of depNodes <= 2
        //outdeg of stNodes >= 1
        //depNode:
        //first outgoing edge has head to a switch node
        //second outgoing edge has head to a departure node

        SizeType nodeId = 1;
        StationID stationId = 0;
        DepIter iDep, endDep, jDep, stopDep;

        //insert the switch and the departure nodes, for each station, in the graph
        for( stationId = 0; stationId < numStations; stationId++)
        {
            DepContainer& depEvents = timeEvents[stationId].departures;

            //set the number of departure events of station
            Station& station = stations[stationId];

            //mark the switch node of station
            station.stNode = m_ids[nodeId];
            setSwitchEvent( station.stNode, stationId, 0, 0);
            ++nodeId;

            //mark (unlabeled) neutral nodes as departure events of station
            for( iDep = depEvents.begin(), endDep = depEvents.end(); iDep != endDep; ++iDep)
            {
                iDep->node = m_ids[nodeId];
                setDepEvent( iDep->node, stationId, iDep->time, iDep->vehicleId, iDep->vehicleTypeId);
                ++nodeId;
            }

            ++st_progress;
        }

        //build the inner transfer network of the station
        for( stationId = 0; stationId < numStations; stationId++)
        {
            const Station& station  = stations[stationId];
            DepContainer& depEvents = timeEvents[stationId].departures;

            //insert the switch-dep and dep-switch connections
            for( iDep = depEvents.begin(), endDep = depEvents.end(); iDep != endDep; ++iDep)
            {
                const DepEvent& depEvent = *(iDep);
                const Station& adjStation = stations[depEvent.arrStId];
                travelTime = getTimeDiff( depEvent.arrTime, depEvent.time);
                assert( depEvent.arrTime >= depEvent.time);

                netEdges.push_back( EdgeData( station.stNode, depEvent.node, 0));
                netEdges.push_back( EdgeData( depEvent.node, adjStation.stNode, travelTime));

                DepContainer& adjDepEvents = timeEvents[depEvent.arrStId].departures;
                DepEvent *toAdjDepPt1 = 0, *toAdjDepPt2 = 0;

                for( jDep = adjDepEvents.begin(), stopDep = adjDepEvents.end(); jDep != stopDep; ++jDep)
                {
                        DepEvent& adjDepEvent = *(jDep);

                        //insert edge (from-ArrEvent) -> (to-ArrEvent)
                        if( adjDepEvent.vehicleId == depEvent.vehicleId)
                        {
                            if( adjDepEvent.time >= depEvent.arrTime)
                            {
                                if( toAdjDepPt1 == 0 || toAdjDepPt1->time > adjDepEvent.time)
                                    toAdjDepPt1 = &(adjDepEvent);                           
                            }

                            else if( toAdjDepPt2 == 0 || toAdjDepPt2->time > adjDepEvent.time)
                                toAdjDepPt2 = &(adjDepEvent);
                        }
                }

                if( toAdjDepPt1 != 0)
                {
                    //dep -> st -> dep
                    travelTime = getTimeDiff( depEvent.arrTime, depEvent.time) + getTimeDiff( toAdjDepPt1->time, depEvent.arrTime);
                    assert( ( depEvent.time + travelTime) == toAdjDepPt1->time);
                    netEdges.push_back( EdgeData( depEvent.node, toAdjDepPt1->node, travelTime));
                }

                else if( toAdjDepPt2 != 0)
                {
                    travelTime = getTimeDiff( depEvent.arrTime, depEvent.time) + ((1440 + toAdjDepPt2->time) - depEvent.arrTime);
                    assert( ( ( depEvent.time + travelTime) % 1440) == toAdjDepPt2->time);
                    netEdges.push_back( EdgeData( depEvent.node, toAdjDepPt2->node, travelTime));
                }
            }

            ++st_progress;
        }

        //delete the depEvents info
        for( stationId = 0; stationId < numStations; stationId++)
        {
            DepContainer& depEvents = timeEvents[stationId].departures;

            depEvents.clear();
            DepContainer empty;
            depEvents.swap( empty);
        }

        //delete the time events info
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

        //tspNet.isValid();
    }

    /**
     * @brief Returns the time-distance between two events.
     **/
    Distance getTimeDiff( const Time& time1, const Time& time2)
    {
        if( time1 >= time2)
            return time1 - time2;
        else
        {
            assert( (1440 + time1) >= time2);
            return (1440 + time1) - time2;
        }
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

        //insert the edges
        for( SizeType i=0, size=netEdges.size(); i<size; i++)
        {
            eD = G.insertEdge( netEdges[i].uD, netEdges[i].vD);
            e = G.getEdgeIterator( eD);
            e->weight = netEdges[i].weight;
            ++edge_progress;
        }

        //delete container
        {netEdges.clear(); std::vector<EdgeData> empty; netEdges.swap( empty);}

        postprocess();
    }

    /**
     * @brief Inserts the sorted edges in graph.
     **/
    void insertSortedEdges()
    {
        EdgeDescriptor eD;
        EdgeIterator e, a;

        ProgressStream edge_progress( netEdges.size());
        edge_progress.label() << "\tSorting and inserting " << netEdges.size() << " edges";

        PackedMemoryArray<EdgeList> edgeLists;
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
                //std::cout << "\nwarning: dump edge" << std::endl;
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

        edgeLists.clear();

        //delete container
        {netEdges.clear(); std::vector<EdgeData> empty; netEdges.swap( empty);}

        //G.compress(); //buggy !!TODO
        postprocess(); 
    }

    void postprocess()
    {
        std::cout << "\n\tPostprocessing Phase\n";

        ProgressStream postprocess_progress( G.getNumNodes() + 3 * stations.size());
        postprocess_progress.label() << "\tOrdering " << numConnections << " connections";

        //change the order of outgoing edges so that switch heads are first
        for( NodeIterator v = G.beginNodes(), endNode = G.endNodes(); v != endNode; ++v)
        {
            if( v->isDeparture())
            {
                EdgeIterator edge1 = G.beginEdges( v);
                EdgeIterator edge2 = edge1;
                ++edge2;

                if( edge2 != G.endEdges( v) && G.target( edge1)->isDeparture())
                    G.swapEdges( edge1, edge2);
            }

            ++postprocess_progress;
        }

        setMultiOrdering();
    }


    void setMultiOrdering()
    {
        ProgressStream multiOrdering_progress( 3 * stations.size());
        multiOrdering_progress.label() << "\tOrdering " << numConnections << " departure nodes";

        //group by station and vehicle and then sort by arrival time
        for( StationID stationId = 0, numStations = stations.size(); stationId < numStations; stationId++)
        {
            Station& station = stations[stationId];
            NodeIterator stNode = G.getNodeIterator( station.stNode);
            std::vector<StationID>& adjStations = station.adjStations;    
            std::sort( adjStations.begin(), adjStations.end());

            for( EdgeIterator eNext, e = G.beginEdges( stNode), endEdge = G.endEdges( stNode); e != endEdge;)
            {
                eNext = e; ++eNext;

                NodeIterator depNode = G.target( e);
                EdgeIterator connEdge = G.beginEdges( depNode);
                Distance minArrTime = depNode->time + connEdge->weight;
                StationID minArrStId = G.target( connEdge)->stId;
                VehicleTypeID minVehTpId = depNode->tpId;
                EdgeIterator tg = endEdge;

                EdgeIterator g = eNext;
                for(; g != endEdge; ++g)
                {
                    const NodeIterator nextDepNode = G.target( g);
                    EdgeIterator nextConnEdge = G.beginEdges( nextDepNode);
                    const Distance arrTime = nextDepNode->time + nextConnEdge->weight;
                    const StationID arrStId = G.target( nextConnEdge)->stId;

                    assert( nextDepNode->isDeparture());
                    assert( G.target( nextConnEdge)->isSwitch());
                    const VehicleTypeID vehTpId = nextDepNode->tpId;

                    if( minArrStId > arrStId)
                    {
                        tg = g;
                        minArrStId = arrStId;
                        minArrTime = arrTime;
                        minVehTpId = vehTpId;
                    }

                    else if( minArrStId == arrStId)
                    {
                        if( minVehTpId > vehTpId)
                        {
                            tg = g;
                            minVehTpId = vehTpId;
                            minArrTime = arrTime;
                        }
                        else if( minVehTpId == vehTpId)
                        {
                            if( minArrTime > arrTime)
                            {
                                tg = g;
                                minArrTime = arrTime;
                            }
                        }
                    }
                }
                
                if( tg != endEdge)
                    G.swapEdges( e, tg);

                e = eNext;
            }

            ++multiOrdering_progress;
        }
    
        //create each (first edge, last edge) pair per adjacent station and vehicle type
        for( StationID stationId = 0, numStations = stations.size(); stationId < numStations; stationId++)
        {
            Station& station = stations[stationId];
            const NodeIterator stNode = G.getNodeIterator( station.stNode);
            const std::vector<StationID>& adjStations = station.adjStations; 
            std::vector< std::vector<EdgeIterator> >& stVehEdges = station.stVehEdges;
            std::vector<EdgeIterator>& stEdges = station.stEdges;  
  
            stVehEdges.resize( adjStations.size());
            unsigned int i=0;

            EdgeIterator e = G.beginEdges( stNode), endEdge = G.endEdges( stNode);

            NodeIterator depNode = G.target( e);
            EdgeIterator connEdge = G.beginEdges( depNode);
            VehicleTypeID nextVehType, currVehType = depNode->tpId;
            StationID nextID, currID = G.target( connEdge)->stId;

            stVehEdges[i].push_back( e);
            stEdges.push_back( e);
            ++e;

            for(; e != endEdge; ++e)
            {
                depNode = G.target( e);
                connEdge = G.beginEdges( depNode);
                nextVehType = depNode->tpId;
                nextID = G.target( connEdge)->stId;

                if( currID != nextID)
                {
                    stVehEdges[i].push_back( e);
                    ++i;
                    currID = nextID;
                    currVehType = nextVehType;
                    stVehEdges[i].push_back( e);
                    stEdges.push_back( e);
                }

                else if( currVehType != nextVehType)
                {
                    stVehEdges[i].push_back( e);
                    currVehType = nextVehType;
                }
            }

            stVehEdges[i].push_back( endEdge);
            stEdges.push_back( endEdge);

            ++multiOrdering_progress;
        }

        //set next pointers
        /*for( StationID stationId = 0, numStations = stations.size(); stationId < numStations; stationId++)
        {
            Station& station = stations[stationId];
            NodeIterator stNode = G.getNodeIterator( station.stNode);
            std::vector<StationID>& adjStations = station.adjStations;
            const std::vector<EdgeIterator>& stEdges = station.stEdges;    

            //add used vehicles
            //station.addUsedVehicles( G);
            station.vehFirstDep.resize( adjStations.size());

            for( unsigned int i=0; i<adjStations.size(); i++)
            {
                std::map<VehicleTypeID, VehicleTypeID> vehIndex;

                //used vehicle types to adj station
                std::vector<VehicleTypeID>& usedVehicles = station.usedVehicles[i];
                for( unsigned int j=0; j<usedVehicles.size(); j++)
                    vehIndex[usedVehicles[j]] = j;

                //dep set per vehicle type
                std::vector< std::vector<NodeIterator> > vehDeps;
                vehDeps.resize( usedVehicles.size());

                //deps to adj station
                for( EdgeIterator e = stEdges[i], endEdge = stEdges[i+1], eNext; e != endEdge; ++e)
                {
                    NodeIterator dep = G.target( e);
                    vehDeps[vehIndex[dep->tpId]].push_back( dep);
                }

                station.vehFirstDep[i].resize( usedVehicles.size());

                //set the pointers from dep to dep
                for( unsigned int j=0; j<usedVehicles.size(); j++)
                {
                    const std::vector<NodeIterator>& deps = vehDeps[j];
                    const int numDeps = deps.size()-1;
                    for( int r=0; r<numDeps; r++)
                        deps[r]->next = G.getNodeDescriptor( deps[r+1]);
 
                    deps.back()->next = G.getNodeDescriptor( deps.front());
                    station.vehFirstDep[i][j] = G.getNodeDescriptor( deps.front()); 
                }
            }

            ++multiOrdering_progress;
        }*/


        //create adj veh dep index
        for( StationID stationId = 0, numStations = stations.size(); stationId < numStations; stationId++)
        {
            Station& station = stations[stationId];
            station.addUsedVehicles( G);
            station.createDepIndex( G);

            ++multiOrdering_progress;
        }
    }

    void setRegOrdering()
    {
        ProgressStream regOrdering_progress( 3 * stations.size());
        regOrdering_progress.label() << "\tOrdering " << numConnections << " departure nodes";

        //group dep nodes by station id and then sort them by ascending arrival time
        for( StationID stationId = 0, numStations = stations.size(); stationId < numStations; stationId++)
        {
            Station& station = stations[stationId];
            NodeIterator stNode = G.getNodeIterator( station.stNode);
            std::vector<StationID>& adjStations = station.adjStations;    
            std::sort( adjStations.begin(), adjStations.end());

            for( EdgeIterator eNext, e = G.beginEdges( stNode), endEdge = G.endEdges( stNode); e != endEdge;)
            {
                eNext = e; ++eNext;

                NodeIterator depNode = G.target( e);
                EdgeIterator connEdge = G.beginEdges( depNode);
                Distance minArrTime = depNode->time + connEdge->weight;
                StationID minArrStId = G.target( connEdge)->stId;
                EdgeIterator tg = endEdge;

                assert( depNode->isDeparture());
                assert( G.target( connEdge)->isSwitch());

                EdgeIterator g = eNext;
                for(; g != endEdge; ++g)
                {
                    const NodeIterator nextDepNode = G.target( g);
                    EdgeIterator nextConnEdge = G.beginEdges( nextDepNode);
                    const Distance arrTime = nextDepNode->time + nextConnEdge->weight;
                    const StationID arrStId = G.target( nextConnEdge)->stId;

                    assert( nextDepNode->isDeparture());
                    assert( G.target( nextConnEdge)->isSwitch());

                    if( minArrStId > arrStId || ( minArrStId == arrStId && minArrTime > arrTime))
                    {
                        tg = g;
                        minArrStId = arrStId;
                        minArrTime = arrTime;
                    }
                }
                
                if( tg != endEdge)
                    G.swapEdges( e, tg);

                e = eNext;
            }

            ++regOrdering_progress;
        }

        //create each (first edge, last edge) pair to an adjacent station
        for( StationID stationId = 0, numStations = stations.size(); stationId < numStations; stationId++)
        {
            Station& station = stations[stationId];
            const NodeIterator stNode = G.getNodeIterator( station.stNode);
            const std::vector<StationID>& adjStations = station.adjStations;    

            station.usedVehicles.resize( adjStations.size());
            station.stVehEdges.resize( adjStations.size());
            for( unsigned int i=0; i<adjStations.size();i++)
                station.usedVehicles[i].push_back( 0);

            std::vector< std::vector<EdgeIterator> >& vehEdges = station.stVehEdges;    

            EdgeIterator e = G.beginEdges( stNode), endEdge = G.endEdges( stNode);

            NodeIterator depNode = G.target( e);
            EdgeIterator connEdge = G.beginEdges( depNode);
            StationID nextID, currID = G.target( connEdge)->stId;
            assert( G.target( connEdge)->isSwitch());
            unsigned int j=0;

            Distance arrTime = depNode->time + connEdge->weight;

            vehEdges[j].push_back( e);
            ++e;

            for(; e != endEdge; ++e)
            {
                depNode = G.target( e);
                connEdge = G.beginEdges( depNode);
                nextID = G.target( connEdge)->stId;
                assert( G.target( connEdge)->isSwitch());
                Distance nextArrTime = depNode->time + connEdge->weight;

                if( currID != nextID)
                {
                    vehEdges[j].push_back( e);
                    currID = nextID;
                    j++;
                    vehEdges[j].push_back( e);
                }
                else
                    assert( arrTime <= nextArrTime);

                arrTime = nextArrTime;
            }

            vehEdges[j].push_back( endEdge);

            ++regOrdering_progress;
        }

        for( NodeIterator u = G.beginNodes(), end = G.endNodes(); u != end; ++u)
            u->tpId = 0;

        //create dep index
        for( StationID stationId = 0, numStations = stations.size(); stationId < numStations; stationId++)
        {
            Station& station = stations[stationId];
            station.createDepIndex( G);

            ++regOrdering_progress;
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

    //dep events
    struct DepEvent
    {
        NodeDescriptor node;
        VehicleID vehicleId;
        VehicleTypeID vehicleTypeId;

        Time time;
        Time arrTime;

        StationID arrStId;

        inline bool operator <( const DepEvent& ev) const
        {
            if( arrTime < ev.arrTime)
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

        static bool sortByArr(const DepEvent &ldv, const DepEvent &rdv) { return ldv.arrTime < rdv.arrTime; }
        static bool sortByDep(const DepEvent &ldv, const DepEvent &rdv) { return ldv.time < rdv.time; }
    };

    typedef typename std::vector<DepEvent> DepContainer;
    typedef typename DepContainer::iterator DepIter;

	//time events
	struct TimeEvents
	{
          DepContainer departures;
	};

};

};

#endif
