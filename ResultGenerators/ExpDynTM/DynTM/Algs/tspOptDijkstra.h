#ifndef DYNTM_OPT_DIJKSTRA
#define DYNTM_OPT_DIJKSTRA

#include <Structs/Trees/priorityQueue.h>
#include <climits>

#include "../Structs/definitions.h"

namespace DynTM
{

/**
 * @class TspOptDijkstra
 *
 * @brief The optimized Dijkstra-based algorithm on Dynamic Timetable graph (time-expanded approach).
 *
 * This class supports finding efficiently the shortest path and the shortest travel time from a source station s to a target station t.
 *
 * @tparam TspNetwork The time expanded data structure used for the representation of transporation network.
 * @author Andreas Paraskevopoulos
 *
 */
template<typename TspNetwork>
class TspOptDijkstra
{

 public:

    typedef typename TspNetwork::GraphImpl                              GraphType;
    typedef typename TspNetwork::SizeType                               SizeType;
    typedef typename TspNetwork::NodeDescriptor                         NodeDescriptor;
    typedef typename TspNetwork::NodeIterator                           NodeIterator;
    typedef typename TspNetwork::EdgeDescriptor                         EdgeDescriptor;
    typedef typename TspNetwork::EdgeIterator                           EdgeIterator;
    typedef typename TspNetwork::InEdgeIterator                         InEdgeIterator;
    typedef typename TspNetwork::StationContainer                       StationContainer;
    typedef typename TspNetwork::Station                                Station;
    typedef StaticPriorityQueue<Distance, NodeIterator, HeapStorage>    PriorityQueueType;

    /**
     * @brief Constructor. Creates an instance of TspOptDijkstra.
     * @param graph The graph to run the algorithm on.
     * @param stationContainer A container with the stations.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order 
     * to check whether a node is visited or not.
     */
    TspOptDijkstra( GraphType& graph, StationContainer& Stations, TTL *timestamp) 
    : G(graph), stations(Stations), m_timestamp(timestamp)
    {
        pq.reserve( G.getNumNodes());
    }
  
    /**
     * @brief Returns the number of the nodes that have have been settled (enqueued and dequeued) by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int& getSettledNodes()
    {
        return settledNodes;
    }

    /**
     * @brief Returns the number of the nodes that have have been explored (enqueued) by the algorithm.
     * @return The number of the explored nodes.
     */
    const unsigned int& getTouchedNodes()
    {
        return touchedNodes;
    }

    /**
     * @brief Returns the number of the edges that have have been traversed by the algorithm.
     * @return The number of the explored edges.
     */
    const unsigned int& getTouchedEdges()
    {
        return touchedEdges;
    }

    /**
     * @brief Runs a shortest path query from a souce station at a start time to a target station.
     * @param depStId Id of source station.
     * @param arrStId Id of target station.
     * @param tstart Start time at source station.
     */
    bool runQuery( const StationID depStId, const StationID arrStId, const Time tstart)
    {
        NodeIterator u, v, depNode, nextDepNode;
        EdgeIterator e, connEdge, tsfEdge, endEdge, lastEdge;

        init( depStId, arrStId, tstart);

        while( !pq.empty())
        {
            //switch node
            u = pq.minItem();
            pq.popMin();
            settledNodes++;

            //set the fixed arrival time to station
            u->time = u->dist % 1440;

            //stop if destination reached
            if( u->stId == arrStId)
            {
                target = u;
                restoreTsTime();
                return true;
            }

            Station& currSt = stations[u->stId];
            const Distance reachTime = u->time + currSt.minTransferTime;
            const std::vector<EdgeIterator>& stEdges = currSt.stEdges;
            assert( ( currSt.adjStations.size() + 1) == stEdges.size());

            for( unsigned int i=0, ssize=currSt.adjStations.size(); i<ssize; ++i)
            {
                std::vector<VehicleTypeID>& usedVehicles = currSt.usedVehicles[i];
                Station& destSt = stations[currSt.adjStations[i]];
                const NodeIterator nextStNode = G.getNodeIterator( destSt.stNode);
                assert( nextStNode->isSwitch());

                //if destination is already visited with an optimal arrival time then skip
                if( ( nextStNode->timestamp == *m_timestamp) && ( u->dist > ( nextStNode->dist + destSt.minTransferTime)))
                    continue;

                for( unsigned int j=0; j<usedVehicles.size(); j++)
                {
                    bool toNextDay = true;

                    //in current day (past & present & future)
                    for( e = currSt.getEdge( u->time, i, j), endEdge = currSt.stVehEdges[i][j+1]; e != endEdge; ++e)
                    {
                        depNode = G.target( e);
                        assert( depNode->isDeparture());
                        touchedNodes++;
                        touchedEdges++;
     
                        if( depNode->time >= reachTime )
                        {
                            //set switch edge weight
                            e->weight = depNode->time - u->time;

                            //update dist of depNode
                            updateDepDist( u, e, depNode);

                            //connection of depNode
                            connEdge = G.beginEdges( depNode);
                            assert( G.target( connEdge) == nextStNode);
                            touchedNodes++;
                            touchedEdges++;

                            //update dist of switch node
                            if( updateStDist( depNode, connEdge, nextStNode, destSt.minTransferTime) == false)
                            {
                                toNextDay = false;
                                break;
                            }

                            ++connEdge;
                            if( connEdge != G.endEdges( depNode))
                            {
                                nextDepNode = G.target( connEdge);
                                assert( G.outdeg( depNode) == 2);
                                assert( nextDepNode->isDeparture());
                                touchedNodes++;
                                touchedEdges++;

                                updateChain( depNode, connEdge, nextDepNode);
                            }
                        }

                        else
                        {
                            if( depNode->timestamp == *m_timestamp && G.getNodeIterator( depNode->pred)->isDeparture())
                            {
                                connEdge = G.beginEdges( depNode);
                                assert( G.target( connEdge) == nextStNode);
                                touchedNodes++;
                                touchedEdges++;

                                //update dist of switch node
                                updateStDist( depNode, connEdge, nextStNode, destSt.minTransferTime);

                                ++connEdge;
                                if( connEdge != G.endEdges( depNode))
                                {
                                    nextDepNode = G.target( connEdge);
                                    assert( G.outdeg( depNode) == 2);
                                    assert( nextDepNode->isDeparture());
                                    touchedNodes++;
                                    touchedEdges++;

                                    updateChain( depNode, connEdge, nextDepNode);
                                }
                            }
                        }
                    }

                    if( toNextDay == false) continue;

                    for( e = currSt.stVehEdges[i][j], endEdge = currSt.stVehEdges[i][j+1]; e != endEdge; ++e)
                    {
                        depNode = G.target( e);
                        assert( depNode->isDeparture());

                        if( ( depNode->time + 1440) >= reachTime)
                        {
                            //set switch edge weight
                            e->weight = ( depNode->time + 1440) - u->time;

                            //update dist of depNode
                            updateDepDist( u, e, depNode);

                            //connection of depNode
                            connEdge = G.beginEdges( depNode);
                            assert( G.target( connEdge) == nextStNode);
                            touchedNodes++;
                            touchedEdges++;

                            //update dist of switch node
                            if( updateStDist( depNode, connEdge, nextStNode, destSt.minTransferTime) == false)
                                break;

                            ++connEdge;
                            if( connEdge != G.endEdges( depNode))
                            {
                                nextDepNode = G.target( connEdge);
                                assert( G.outdeg( depNode) == 2);
                                assert( nextDepNode->isDeparture());
                                touchedNodes++;
                                touchedEdges++;

                                updateChain( depNode, connEdge, nextDepNode);
                            }
                        }

                        else if( depNode->timestamp == *m_timestamp && G.getNodeIterator( depNode->pred)->isDeparture())
                        {
                            connEdge = G.beginEdges( depNode);
                            assert( G.target( connEdge) == nextStNode);
                            touchedNodes++;
                            touchedEdges++;

                            //update dist of switch node
                            updateStDist( depNode, connEdge, nextStNode, destSt.minTransferTime);

                            ++connEdge;
                            if( connEdge != G.endEdges( depNode))
                            {
                                nextDepNode = G.target( connEdge);
                                assert( G.outdeg( depNode) == 2);
                                assert( nextDepNode->isDeparture());
                                touchedNodes++;
                                touchedEdges++;

                                updateChain( depNode, connEdge, nextDepNode);
                            }
                        }
                    }
                }
            }
        }

        restoreTsTime();
        return false;
    }


    /**
     * @brief Prints the computed shortest path.
     */
    void printShortestPath()
    {
        std::vector<NodeIterator> sp;
        NodeIterator v = target;

        if( target == G.endNodes())
        {
            std::cout << "\nTarget station is not reachable";
            return;
        }

        sp.push_back( target);

        //collect the nodes of the shortest s-t path
        while( G.hasNode( (NodeDescriptor) v->pred))
        {
            v = G.getNodeIterator( v->pred);
            sp.push_back( v);
        }

        std::cout << "\nStarting at station " << stations[source->stId].name 
                  << " (" << source->stId << ")"
                  << " at time " << startTime << "\n";

        typename std::vector<NodeIterator>::const_reverse_iterator it = sp.rbegin()+1, end = sp.rend();
        for(; it != end; ++it)
        {
            const NodeIterator& v = (*it);
            if( v->isDeparture())
                std::cout << "Departing from station " << stations[v->stId].name << " (" << v->stId << ")" << " at time " << v->time << " with vehicle " << v->vhId << "\n";

            else
                std::cout << "Arriving to station " << stations[v->stId].name << " (" << v->stId << ")" << " at time " << v->time << " with vehicle " << v->vhId << "\n";
        }

        std::cout << "Overall travel time: " << getDistance() << " mins \n";
        std::cout << "Settled nodes: " << getSettledNodes() << "\n";
    }

    /**
     * @brief Returns the shortest distance (travel time) from source station to destination station, 
     * according to the selected departure time.
     * @return The shortest distance.
     */
    Distance getDistance()
    {
        if( target != G.endNodes())
            return ( target->dist - startTime);
        else
            return std::numeric_limits<Distance>::max();

    }


 private:

    GraphType& G;
    StationContainer& stations;
    TTL *m_timestamp;
    PriorityQueueType pq;
    Time startTime;

    NodeIterator source, target;

    unsigned int settledNodes;
    unsigned int touchedNodes;
    unsigned int touchedEdges;

    Time sminTransferTime;

    /**
     * @brief Restores the original min transfer time of source station.
     */
    void restoreTsTime()
    {
        stations[source->stId].minTransferTime = sminTransferTime;
    }

    /**
     * @brief Initializes the algorithm.
     * @param depStId Id of source station.
     * @param arrStId Id of target station.
     * @param tstart Start time at source station.
     **/
    void init( const StationID depStId, const StationID arrStId, const Time tstart)
    {
        pq.clear();
        ++(*m_timestamp);

        settledNodes = 0;
        touchedNodes = 0;
        touchedEdges = 0;

        target = G.endNodes();

        Station& depStation = stations[depStId];

        sminTransferTime = depStation.minTransferTime;
        depStation.minTransferTime = 0;
        source = G.getNodeIterator( depStation.stNode);

        source->pred = G.nilNodeDescriptor();
        source->timestamp = *m_timestamp;
        startTime = source->dist = tstart;

        pq.insert( source->dist, source, &(source->pqItem));
    }


    /**
     * @brief Relaxes the departure-departure and departure-switch edges without transit.
     * @param depNode A departure node (tail).
     * @param connEdge A travel edge.
     * @param nextDepNode A departure node (head).
     **/
    inline void updateChain( NodeIterator depNode, EdgeIterator connEdge, NodeIterator nextDepNode)
    {
        while(1)
        {
            touchedNodes++;
            touchedEdges++;

            Distance discoveredDistance = depNode->dist + connEdge->weight;

            if( nextDepNode->timestamp != *(m_timestamp))
            {
                nextDepNode->pred = depNode->getDescriptor();
                nextDepNode->dist = discoveredDistance;
                nextDepNode->timestamp = *(m_timestamp);
            }

            else if( nextDepNode->dist > discoveredDistance)
            {
                nextDepNode->pred = depNode->getDescriptor();
                nextDepNode->dist = discoveredDistance;
            }

            else
                return;

            Station& dest = stations[nextDepNode->stId];
            const NodeIterator stNode = G.getNodeIterator( dest.stNode);

            if( stNode->timestamp == *(m_timestamp) && pq.isMember( &(stNode->pqItem)) == false)
            {
                connEdge = G.beginEdges( nextDepNode);
                const NodeIterator nextStNode = G.target( connEdge);
                const Station& destSt = stations[nextStNode->stId];
                assert( nextStNode->isSwitch());

                updateStDist( nextDepNode, connEdge, nextStNode, destSt.minTransferTime);

                ++connEdge;
                if( connEdge != G.endEdges( nextDepNode))
                {
                    depNode = nextDepNode;
                    nextDepNode = G.target( connEdge);
                }
            }

            else
                return;
        }
    }

    /**
     * @brief Relaxes the switch-departure edge (travel with transit).
     * @param stNode A switch node (tail).
     * @param e A travel edge.
     * @param depNode A departure node (head).
     **/
    inline bool updateDepDist( const NodeIterator& stNode, const EdgeIterator& e, const NodeIterator& depNode)
    {
        touchedNodes++;
        touchedEdges++;

        Distance discoveredDistance = stNode->dist + e->weight;

        if( depNode->timestamp != *(m_timestamp))
        {
            depNode->pred = stNode->getDescriptor();
            depNode->dist = discoveredDistance;
            depNode->timestamp = *(m_timestamp);
            return true;
        }

        else if( depNode->dist > discoveredDistance)
        {
            depNode->pred = stNode->getDescriptor();
            depNode->dist = discoveredDistance;
            return true;
        }

        else
            return false;
    }

    /**
     * @brief Relaxes the departure-switch edge.
     * @param depNode A departure node (tail).
     * @param e A departure-switch edge.
     * @param stNode A switch node (head).
     **/
    inline bool updateStDist( const NodeIterator& depNode, const EdgeIterator& e, const NodeIterator& stNode, const Time& minTransferTime)
    {
        touchedNodes++;
        touchedEdges++;

        const Distance discoveredDistance = depNode->dist + e->weight;

        if( stNode->timestamp != *(m_timestamp))
        {
            stNode->pred = depNode->getDescriptor();
            stNode->dist = discoveredDistance;
            stNode->timestamp = *(m_timestamp);

            pq.insert( stNode->dist, stNode, &(stNode->pqItem));
            return true;
        }

        else if( stNode->dist > discoveredDistance)
        {
            stNode->pred = depNode->getDescriptor();
            stNode->dist = discoveredDistance;

            pq.decrease( stNode->dist, &(stNode->pqItem));
            return true;
        }

        else if( discoveredDistance <= ( stNode->dist + minTransferTime))
            return true;

        else
            return false;  
    }
};

};

#endif
