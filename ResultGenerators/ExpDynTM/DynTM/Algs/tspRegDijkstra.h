#ifndef DYNTM_REGULAR_DIJKSTRA
#define DYNTM_REGULAR_DIJKSTRA

#include <Structs/Trees/priorityQueue.h>
#include <climits>

#include "../Structs/definitions.h"

namespace DynTM
{

struct DjkNode
{
    DjkNode(): dist(std::numeric_limits<Distance>::max()),
               pqItem(std::numeric_limits<PQRange>::max()), pred(0), timestamp(0), isMarked(false)
    {}

    union{ Time dist, distBack; };
    union{ PQRange pqItem, pqItemBack; };
    union{ void* pred, * succ; };
    union{ TTL timestamp; };
    union{ bool isMarked, isSettled; };
};

/**
 * @class TspRegDijkstra
 *
 * @brief The regular Dijkstra algorithm on dynTM time-expanded graphs.
 *
 * This class supports finding the shortest path and the shortest travel time from a source station s to a target station t.
 *
 * @tparam TspNetwork The time expanded data structure used for the representation of transporation network.
 * @author Andreas Paraskevopoulos
 *
 */
template<typename TspNetwork>
class TspRegDijkstra
{

 public:

    typedef typename TspNetwork::GraphImpl                              Graph;
    typedef typename TspNetwork::SizeType                               SizeType;
    typedef typename TspNetwork::NodeDescriptor                         NodeDescriptor;
    typedef typename TspNetwork::NodeIterator                           NodeIterator;
    typedef typename TspNetwork::EdgeDescriptor                         EdgeDescriptor;
    typedef typename TspNetwork::EdgeIterator                           EdgeIterator;
    typedef typename TspNetwork::StationContainer                       StationContainer;
    typedef typename TspNetwork::Station                                Station;
    typedef StaticPriorityQueue<Distance, NodeIterator, HeapStorage>    PriorityQueueType;

    /**
     * @brief Constructor. Creates an instance of TspRegDijkstra.
     * @param graph The graph to run the algorithm on.
     * @param stationContainer A container with the stations.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order 
     * to check whether a node is visited or not.
     */
    TspRegDijkstra( Graph& graph, StationContainer& Stations, TTL *timestamp) 
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
    bool runQuery( StationID depStId, StationID arrStId, Time tstart)
    {
        NodeIterator u, v, nextStNode, depNode, nextDepNode;
        EdgeIterator e, connEdge, tsfEdge, endEdge;

        init( depStId, arrStId, tstart);
      
        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            settledNodes++;

            //switch node
            if( u->isSwitch())
            {
                //set the fixed arrival time to station
                u->time = u->dist % 1440;

                //stop if destination reached
                if( u->stId == arrStId)
                {
                    target = u;
                    restoreTsTime();
                    return true;
                }

                const Station& currentSt = stations[u->stId];

                for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
                {
                    depNode = G.target( e);

                    assert( depNode->isDeparture());
                    assert( G.outdeg( depNode) <= 2);
                    touchedNodes++;
                    touchedEdges++;

                    //set switch edge weight
                    if( depNode->time >= ( u->time + currentSt.minTransferTime))
                        //via current day
                        e->weight = depNode->time - u->time;
                    else
                        //via next day
                        e->weight = ( depNode->time + 1440) - u->time;

                    //update dist of depNode
                    updateDist( u, depNode, e);
                }
            }

            //departure node
            else
            {
                assert( u->isDeparture());
                assert( G.outdeg( u) <= 2);
                touchedNodes++;
                touchedEdges++;

                connEdge = G.beginEdges( u);
                nextStNode = G.target( connEdge);
                assert( nextStNode->isSwitch());

                //update dist of nextStNode
                updateDist( u, nextStNode, connEdge);

                ++connEdge;
                if( connEdge != G.endEdges( u))
                {
                    nextDepNode = G.target( connEdge);
                    assert( G.outdeg( u) == 2);
                    assert( nextDepNode->isDeparture());
                    touchedNodes++;
                    touchedEdges++;

                    //update dist of nextDepNode
                    updateDist( u, nextDepNode, connEdge);
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

        typename std::vector<NodeIterator>::const_reverse_iterator it = sp.rbegin(), end = sp.rend();
        for(; it != end; ++it)
        {
            const NodeIterator& v = (*it);
            if( v->isDeparture())
            {
                std::cout << "Departing from station " << stations[v->stId].name << " (" << v->stId << ")" << " at time " << v->time << " with vehicle " << v->vhId << "\n";
            }

            else
            {
                std::cout << "Arriving to station " << stations[v->stId].name << " (" << v->stId << ")" << " at time " << v->time << " with vehicle " << v->vhId << "\n";
            }
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

    Graph& G;
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
    void init( StationID depStId, StationID arrStId, Time tstart)
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

    inline void updateDist( const NodeIterator& u, const NodeIterator& v, const EdgeIterator& e)
    {
        touchedNodes++;
        touchedEdges++;

        Distance discoveredDistance = u->dist + e->weight;

        if( v->timestamp != *(m_timestamp))
        {
            v->pred = u->getDescriptor();
            v->dist = discoveredDistance;
            v->timestamp = *(m_timestamp);

            pq.insert( v->dist, v, &(v->pqItem));
        }

        else if( v->dist > discoveredDistance)
        {
            v->pred = u->getDescriptor();
            v->dist = discoveredDistance;

            pq.decrease( v->dist, &(v->pqItem));
        }
    }
};

};

#endif
