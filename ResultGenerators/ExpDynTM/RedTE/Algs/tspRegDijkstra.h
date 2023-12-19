#ifndef TIME_EXPANDED_DIJKSTRA_H
#define TIME_EXPANDED_DIJKSTRA_H

#include <Structs/Trees/priorityQueue.h>
#include <limits>
#include <vector>
#include <cmath>
#include "../Structs/definitions.h"

struct DjkNode
{

    DjkNode() : dist(std::numeric_limits<Distance>::max()),
                pqItem(std::numeric_limits<PQRange>::max()), pred(0), timestamp(0)
    {}

    union{ Distance dist, distBack; };
    union{ PQRange pqItem, pqItemBack; };
    union{ void* pred, * succ; };
    TTL timestamp;
};

/**
 * @class TspRegDijkstra
 *
 * @brief The classical Dijkstra algorithm on time-expanded graphs.
 *
 * This class supports finding the shortest path and the shortest travel time from a source station s to a target station t.
 *
 * @tparam TspNetwork The time expanded data structure used for the representation of transporation network.
 * @author Andreas Paraskevopoulos
 *
 */

template<class TspNetwork>
class TspRegDijkstra
{

 public:

    typedef typename TspNetwork::StationContainer                          StationContainer;
    typedef typename TspNetwork::Station                                   Station;
    typedef typename TspNetwork::GraphImpl                                 GraphType;
    typedef typename TspNetwork::NodeIterator                              NodeIterator;
    typedef typename TspNetwork::EdgeIterator                              EdgeIterator;
    typedef typename TspNetwork::InEdgeIterator                            InEdgeIterator;
    typedef typename TspNetwork::NodeDescriptor                            NodeDescriptor;
    typedef StaticPriorityQueue<Distance, NodeIterator, HeapStorage>       PriorityQueueType;
    
    /**
     * @brief Constructor. Creates an instance of TspRegDijkstra.
     * @param graph The graph to run the algorithm on.
     * @param stationContainer A container with the stations.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order 
     * to check whether a node is visited or not.
     */
    TspRegDijkstra( GraphType& graph, StationContainer& stationContainer, TTL* timestamp): 
    G(graph), stations(stationContainer), m_timestamp(timestamp)
    {
        pq.reserve( G.getNumNodes());
        target = G.endNodes();
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
     * @brief Runs a single-pair shortest path query from a source node s to a target node t.
     * @param s The source node.
     * @param t The target node.
     **/
    void runQuery( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u, v;
        EdgeIterator e, lastedge;
        Distance discoveredDistance;

        init( s);
        source = s;
        target = t;

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++settledNodes;

            if( u == t) 
                break;

            for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
            {
                discoveredDistance = u->dist + e->weight;
                v = G.target(e);

                if( v->timestamp != (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->timestamp = (*m_timestamp);
                    v->dist = discoveredDistance;
                    pq.insert( v->dist, v, &(v->pqItem));   
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( v->dist, &(v->pqItem));
                }

                touchedNodes++;
                touchedEdges++;
            }
        }
    }

    /**
     * @brief Runs a single-source shortest path query from a source-station.
     * @param depStationId The id of departure-station.
     **/
    void buildTree( const StationID depStationId)
    {
        NodeIterator u, v;
        EdgeIterator e, lastedge;
        Distance discoveredDistance;

        init( depStationId);

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++settledNodes;

            for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
            {
                discoveredDistance = u->dist + e->weight;
                v = G.target(e);

                if( v->timestamp != (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->timestamp = (*m_timestamp);
                    v->dist = discoveredDistance;
                    pq.insert( v->dist, v, &(v->pqItem));
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( v->dist, &(v->pqItem));
                }

                touchedNodes++;
                touchedEdges++;
            }
        }
    }

    /**
     * @brief Runs a single-source shortest path query from a source-station.
     * @param depStationId The id of departure-station.
     **/
    void optBuildTree( const StationID depStationId)
    {
        NodeIterator u, v;
        EdgeIterator e, lastedge;
        Distance discoveredDistance;
        bool depUpdate;
        NodeIterator depNode;

        init( depStationId);

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++settledNodes;

            scanDep1:
            depUpdate = false;

            for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
            {
                discoveredDistance = u->dist + e->weight;
                v = G.target(e);

                if( v->timestamp != (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->timestamp = (*m_timestamp);
                    v->dist = discoveredDistance;
                    if( v->isArrival() == true)
                        pq.insert( v->dist, v, &(v->pqItem));
                    else
                    {
                        depNode = v;
                        depUpdate = true;
                    }
                    
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    if( v->isArrival() == true)
                        pq.decrease( v->dist, &(v->pqItem));
                    else
                    {
                        depNode = v;
                        depUpdate = true;
                    }
                }

                touchedNodes++;
                touchedEdges++;
            }

            if( depUpdate == true)
            {
                u = depNode;
                goto scanDep1;
            }
        }
    }

    /**
     * @brief Runs a reverse single-source shortest path query from a source node s.
     * @param s The source node.
     **/
    void buildRevTree( const NodeIterator& s)
    {
        NodeIterator u, v;
        InEdgeIterator k, lastInedge;
        EdgeIterator e;
        Distance discoveredDistance;
        bool depUpdate;
        NodeIterator depNode;

        init( s);
        source = s;
        target = G.endNodes();

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++settledNodes;

            scanDep2:
            depUpdate = false;

            for( k = G.beginInEdges(u), lastInedge = G.endInEdges(u); k != lastInedge; ++k)
            {
                e = G.getEdgeIterator( k);
                v = G.source( k);
                discoveredDistance = u->dist + e->weight;

                if( v->timestamp != (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->timestamp = (*m_timestamp);
                    v->dist = discoveredDistance;
                    if( v->isArrival() == true)
                        pq.insert( v->dist, v, &(v->pqItem));
                    else
                    {
                        depNode = v;
                        depUpdate = true;
                    }
                    
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    if( v->isArrival() == true)
                        pq.decrease( v->dist, &(v->pqItem));
                    else
                    {
                        depNode = v;
                        depUpdate = true;
                    }
                }

                touchedNodes++;
                touchedEdges++;
            }

            if( depUpdate == true)
            {
                u = depNode;
                goto scanDep2;
            }
        }
    }


    /**
     * @brief Runs a shortest path query from a source-station to a destination-station, with the regular Dijkstra's algorithm.
     * @param depStationId The id of departure-station.
     * @param arrStationId The id of the destination-station.
     * @param startTime The start time at the departure-station.
     **/
    void runQuery( const StationID depStationId, const StationID arrStationId, const Time startTime)
    {
        NodeIterator u, v;
        EdgeIterator e, lastedge;
        Distance discoveredDistance;

        init( depStationId, arrStationId, startTime);

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++settledNodes;

            if( u->stId == arrStationId) 
            { 
                target = u;  
                break;
            }

            for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
            {
                discoveredDistance = u->dist + e->weight;
                v = G.target(e);

                if( v->timestamp != (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->timestamp = (*m_timestamp);
                    v->dist = discoveredDistance;
                    pq.insert( v->dist, v, &(v->pqItem));
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( v->dist, &(v->pqItem));
                }

                touchedNodes++;
                touchedEdges++;
            }
        }
    }
 
    /**
     * @brief Prints the computed shortest path.
     */
    void printShortestPath()
    {
        std::vector<NodeIterator> sp;
        NodeIterator v = target;
        bool wait = false;

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

            if( v->isDeparture() && wait == false)
            {
                sp.push_back( v);
                wait = true;
            }

            else if( v->isArrival())
            {
                sp.push_back( v);
                wait = false;
            }
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

        std::cout << "Overall travel time: " << getDistance() << "\n";
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


    /**
     * @brief Checks if the node v has been visited (enqueued) by the algorithm.
     * @return True if the node v is explored, otherwise false.
     */
    bool isExplored( const NodeIterator& v) const
    {
        return ( v->timestamp >= (*m_timestamp));
    }

 private:

    GraphType& G;
    StationContainer& stations;
    PriorityQueueType pq;
    TTL* m_timestamp;

    unsigned int settledNodes;
    unsigned int touchedNodes;
    unsigned int touchedEdges;

    NodeIterator source, target;
    Time startTime;
   
    /**
     * @brief Resets the timestamp of all nodes.
     **/
    void resetTimestamps()
    {
        //reset the timestamp for each node in graph, if the global counter (m_timestamp) is going to be overflowed
        if( ( (*m_timestamp) + 1) == 0)
        {
            (*m_timestamp) = 0;
            for( NodeIterator v = G.beginNodes(), lastNode = G.endNodes(); v!=lastNode; ++v)
                v->timestamp = 0;
        }
    }

    /*
     * @brief Initliazes the algorithm.
     * @param s The source node.
     **/
    void init( const NodeIterator& s)
    {
        resetTimestamps();

        pq.clear();
        settledNodes = 0;
        ++(*m_timestamp);

        source = s;
        target = G.endNodes();

        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();

        pq.insert( s->dist, s, &(s->pqItem));
    }

    /**
     * @brief Initliazes single-pair Dijkstra's algorithm.
     * @param depStationId The id of departure-station.
     * @param arrStationId The id of the destination-station.
     * @param startTime The start time at the departure-station.
     **/
    void init( const StationID depStationId, const StationID arrStationId, const Time startTime)
    {
        pq.clear();
        ++(*m_timestamp);
        target = G.endNodes();

        settledNodes = 0;
        touchedNodes = 0;
        touchedEdges = 0;

        this->startTime = startTime;
        Station& depStation = stations[depStationId];

        if( depStation.numDepEvents == 0)
            return;

        NodeIterator firstDepNode = G.getNodeIterator( depStation.firstDepNode);
        source = firstDepNode;
        unsigned int counter = 0;

        while( source->time < startTime && counter < depStation.numDepEvents)
        {
            for( EdgeIterator e = G.beginEdges(source), lastedge = G.endEdges(source); e != lastedge; ++e)
            {
                NodeIterator v = G.target( e);

                if( v->isDeparture() == true)
                {
                    source = v;
                    break;
                }
            }

            counter++;
        }

        if( counter == depStation.numDepEvents)
        {
            source = firstDepNode;
            source->dist = 1440 + firstDepNode->time;
        }

        else
            source->dist = source->time;

        source->timestamp = (*m_timestamp);
        source->pred = G.nilNodeDescriptor();
        pq.insert( source->dist, source, &(source->pqItem));

        //test graph
        /*{
            NodeIterator u, v, endNode;
            EdgeIterator e, endEdge;

            for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
                for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
                {
                    v = G.target( e);

                    std::cout << "\n(s" << u->stId << ", v" << u->vhId << ", " << u->label << ", t" << u->time << ") " << e->weight << " -> "
                              << "(s" << v->stId << ", v" << v->vhId << ", " << v->label << ", t" << v->time << ")";
                }
            exit(1);
        }*/
    }


   /**
     * @brief Initliazes singe-sourceDijkstra's algorithm.
     * @param depStationId The id of departure-station.
     **/
    void init( const StationID depStationId)
    {
        pq.clear();
        ++(*m_timestamp);

        settledNodes = 0;
        touchedNodes = 0;
        touchedEdges = 0;

        Station& depStation = stations[depStationId];
        if( depStation.numDepEvents == 0)
            return;

        NodeIterator depNode = G.getNodeIterator( depStation.firstDepNode);
        unsigned int counter = 0;

        //insert all arrival nodes of station
        while( counter < depStation.numDepEvents)
        {
            for( InEdgeIterator e = G.beginInEdges( depNode), lastedge = G.endInEdges( depNode); e != lastedge; ++e)
            {
                const NodeIterator v = G.source( e);

                if( v->isArrival())
                {
                    v->timestamp = (*m_timestamp);
                    v->pred = G.nilNodeDescriptor();
                    v->dist = 0;
                    pq.insert( v->dist, v, &(v->pqItem));
                }
                else
                    depNode = v;
            }

            counter++;
        }
    }
};

#endif
