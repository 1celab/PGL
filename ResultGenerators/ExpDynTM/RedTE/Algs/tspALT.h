#ifndef TIME_EXPANDED_ALT_H
#define TIME_EXPANDED_ALT_H

#include <Structs/Trees/priorityQueueNoDec.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include "../Structs/definitions.h"

struct ALTNode
{
    ALTNode() : dist(std::numeric_limits<Distance>::max()),
                pqkey(std::numeric_limits<PQRange>::max()), pred(0), timestamp(0)
    {}

    union{ Time dist, distBack; };
    union{ PQRange pqkey, pqkeyBack; };
    union{ void* pred, * succ; };
    TTL timestamp;
};


/**
 * @class TspALT
 *
 * @brief The ALT algorithm on Dynamic Timetable graph (time-expanded approach).
 *
 * This class supports finding efficiently the shortest path from a source station s, at a given departure time, to a target station t.
 *
 * @tparam TspNetwork The time expanded structure used for the representation of public transporation network.
 * @author Andreas Paraskevopoulos
 *
 */
template<class TspNetwork>
class TspALT
{

 public:

    typedef typename TspNetwork::GraphImpl                              GraphType;
    typedef typename TspNetwork::SizeType                               SizeType;
    typedef typename TspNetwork::NodeDescriptor                         NodeDescriptor;
    typedef typename TspNetwork::NodeIterator                           NodeIterator;
    typedef typename TspNetwork::EdgeDescriptor                         EdgeDescriptor;
    typedef typename TspNetwork::EdgeIterator                           EdgeIterator;
    typedef typename TspNetwork::StationContainer                       StationContainer;
    typedef typename TspNetwork::Station                                Station;

    typedef StaticPriorityQueue<Distance, NodeIterator, HeapStorage>    PriorityQueueType;
    
    /**
     * @brief Constructor. Creates an instance of TspALT.
     * @param graph The graph to run the algorithm on.
     * @param stationContainer A container with the stations.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order 
     * to check whether a node is visited or not.
     */
    TspALT( GraphType& graph, StationContainer& stationContainer, TTL* timestamp): 
    G(graph), stations(stationContainer), m_timestamp(timestamp)
    {
        pq.reserve( G.getNumNodes());
    }

    /**
     * @brief Constructor. Creates an instance of TspALT.
     * @param graph The graph to run the algorithm on.
     * @param stationContainer A container with the stations.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order 
     * to check whether a node is visited or not.
     * @param lowBndFileName The name of file which contains lower bound distances - heuristic data.
     */
    TspALT( GraphType& graph, StationContainer& stationContainer, TTL* timestamp, 
    const std::string& lowBndFileName): 
    G(graph), stations(stationContainer), m_timestamp(timestamp)
    {
        pq.reserve( G.getNumNodes());
        loadHeuristicData( lowBndFileName);
    }

    /**
     * @brief Loads lower bound distance heuristic data.
     * @param lowBndFileName The name of file which contains lower bound distances - heuristic data.
     */
    void loadHeuristicData( const std::string& lowBndFileName)
    {
        //data file
        std::ifstream in( lowBndFileName.c_str(), std::ios::in | std::ios::binary);
        if( !in)
        {
            std::cerr << "\nError opening file '" << lowBndFileName << "'" << std::endl;
            return;
        }

        //number of stations
        unsigned int numStations;
        in.read( (char*) ( &numStations), sizeof( numStations));

        ProgressStream stLowBnds_progress( numStations);
        stLowBnds_progress.label() << "Reading ALT heuristics for " << numStations << " stations";

        stLowBnds.clear();
        stLowBnds.resize( numStations);

        //read the data
        for( StationID stId=0; stId<numStations; stId++)
        {
            stLowBnds[stId].resize( numStations, 0);

            in.read( (char*) (&stLowBnds[stId][0]), numStations * sizeof( stLowBnds[stId][0]));

            /*for( int j=0; j<numStations; j++)
            {
                std::cout << stLowBnds[stId][j] << std::endl;
                {int a; std::cin >> a;}
            }*/

            ++stLowBnds_progress;
        }    

        in.close();
    }

    /**
     * @brief Removes heuristic data.
     */
    void clearHeuristicData()
    {
        pq.clear();
        std::vector< std::vector<Distance> > empty;
        stLowBnds.swap( empty);
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
     * @brief Runs a shortest path query from a source-station to a destination-station.
     * @param depStationId The id of departure-station.
     * @param arrStationId The id of the destination-station.
     * @param startTime The start time at the departure-station.
     **/
    void runQuery( const StationID depStationId, const StationID arrStationId, const Time startTime)
    {
        NodeIterator u;

        init( depStationId, arrStationId, startTime);

        if( !pq.empty())
        {
            //depNode
            u = pq.minItem();
            pq.popMin();
            ++settledNodes;

            scan( u);
        }

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

            if( stations[u->stId].numDepEvents != 0)
            {
                //arrNode
                u = setDep( u);

                //depNode
                scan( u);
            }
        }
    }

    /**
     * @brief Checks if the node v has been visited (enqueued) by the algorithm.
     * @return True if the node v is explored, otherwise false.
     */
    bool isExplored( const NodeIterator& v) const
    {
        return ( v->timestamp >= (*m_timestamp));
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
    PriorityQueueType pq;
    TTL* m_timestamp;

    StationID stDestId;

    unsigned int settledNodes;
    unsigned int touchedNodes;
    unsigned int touchedEdges;

    std::vector< std::vector<Distance> > stLowBnds;
    std::vector<NodeIterator> destArrNodes;

    NodeIterator source, target;
    Time startTime;
   
    NodeIterator setDep( NodeIterator& arrNode)
    {
        NodeIterator nodeEvent, destArrNode, depNode = G.endNodes();
        Distance discoveredDistance;
        EdgeIterator e, lastedge;
       
        assert( arrNode->isArrival());

        for( e = G.beginEdges(arrNode), lastedge = G.endEdges(arrNode); e != lastedge; e++)
        {
            nodeEvent = G.target( e);
            discoveredDistance = arrNode->dist + e->weight;

            touchedNodes++;
            touchedEdges++;

            //arrival node in adjacent destination station (route without transit)
            if( nodeEvent->isArrival())
            {
                destArrNode = nodeEvent;
        
                if( destArrNode->timestamp != (*m_timestamp))
                {
                    destArrNode->timestamp = (*m_timestamp);
                    destArrNode->dist = discoveredDistance;
                    destArrNode->pred = arrNode->getDescriptor();
                }

                else if( destArrNode->dist > discoveredDistance)
                {
                    destArrNode->dist = discoveredDistance;
                    destArrNode->pred = arrNode->getDescriptor();
                }

                else
                    continue;

                destArrNodes.push_back( destArrNode);
            }

            //departure node in current depStation
            else if( nodeEvent->isDeparture())
            {
                depNode = nodeEvent;

                //set distance
                if( depNode->timestamp != (*m_timestamp))
                {
                    depNode->timestamp = (*m_timestamp);
                    depNode->dist = discoveredDistance;
                    depNode->pred = arrNode->getDescriptor();
                }

                //update distance
                else if( depNode->dist > discoveredDistance)
                {
                    depNode->dist = discoveredDistance;
                    depNode->pred = arrNode->getDescriptor();
                }
            }
        }

        for( unsigned int i=0, size=destArrNodes.size(); i<size; i++)
        {
            destArrNode = destArrNodes[i];
            Station& destStation = stations[destArrNode->stId];

            /*if( destArrNode->dist <= destStation.minDist) //X0
                destStation.minDist = destArrNode->dist;  //X0
            pq.insert( key( destArrNode), destArrNode);   //X0*/

            if( destArrNode->dist <= destStation.minDist)
            {
                destStation.minDist = destArrNode->dist;
                if( depNode->dist <= (destStation.transitTime + destStation.minDist) && destStation.numDepEvents != 1)
                    destStation.optArrNodes.push_back( destArrNode);
                else
                    pq.insert( key( destArrNode), destArrNode); 
            }

            else if( destArrNode->dist <= ( destStation.transitTime + destStation.minDist) && G.outdeg( destArrNode) > 1)
            {
                if( depNode->dist <= (destStation.transitTime + destStation.minDist) && destStation.numDepEvents != 1)
                    destStation.tnsArrNodes.push_back( destArrNode);
                else
                    pq.insert( key( destArrNode), destArrNode); 
            }
        }

        destArrNodes.clear();
 
        assert( depNode!=G.endNodes());

        return depNode;
    }

    void scan( NodeIterator& depNode)
    {
        NodeIterator v, destArrNode, nextDepNode;
        EdgeIterator e, lastedge;
        Distance discoveredDistance;

        assert(depNode->isDeparture());

        //depStation was settled
        Station& depStation = stations[depNode->stId];

        unsigned int numReachedStations = 0;
        for( unsigned int i=0, size=depStation.adjStations.size(); i<size; i++)
        {
            Station& destStation = stations[depStation.adjStations[i]];

            //the adjacent destination station is not settled
            if( destStation.minDist == std::numeric_limits<Distance>::max() || 
                depNode->dist <= (destStation.transitTime + destStation.minDist))
                destStation.isSettled = false;

            //the adjacent destination station is settled
            else
            {
                destStation.isSettled = true;
                numReachedStations++;
            }
        }

        //skip if all adjacent destination stations are settled
        if( numReachedStations == depStation.adjStations.size())
            return;

        //case A: depStation has more than one dep node
        if( depStation.numDepEvents > 1)
        {
            do
            {
                touchedNodes+=2;
                touchedEdges+=2;

                //arrival node in a adjacent destination station (route with transit)
                e = G.beginEdges( depNode);

                destArrNode = G.target( e);
                assert(destArrNode->isArrival());
                Station& destStation = stations[destArrNode->stId];

                assert( std::find( depStation.adjStations.begin(), 
                        depStation.adjStations.end(), destArrNode->stId) != depStation.adjStations.end());

                if( destStation.isSettled == false)
                {
                    discoveredDistance = depNode->dist + e->weight;

                    if( destArrNode->timestamp != (*m_timestamp))
                    {
                        destArrNode->timestamp = (*m_timestamp);
                        destArrNode->dist = discoveredDistance;
                        destArrNode->pred = depNode->getDescriptor();
                    }

                    else if( destArrNode->dist > discoveredDistance)
                    {
                        destArrNode->dist = discoveredDistance;
                        destArrNode->pred = depNode->getDescriptor();
                    }

                    else
                        goto NextDepNode;
                
                    //set min arrival time to a connected arrival-station 
                    //(according to the current selected arrival time at the departure-station)
                    if( destArrNode->dist <= destStation.minDist)
                    {
                        if( destArrNode->dist < destStation.minDist)
                        {
                            destStation.minDist = destArrNode->dist;

                            while( destStation.optArrNodes.empty() == false)
                            {
                                v = destStation.optArrNodes.back();
                                destStation.optArrNodes.pop_back();
                                assert( v->isArrival());

                                if( G.outdeg( v) > 1)
                                    destStation.tnsArrNodes.push_back( v);
                            }

                            destStation.optArrNodes.push_back( destArrNode);
                        }

                        else
                            destStation.optArrNodes.push_back( destArrNode);
                    }
            
                    //get the next earliest arrival time, only for routes without transit
                    else if( destArrNode->dist <= (destStation.transitTime + destStation.minDist) && G.outdeg( destArrNode) > 1)
                        destStation.tnsArrNodes.push_back( destArrNode);

                    //arrival station is settled (any next route to destStation cannot be optimal)
                    else if( depNode->dist > (destStation.transitTime + destStation.minDist))
                    {
                        destStation.isSettled = true;
                        numReachedStations++;   
                        if( numReachedStations == depStation.adjStations.size())
                            break;
                    }
                }

                NextDepNode:
                //departure node
                e++;

                nextDepNode = G.target( e);
                assert(nextDepNode->isDeparture());
                touchedEdges++;

                discoveredDistance = depNode->dist + e->weight;
     
                //unexplored - set travel time
                if( nextDepNode->timestamp != (*m_timestamp))
                {
                    nextDepNode->timestamp = (*m_timestamp);
                    nextDepNode->pred = depNode->getDescriptor(); 
                    nextDepNode->dist = discoveredDistance;
                }

                //explored - update shortest travel time
                else if( nextDepNode->dist > discoveredDistance)
                {
                    nextDepNode->pred = depNode->getDescriptor(); 
                    nextDepNode->dist = discoveredDistance;
                }

                else
                    break;

               depNode = nextDepNode;

            } while( true);
  

            for( unsigned int i=0, end = depStation.adjStations.size(); i < end; i++)
            {
                Station& destStation = stations[depStation.adjStations[i]];
                assert( destStation.minDist != std::numeric_limits<Distance>::max());

                for( unsigned int j=0, oSize = destStation.optArrNodes.size(); j < oSize; j++)
                {
                    v = destStation.optArrNodes[j];
                    assert(v->isArrival());
                    pq.insert( key( v), v);
                }

                destStation.optArrNodes.clear();

                for( unsigned int j=0, tSize = destStation.tnsArrNodes.size(); j < tSize; j++)
                {
                    v = destStation.tnsArrNodes[j];
                    assert(v->isArrival());

                    if( v->dist <= (destStation.transitTime + destStation.minDist))
                        pq.insert( key( v), v);
                }

                destStation.tnsArrNodes.clear();
            }
        }

        //case B: depStation has only one dep node
        else
        {
            touchedNodes++;
            touchedEdges++;
            e = G.beginEdges( depNode);
            destArrNode = G.target( e);
            assert(destArrNode->isArrival());
            discoveredDistance = depNode->dist + e->weight;
    
            if( destArrNode->timestamp != (*m_timestamp))
            {
                destArrNode->timestamp = (*m_timestamp);
                destArrNode->dist = discoveredDistance;
                destArrNode->pred = depNode->getDescriptor();
            }

            else if( destArrNode->dist > discoveredDistance)
            {
                destArrNode->dist = discoveredDistance;
                destArrNode->pred = depNode->getDescriptor();
            }
            else
                return;

            Station& destStation = stations[destArrNode->stId];

            if( destArrNode->dist <= destStation.minDist)
            {
                destStation.minDist = destArrNode->dist;
                pq.insert( key( destArrNode), destArrNode); 
            }
    
            else if( destArrNode->dist <= ( destStation.transitTime + destStation.minDist) && G.outdeg( destArrNode) > 1)
                pq.insert( key( destArrNode), destArrNode);
        }
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
        stDestId = arrStationId;

        settledNodes = 0;
        touchedNodes = 0;
        touchedEdges = 0;

        this->startTime = startTime;
        Station& depStation = stations[depStationId];

        if( stLowBnds.size() == 0)
            return;

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
        pq.insert( 0, source);

        //reset station-states
        for( unsigned int stationID = 0, end = stations.size(); stationID<end; stationID++)
        {
            Station& station = stations[stationID];

            station.minDist = std::numeric_limits<Distance>::max();
            station.timestamp = 0;
            station.isSettled = false;
            station.tnsArrNodes.clear();
            station.optArrNodes.clear();
        }

        depStation.timestamp = (*m_timestamp);
        depStation.isSettled = true;
        depStation.minDist = 0;
    }

    /**
     * @brief Returns the shortest distance from source to v and the lower bound distance from v to target.
     * @brief v A node.
     * @retun The priority key which corresponds to v.
     **/
    Distance key( const NodeIterator& v)
    {
        /*std::cout << "s->v:" << v->dist
                  << " v->t:" << stLowBnds[v->stId][stDestId] << std::endl;
        {int a; std::cin>>a;}*/

        return v->dist + stLowBnds[v->stId][stDestId];
    }
};

#endif //RAIL_NETWORK_DIJKSTRA_H
