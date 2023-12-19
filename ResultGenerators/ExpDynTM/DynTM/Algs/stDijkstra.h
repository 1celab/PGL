#ifndef STATIC_DIJKSTRA_H
#define STATIC_DIJKSTRA_H

#include <Structs/Trees/priorityQueue.h>
#include <Structs/Trees/priorityQueueNoDec.h>

/**
 * @class Dijkstra
 *
 * @brief Regular Dijkstra's algorithm implementation.
 * This class supports building a full shortest path tree from a source node s, or running queries between source and target nodes
 *
 * @tparam GraphType The type of graph to run an algorithm on.
 * @tparam DistType The distance domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType, class DistType=unsigned int>
class Dijkstra
{

 public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef PriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    Dijkstra( GraphType& graph, unsigned int* timestamp): G(graph), m_timestamp(timestamp)
    {}
    

    /**
     * @brief Returns the number of the nodes that have have been settled (enqueued and dequeued) by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int& getNumSettledNodes() const
    {
        return m_numSettledNodes;
    }


    /**
     * @brief Returns the number of the nodes that have have been visited by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int getNumVisitedNodes()
    {
        return ( m_numSettledNodes + pq.size());
    }


    /**
     * @brief Returns the number of nodes in the shortest s-v path.
     * @param v A target node.
     * @return The number of nodes in shortest s-v path.
     **/
    unsigned int getNumSPNodes( NodeIterator v) const
    {
        unsigned int numSPNodes = 1;

        while( G.hasNode( (NodeDescriptor) v->pred))
        {
            numSPNodes++;
            v = G.getNodeIterator( v->pred);
        }

        return numSPNodes;
    }


    /**
     * @brief Builds a shortest path tree routed on a source node.
     * @param s The source node.
     */
    void buildTree( const NodeIterator& s)
    {
        NodeIterator u, v, endNode;
        EdgeIterator e, lastEdge;
        DistType discoveredDistance;
        
        init( s);

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_numSettledNodes;
    
            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                discoveredDistance = u->dist + e->weight;
                v = G.target(e);

                if( v->timestamp < (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pq.insert( v->dist, v, &(v->pqItem));
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( v->dist, &(v->pqItem));
                }
            }
        }
    }


    /**
     * @brief Builds a shortest path tree routed on a source node.
     * @param s The source node.
     * @param targets Vector with the nearest station nodes to source node.
     * @param maxDist The max travel time radius around source.
     */
    void reachNearestStations( const NodeIterator& s, std::vector<NodeIterator>& targets, int maxNumTargets, DistType maxDist)
    {
        NodeIterator u, v, endNode;
        EdgeIterator e, lastEdge;
        DistType discoveredDistance;
        
        init( s);

        const StationID stId = s->stId;
        s->stId = std::numeric_limits<StationID>::max();

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_numSettledNodes;

            //termination criteria
            if( u->dist <= maxDist)
            {
                if( u->stId != std::numeric_limits<StationID>::max())
                {
                    targets.push_back( u);
                    if( maxNumTargets <= targets.size())
                        break;
                }
            }

            else
                break;

            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                discoveredDistance = u->dist + e->weight;
                v = G.target(e);

                if( v->timestamp < (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pq.insert( v->dist, v, &(v->pqItem));
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( v->dist, &(v->pqItem));
                }
            }
        }

        s->stId = stId;
    }


    /**
     * @brief Runs a shortest path query between a source node s and a target node t.
     * @param s The source node.
     * @param t The target node.
     * @return The distance of the target node.
     */
    DistType runQuery( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u, v, endNode;
        EdgeIterator e, lastEdge;
        DistType discoveredDistance;

        init( s);
        t->dist = std::numeric_limits<DistType>::max();

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_numSettledNodes;

            if( u == t) break;

            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                discoveredDistance = u->dist + e->weight;
                v = G.target(e);

                if( v->timestamp < (*m_timestamp))
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
            }
        }

        return t->dist;
    }

 private:

    GraphType& G;
    PriorityQueueType pq;
    unsigned int* m_timestamp;
    unsigned int m_numSettledNodes;

    /*
     * @brief Initializes the algorithm.
     * @param s The source node.
     **/
    void init( const NodeIterator& s)
    {
        //reset the timestamp of each node in graph
        resetTimestamps();

        pq.clear();
        ++(*m_timestamp);

        m_numSettledNodes = 0;

        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();
        pq.insert( 0, s, &(s->pqItem));
    }

    /**
     * @brief Resets the timestamp of all nodes.
     **/
    void resetTimestamps()
    {
        //reset the timestamp for each node in graph, if the global counter (m_timestamp) is going to be overflowed
        if( ( (*m_timestamp) + 1) == 0)
        {
            (*m_timestamp) = 0;
            for( NodeIterator u = G.beginNodes(), endNode = G.endNodes(); u!=endNode; ++u)
                u->timestamp = 0;
        }
    }
};

#endif//DIJKSTRA_H
