#ifndef BIDIRECTIONAL_DIJKSTRA_H
#define BIDIRECTIONAL_DIJKSTRA_H

#include <Structs/Trees/priorityQueue.h>
#include <Structs/Trees/priorityQueueNoDec.h>

/**
 * @class BidirectionalDijkstra
 *
 * @brief Bidirectional Dijkstra's algorithm implementation.
 * This class supports running shortest path queries from a source node to target node.
 * This algorithm builds and expands two search trees, one from source node and one from target node.
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The distance domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */
template<class GraphType, class DistType=unsigned int>
class BidirectionalDijkstra
{

 public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef typename GraphType::InEdgeIterator                      InEdgeIterator;
    typedef PriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not.
     */
    BidirectionalDijkstra( GraphType& graph, unsigned int* timestamp): G(graph), m_timestamp(timestamp)
    {}
    

    /**
     * @brief Returns the number of the nodes that have have been settled (enqueued and dequeued) by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int getNumSettledNodes() const
    {
        return m_numSettledNodes;
    }


    /**
     * @brief Returns the number of the nodes that have have been visited by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int getNumVisitedNodes()
    {
        return ( m_numSettledNodes + pqFront.size() + pqBack.size());
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
     * @brief Returns the shortest s-v path.
     * @param v A target node.
     * @return The number of nodes in shortest s-v path.
     **/
    std::vector<NodeIterator> getShPath( NodeIterator v) const
    {
        std::vector<NodeIterator> shpath;
        shpath.push_back( v);

        while( G.hasNode( (NodeDescriptor) v->pred))
        {
            v = G.getNodeIterator( v->pred);
            shpath.push_back( v);
        }

        return shpath;
    }


    /**
     * @brief Runs a shortest path query between a source node s and a target node t
     *
     * @param s The source node
     * @param t The target node
     * @return The distance of the target node
     */
    DistType runQuery( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u, v, endNode;
        EdgeIterator e, endEdge;

        init( s, t);

        while( !(pqFront.empty() || pqBack.empty()))
        {
            if( (pqFront.minKey() + pqBack.minKey()) >= m_minDistance)
                break;

            searchForward();
            searchBackward();
        }

        if( m_viaNode != G.endNodes())
        {
            u = m_viaNode;
            t->dist = m_minDistance;

            while( u->succ != G.nilNodeDescriptor())
            {
                v = G.getNodeIterator(u->succ);
                v->pred = G.getNodeDescriptor( u);
                u = v;
            }
        
            return t->dist;
        }

        return std::numeric_limits<DistType>::max();
    }

 private:

    GraphType& G;
    unsigned int* m_timestamp;
    PriorityQueueType pqFront, pqBack;
    NodeIterator m_viaNode;
    DistType m_minDistance;
    unsigned int m_numSettledNodes;    

    /*
     * @brief Initializes the algorithm.
     * @param s The source node.
     * @param t The target node.
     **/
    void init( const NodeIterator& s, const NodeIterator& t)
    {
        resetTimestamps();

        m_viaNode = G.endNodes();

        m_minDistance = std::numeric_limits<DistType>::max();
        m_numSettledNodes = 0;

        pqFront.clear();
        pqBack.clear();
        ++(*m_timestamp);

        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();
        pqFront.insert( 0, s, &(s->pqitem));

        t->distBack = 0;
        t->timestampBack = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();
        pqBack.insert( 0, t, &(t->pqitemBack));
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
                u->timestamp = u->timestampBack = 0;
        }
    }


    /*
     * @brief Expands forward tree.
     **/
    void searchForward()
    {
        EdgeIterator e, endEdge;
        NodeIterator u, v;
        DistType discoveredDistance;
        
        u = pqFront.minItem();
        pqFront.popMin();
        ++m_numSettledNodes;

        for( e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
        {
            discoveredDistance = u->dist + e->weight;
            //discoveredDistance = u->dist + e->distance;
            v = G.target(e);

            //explored by the forward search
            if( v->timestamp < (*m_timestamp))
            {
                v->timestamp = (*m_timestamp);
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();

                pqFront.insert( v->dist, v, &(v->pqitem));

                //meeting point with the backward search
                if( ( v->timestampBack == (*m_timestamp)) && ( discoveredDistance + v->distBack < m_minDistance))
                {
                    m_minDistance = discoveredDistance + v->distBack;
                    m_viaNode = v;
                }
            }

            //updated by the forward search
            else if(  v->dist > discoveredDistance)
            {
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();

                pqFront.decrease( v->dist, &(v->pqitem));

                //meeting point with the backward search
                if( ( v->timestampBack == (*m_timestamp)) && ( discoveredDistance + v->distBack < m_minDistance))
                {
                    m_minDistance = discoveredDistance + v->distBack;
                    m_viaNode = v;
                }
            }
        }
    }
    

    /*
     * @brief Expands backward tree.
     **/
    void searchBackward()
    {
        InEdgeIterator k, endInEdge;
        NodeIterator u, v;
        DistType discoveredDistance;
        
        u = pqBack.minItem();
        pqBack.popMin();
        ++m_numSettledNodes;

        for( k = G.beginInEdges(u), endInEdge = G.endInEdges(u); k != endInEdge; ++k)
        {
            discoveredDistance = u->distBack + k->weight;
            v = G.source(k);

            //explored by the backward search
            if( v->timestampBack < (*m_timestamp))
            {
                v->timestampBack = (*m_timestamp);
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();

                pqBack.insert( v->distBack, v, &(v->pqitemBack));

                //meeting point with the forward search
                if( ( v->timestamp == (*m_timestamp)) && ( v->dist + discoveredDistance < m_minDistance))
                {
                    m_minDistance = v->dist + discoveredDistance;
                    m_viaNode = v;
                }
            }

            //updated by the backward search
            else if( v->distBack > discoveredDistance)
            {
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();

                pqBack.decrease( v->distBack, &(v->pqitemBack));

                //meeting point with the forward search
                if( ( v->timestamp == (*m_timestamp)) && ( v->dist + discoveredDistance < m_minDistance))
                {
                    m_minDistance = v->dist + discoveredDistance;
                    m_viaNode = v;
                }
            }
        }
    }
};

namespace StcNoDec
{

/**
 * @class BidirectionalDijkstra
 *
 * @brief Bidirectional Dijkstra's algorithm implementation.
 * This class supports running shortest path queries from a source node to target node.
 * This algorithm builds and expands two search trees, one from source node and one from target node.
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The distance domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */
template<class GraphType, class DistType=unsigned int>
class BidirectionalDijkstra
{

 public:

    typedef typename GraphType::NodeIterator                                   NodeIterator;
    typedef typename GraphType::NodeDescriptor                                 NodeDescriptor;
    typedef typename GraphType::EdgeIterator                                   EdgeIterator;
    typedef typename GraphType::InEdgeIterator                                 InEdgeIterator;
    typedef StaticNoDecPriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not.
     */
    BidirectionalDijkstra( GraphType& graph, unsigned int* timestamp): G(graph), m_timestamp(timestamp)
    {
        pqFront.reserve( G.getNumNodes());
        pqBack.reserve( G.getNumNodes());
    }
    

    /**
     * @brief Returns the number of the nodes that have have been settled (enqueued and dequeued) by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int getNumSettledNodes() const
    {
        return m_numSettledNodes;
    }


    /**
     * @brief Returns the number of the nodes that have have been visited by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int getNumVisitedNodes()
    {
        return ( m_numSettledNodes + pqFront.size() + pqBack.size());
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
     * @brief Runs a shortest path query between a source node s and a target node t
     *
     * @param s The source node
     * @param t The target node
     * @return The distance of the target node
     */
    DistType runQuery( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u, v, endNode;
        EdgeIterator e, endEdge;

        init( s, t);

        while( !(pqFront.empty() || pqBack.empty()))
        {
            if( (pqFront.minKey() + pqBack.minKey()) >= m_minDistance)
                break;

            searchForward();
            searchBackward();
        }

        if( m_viaNode != G.endNodes())
        {
            u = m_viaNode;
            t->dist = m_minDistance;

            while( u->succ != G.nilNodeDescriptor())
            {
                v = G.getNodeIterator(u->succ);
                v->pred = G.getNodeDescriptor( u);
                u = v;
            }
        
            return t->dist;
        }

        return std::numeric_limits<DistType>::max();
    }
    
 private:

    GraphType& G;
    unsigned int* m_timestamp;
    PriorityQueueType pqFront, pqBack;
    NodeIterator m_viaNode;
    DistType m_minDistance;
    unsigned int m_numSettledNodes;    

    /*
     * @brief Initializes the algorithm.
     * @param s The source node.
     * @param t The target node.
     **/
    void init( const NodeIterator& s, const NodeIterator& t)
    {
        resetTimestamps();

        m_viaNode = G.endNodes();

        m_minDistance = std::numeric_limits<DistType>::max();
        m_numSettledNodes = 0;

        pqFront.clear();
        pqBack.clear();
        ++(*m_timestamp);

        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();
        pqFront.insert( 0, s);

        t->distBack = 0;
        t->timestampBack = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();
        pqBack.insert( 0, t);
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
                u->timestamp = u->timestampBack = 0;
        }
    }


    /**
     * @brief Expands forward tree.
     **/
    void searchForward()
    {
        EdgeIterator e, endEdge;
        NodeIterator u, v;
        DistType discoveredDistance;
        
        u = pqFront.minItem();
        pqFront.popMin();
        ++m_numSettledNodes;

        for( e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
        {
            discoveredDistance = u->dist + e->weight;
            //discoveredDistance = u->dist + e->distance;
            v = G.target(e);

            //explored by the forward search
            if( v->timestamp < (*m_timestamp))
            {
                v->timestamp = (*m_timestamp);
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();

                pqFront.insert( v->dist, v);

                //meeting point with the backward search
                if( ( v->timestampBack == (*m_timestamp)) && ( discoveredDistance + v->distBack < m_minDistance))
                {
                    m_minDistance = discoveredDistance + v->distBack;
                    m_viaNode = v;
                }
            }

            //updated by the forward search
            else if(  v->dist > discoveredDistance)
            {
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();

                pqFront.insert( v->dist, v);

                //meeting point with the backward search
                if( ( v->timestampBack == (*m_timestamp)) && ( discoveredDistance + v->distBack < m_minDistance))
                {
                    m_minDistance = discoveredDistance + v->distBack;
                    m_viaNode = v;
                }
            }
        }
    }
    

    /**
     * @brief Expands backward tree.
     **/
    void searchBackward()
    {
        InEdgeIterator k, endInEdge;
        NodeIterator u, v;
        DistType discoveredDistance;
        
        u = pqBack.minItem();
        pqBack.popMin();
        ++m_numSettledNodes;

        for( k = G.beginInEdges(u), endInEdge = G.endInEdges(u); k != endInEdge; ++k)
        {
            discoveredDistance = u->distBack + k->weight;
            v = G.source(k);

            //explored by the backward search
            if( v->timestampBack < (*m_timestamp))
            {
                v->timestampBack = (*m_timestamp);
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();

                pqBack.insert( v->distBack, v);

                //meeting point with the forward search
                if( ( v->timestamp == (*m_timestamp)) && ( v->dist + discoveredDistance < m_minDistance))
                {
                    m_minDistance = v->dist + discoveredDistance;
                    m_viaNode = v;
                }
            }

            //updated by the backward search
            else if( v->distBack > discoveredDistance)
            {
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();

                pqBack.insert( v->distBack, v);

                //meeting point with the forward search
                if( ( v->timestamp == (*m_timestamp)) && ( v->dist + discoveredDistance < m_minDistance))
                {
                    m_minDistance = v->dist + discoveredDistance;
                    m_viaNode = v;
                }
            }
        }
    }
};

}

#endif//BIDIRECTIONALDIJKSTRA_H

