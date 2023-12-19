#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <Structs/Trees/priorityQueue.h>
#include <Structs/Trees/priorityQueueNoDec.h>

/**
 * @class Dijkstra
 *
 * @brief Regular Dijkstra's algorithm implementation.
 * This class supports building a full shortest path tree from a source node s, or running queries between source and target nodes
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
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
                    pq.insert( v->dist, v, &(v->pqitem));
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( v->dist, &(v->pqitem));
                }
            }
        }
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
                    pq.insert( v->dist, v, &(v->pqitem));
                    
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( v->dist, &(v->pqitem));
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
        pq.insert( 0, s, &(s->pqitem));
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



/**
 * @class BackwardDijkstra
 *
 * @brief Regular Backward Dijkstra's algorithm implementation.
 * This class supports building a full reverse shortest path tree from a target node t, 
 * or running queries between source and target nodes
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The edge weight domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType, class DistType=unsigned int>
class BackwardDijkstra
{

 public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::InEdgeIterator                      InEdgeIterator;
    typedef PriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not.
     */
    BackwardDijkstra( GraphType& graph, unsigned int* timestamp): G(graph), m_timestamp(timestamp)
    {
    }


    /**
     * @brief Returns the number of the nodes which have have been settled (enqueued and dequeued) by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int getNumSettledNodes() const
    {
        return m_numSettledNodes;
    }


    /**
     * @brief Returns the number of the nodes which have have been visited (enqueued) by the algorithm.
     * @return The number of the explored nodes.
     */
    const unsigned int getNumVisitedNodes()
    {
        return ( m_numSettledNodes + pqBack.size());
    }


    /**
     * @brief Returns the number of nodes in the discovered shortest v-t path.
     * @param v A source node.
     * @return The number of nodes in shortest v-t path.
     **/
    unsigned int getNumSPNodes( NodeIterator v) const
    {
        unsigned int numSPNodes = 1;

        while( G.hasNode( (NodeDescriptor) v->succ))
        {
            numSPNodes++;
            v = G.getNodeIterator( v->succ);
        }

        return numSPNodes;
    }


    /**
     * @brief Builds a backwards shortest path tree routed on a target node.
     * @param t The target node.
     */
    void buildTree( const NodeIterator& t)
    {
        NodeIterator u, v, endNode;
        InEdgeIterator k, lastInEdge;
        DistType discoveredDistance;
        
        init( t);

        while( !pqBack.empty())
        {
            u = pqBack.minItem();
            pqBack.popMin();
            ++m_numSettledNodes;

            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                discoveredDistance = u->distBack + k->weight;
                v = G.source(k);
                
                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pqBack.insert( v->distBack, v, &(v->pqitemBack));
                }

                else if( v->distBack > discoveredDistance)
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    pqBack.decrease( v->distBack, &(v->pqitemBack));
                }
            }
        }
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
        InEdgeIterator k, lastInEdge;
        DistType discoveredDistance;
        
        init( t);
        s->distBack = std::numeric_limits<DistType>::max();

        while( !pqBack.empty())
        {
            u = pqBack.minItem();
            pqBack.popMin();
            ++m_numSettledNodes;

            if( u == s) break;

            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                discoveredDistance = u->distBack + k->weight;
                v = G.source(k);
                
                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pqBack.insert( v->distBack, v, &(v->pqitemBack));
                }

                else if( v->distBack > discoveredDistance)
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    pqBack.decrease( v->distBack, &(v->pqitemBack));
                }
            }
        }
        
        return s->distBack;
    }
    
 private:

    GraphType& G;
    unsigned int* m_timestamp;
    PriorityQueueType pqBack;
    unsigned int m_numSettledNodes;

    /**
     * @brief Initliazes the algorithm.
     * @param t The target node.
     **/
    void init( const NodeIterator& t)
    {
        //reset the timestamp of each node in graph
        resetTimestamps();

        pqBack.clear();
        ++(*m_timestamp);

        m_numSettledNodes = 0;

        t->distBack = 0;
        t->timestamp = (*m_timestamp);
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
                u->timestamp = 0;
        }
    }
};


/**
 * @class DualDijkstra
 *
 * @brief Regular Forward and Backward Dijkstra's algorithm implementation.
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The distance domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType, class DistType=unsigned int>
class DualDijkstra
{

 public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef typename GraphType::InEdgeIterator                      InEdgeIterator;
    typedef PriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    DualDijkstra( GraphType& graph, unsigned int* timestamp): G(graph), m_timestamp(timestamp)
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
                    pq.insert( v->dist, v, &(v->pqitem));
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( v->dist, &(v->pqitem));
                }
            }
        }
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
                    pq.insert( v->dist, v, &(v->pqitem));
                    
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( v->dist, &(v->pqitem));
                }
            }
        }

        return t->dist;
    }

    /**
     * @brief Builds a backwards shortest path tree routed on a target node.
     * @param t The target node.
     */
    void buildRevTree( const NodeIterator& t)
    {
        NodeIterator u, v, endNode;
        InEdgeIterator k, lastInEdge;
        DistType discoveredDistance;
        
        initRev( t);

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_numSettledNodes;

            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                discoveredDistance = u->distBack + k->weight;
                v = G.source(k);
                
                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pq.insert( v->distBack, v, &(v->pqitemBack));
                }

                else if( v->distBack > discoveredDistance)
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    pq.decrease( v->distBack, &(v->pqitemBack));
                }
            }
        }
    }
    
    /**
     * @brief Runs a shortest path query between a source node s and a target node t.
     * @param s The source node.
     * @param t The target node.
     * @return The distance of the target node.
     */
    DistType runRevQuery( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u, v, endNode;
        InEdgeIterator k, lastInEdge;
        DistType discoveredDistance;
        
        initRev( t);
        s->distBack = std::numeric_limits<DistType>::max();

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_numSettledNodes;

            if( u == s) break;

            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                discoveredDistance = u->distBack + k->weight;
                v = G.source(k);
                
                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pq.insert( v->distBack, v, &(v->pqitemBack));
                }

                else if( v->distBack > discoveredDistance)
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    pq.decrease( v->distBack, &(v->pqitemBack));
                }
            }
        }
        
        return s->distBack;
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
        pq.insert( 0, s, &(s->pqitem));
    }

    /**
     * @brief Initliazes the algorithm.
     * @param t The target node.
     **/
    void initRev( const NodeIterator& t)
    {
        //reset the timestamp of each node in graph
        resetTimestamps();

        pq.clear();
        ++(*m_timestamp);

        m_numSettledNodes = 0;

        t->distBack = 0;
        t->timestamp = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();
        pq.insert( 0, t, &(t->pqitemBack));
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


namespace StcNoDec
{

/**
 * @class Dijkstra
 *
 * @brief Regular Dijkstra's algorithm implementation.
 * This class supports building a full shortest path tree from a source node s, or running queries between source and target nodes
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The distance domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType, class DistType=unsigned int>
class Dijkstra
{

 public:

    typedef typename GraphType::NodeIterator                                   NodeIterator;
    typedef typename GraphType::NodeDescriptor                                 NodeDescriptor;
    typedef typename GraphType::EdgeIterator                                   EdgeIterator;
    typedef StaticNoDecPriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    Dijkstra( GraphType& graph, unsigned int* timestamp): G(graph), m_timestamp(timestamp)
    {
        pq.reserve( G.getNumNodes());
    }
    

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
                    pq.insert( v->dist, v);
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.insert( v->dist, v);
                }
            }
        }
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
                    pq.insert( v->dist, v);
                    
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.insert( v->dist, v);
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
        pq.insert( 0, s);
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



/**
 * @class BackwardDijkstra
 *
 * @brief Regular Backward Dijkstra's algorithm implementation.
 * This class supports building a full reverse shortest path tree from a target node t, 
 * or running queries between source and target nodes
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The edge weight domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType, class DistType=unsigned int>
class BackwardDijkstra
{

 public:

    typedef typename GraphType::NodeIterator                                   NodeIterator;
    typedef typename GraphType::NodeDescriptor                                 NodeDescriptor;
    typedef typename GraphType::InEdgeIterator                                 InEdgeIterator;
    typedef StaticNoDecPriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not.
     */
    BackwardDijkstra( GraphType& graph, unsigned int* timestamp): G(graph), m_timestamp(timestamp)
    {
        pqBack.reserve( G.getNumNodes());
    }


    /**
     * @brief Returns the number of the nodes which have have been settled (enqueued and dequeued) by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int getNumSettledNodes() const
    {
        return m_numSettledNodes;
    }


    /**
     * @brief Returns the number of the nodes which have have been visited (enqueued) by the algorithm.
     * @return The number of the explored nodes.
     */
    const unsigned int getNumVisitedNodes()
    {
        return ( m_numSettledNodes + pqBack.size());
    }


    /**
     * @brief Returns the number of nodes in the discovered shortest v-t path.
     * @param v A source node.
     * @return The number of nodes in shortest v-t path.
     **/
    unsigned int getNumSPNodes( NodeIterator v) const
    {
        unsigned int numSPNodes = 1;

        while( G.hasNode( (NodeDescriptor) v->succ))
        {
            numSPNodes++;
            v = G.getNodeIterator( v->succ);
        }

        return numSPNodes;
    }


    /**
     * @brief Builds a backwards shortest path tree routed on a target node.
     * @param t The target node.
     */
    void buildTree( const NodeIterator& t)
    {
        NodeIterator u, v, endNode;
        InEdgeIterator k, lastInEdge;
        DistType discoveredDistance;
        
        init( t);

        while( !pqBack.empty())
        {
            u = pqBack.minItem();
            pqBack.popMin();
            ++m_numSettledNodes;

            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                discoveredDistance = u->distBack + k->weight;
                v = G.source(k);
                
                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pqBack.insert( v->distBack, v);
                }

                else if( v->distBack > discoveredDistance)
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    pqBack.insert( v->distBack, v);
                }
            }
        }
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
        InEdgeIterator k, lastInEdge;
        DistType discoveredDistance;
        
        init( t);
        s->distBack = std::numeric_limits<DistType>::max();

        while( !pqBack.empty())
        {
            u = pqBack.minItem();
            pqBack.popMin();
            ++m_numSettledNodes;

            if( u == s) break;

            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                discoveredDistance = u->distBack + k->weight;
                v = G.source(k);
                
                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pqBack.insert( v->distBack, v);
                }

                else if( v->distBack > discoveredDistance)
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    pqBack.insert( v->distBack, v);
                }
            }
        }
        
        return s->distBack;
    }
    
 private:

    GraphType& G;
    unsigned int* m_timestamp;
    PriorityQueueType pqBack;
    unsigned int m_numSettledNodes;

    /**
     * @brief Initliazes the algorithm.
     * @param t The target node.
     **/
    void init( const NodeIterator& t)
    {
        //reset the timestamp of each node in graph
        resetTimestamps();

        pqBack.clear();
        ++(*m_timestamp);

        m_numSettledNodes = 0;

        t->distBack = 0;
        t->timestamp = (*m_timestamp);
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
                u->timestamp = 0;
        }
    }
};


/**
 * @class DualDijkstra
 *
 * @brief Regular Forward and Backward Dijkstra's algorithm implementation.
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The distance domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType, class DistType=unsigned int>
class DualDijkstra
{

 public:

    typedef typename GraphType::NodeIterator                                   NodeIterator;
    typedef typename GraphType::NodeDescriptor                                 NodeDescriptor;
    typedef typename GraphType::EdgeIterator                                   EdgeIterator;
    typedef typename GraphType::InEdgeIterator                                 InEdgeIterator;
    typedef StaticNoDecPriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    DualDijkstra( GraphType& graph, unsigned int* timestamp): G(graph), m_timestamp(timestamp)
    {
        pq.reserve( G.getNumNodes());
    }
    

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
                    pq.insert( v->dist, v);
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.insert( v->dist, v);
                }
            }
        }
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
                    pq.insert( v->dist, v);
                    
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.insert( v->dist, v);
                }
            }
        }

        return t->dist;
    }


    /**
     * @brief Builds a backwards shortest path tree routed on a target node.
     * @param t The target node.
     */
    void buildRevTree( const NodeIterator& t)
    {
        NodeIterator u, v, endNode;
        InEdgeIterator k, lastInEdge;
        DistType discoveredDistance;
        
        initRev( t);

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_numSettledNodes;

            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                discoveredDistance = u->distBack + k->weight;
                v = G.source(k);
                
                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pq.insert( v->distBack, v);
                }

                else if( v->distBack > discoveredDistance)
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    pq.insert( v->distBack, v);
                }
            }
        }
    }
    

    /**
     * @brief Runs a shortest path query between a source node s and a target node t.
     * @param s The source node.
     * @param t The target node.
     * @return The distance of the target node.
     */
    DistType runRevQuery( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u, v, endNode;
        InEdgeIterator k, lastInEdge;
        DistType discoveredDistance;
        
        initRev( t);
        s->distBack = std::numeric_limits<DistType>::max();

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_numSettledNodes;

            if( u == s) break;

            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                discoveredDistance = u->distBack + k->weight;
                v = G.source(k);
                
                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    v->timestamp = (*m_timestamp);
                    pq.insert( v->distBack, v);
                }

                else if( v->distBack > discoveredDistance)
                {
                    v->succ = u->getDescriptor();
                    v->distBack = discoveredDistance;
                    pq.insert( v->distBack, v);
                }
            }
        }
        
        return s->distBack;
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
        pq.insert( 0, s);
    }


    /**
     * @brief Initliazes the algorithm.
     * @param t The target node.
     **/
    void initRev( const NodeIterator& t)
    {
        //reset the timestamp of each node in graph
        resetTimestamps();

        pq.clear();
        ++(*m_timestamp);

        m_numSettledNodes = 0;

        t->distBack = 0;
        t->timestamp = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();
        pq.insert( 0, t);
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

}

#endif//DIJKSTRA_H
