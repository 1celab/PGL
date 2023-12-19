#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <Structs/Trees/priorityQueue.h>


/**
 * @class Dijkstra
 *
 * @brief Plain Dijkstra algorithm implementation
 *
 * This class supports building a full shortest path tree from a source node s, or running queries between source and target nodes
 *
 * @tparam GraphType The type of the graph to run the algorithm on
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType>
class Dijkstra
{

public:
    typedef typename GraphType::NodeIterator                              NodeIterator;
    typedef typename GraphType::EdgeIterator                              EdgeIterator;
    typedef typename GraphType::SizeType                                  SizeType;
    typedef unsigned int                                                  WeightType;
    typedef StaticPriorityQueue< WeightType, NodeIterator, HeapStorage>   PriorityQueueType;
    
    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    Dijkstra( GraphType& graph, unsigned int* timestamp):G(graph),m_timestamp(timestamp)
    {
        pq.reserve( G.getNumNodes());
    }
    

   /**
     * @brief Returns the number of the nodes that have have been settled (enqueued and dequeued) by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int& getNumSettledNodes()
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


    const unsigned int getNumUnvisitedNodes()
    {
        return ( G.getNumNodes() - getNumVisitedNodes());
    }


    const unsigned int getNumSPNodes( const typename GraphType::NodeIterator& t)
    {
        NodeIterator u = t;
        unsigned int numSPNodes = 1;
        //s -- v -- t 
        //1 + the rest nodes in sp
        while( u->pred != G.nilNodeDescriptor())
        {
            numSPNodes++;
            u = G.getNodeIterator(u->pred);
        }

        return numSPNodes;
    }


    /**
     * @brief Prints the shortest path from the source node to the target node.
     */
    void printShortestPath(const NodeIterator& s, const NodeIterator& t, 
                           const std::vector<typename GraphType::NodeDescriptor>& ids) const
    {
        NodeIterator u = t;
        unsigned int numSpNodes = 1;

        std::cout << "\nSp edges :\n";

        std::list<unsigned int> sp;

        while(u->pred != G.nilNodeDescriptor())
        {
            for(unsigned int j = 0; j < ids.size(); j++)
                if(G.getNodeDescriptor(u) == ids[j])
                {
                    sp.push_front(j);
                    break;
                }

            numSpNodes++;
            u = G.getNodeIterator(u->pred);
        }

        for(unsigned int j = 0; j < ids.size(); j++)
            if(G.getNodeDescriptor(s) == ids[j])
            {
                sp.push_front(j);
                break;
            }

        for (std::list<unsigned int>::iterator it = sp.begin(); it != sp.end(); ++it)
            std::cout << "[" << *it << "]->";
        std::cout << "[-]";

        std::cout << "\n\nSp length : " << t->dist << "\n";
        std::cout << "\nSp num nodes : " << numSpNodes << "\n";
    }


    void init( const typename GraphType::NodeIterator& s)
    {
        pq.clear();
        m_numSettledNodes = 0;
        ++(*m_timestamp);

        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();

        pq.insert( 0, s, &(s->pqitem));
    }


    /**
     * @brief Builds a shortest path tree routed on a source node
     *
     * @param s The source node
     */
    void buildTree( const typename GraphType::NodeIterator& s)
    {
        NodeIterator u, v, lastNode;
        EdgeIterator e, lastEdge;
        WeightType discoveredDistance;
        
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
     * @brief Runs a shortest path query between a source node s and a target node t
     *
     * @param s The source node
     * @param t The target node
     * @return The distance of the target node
     */
    WeightType runQuery( const typename GraphType::NodeIterator& s, const typename GraphType::NodeIterator& t)
    {
        NodeIterator u, v, lastNode;
        EdgeIterator e, lastEdge;
        WeightType discoveredDistance;

        init( s);

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
                    pq.insert( discoveredDistance, v, &(v->pqitem));
                    
                }

                else if( v->dist > discoveredDistance)
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;
                    pq.decrease( discoveredDistance, &(v->pqitem));
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
};



/**
 * @class BackwardDijkstra
 *
 * @brief Plain Dijkstra algorithm backwards implementation
 *
 * This class supports building a full shortest path tree towards a target node t, or running queries between source and target nodes
 *
 * @tparam GraphType The type of the graph to run the algorithm on
 * @author Panos Michail
 *
 */

template<class GraphType>
class BackwardDijkstra
{
public:
    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef typename GraphType::InEdgeIterator                    InEdgeIterator;
    typedef typename GraphType::SizeType                            SizeType;
    typedef unsigned int                                            WeightType;
    typedef PriorityQueue< WeightType, NodeIterator, HeapStorage>   PriorityQueueType;
    
    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    BackwardDijkstra( GraphType& graph, unsigned int* timestamp):G(graph),m_timestamp(timestamp)
    {
    }
    
    /**
     * @brief Builds a backwards shortest path tree routed on a target node
     *
     * @param t The target node
     */
    void buildTree( const typename GraphType::NodeIterator& t)
    {
        NodeIterator u,v,lastNode;
        InEdgeIterator k,lastInEdge;
        
        pqBack.clear();
        ++(*m_timestamp);

        t->distBack = 0;
        t->timestamp = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();;

        pqBack.insert( t->distBack, t, &(t->pqitemBack));

        while( !pqBack.empty())
        {
            u = pqBack.minItem();
            pqBack.popMin();
            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                v = G.source(k);

                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = u->distBack + k->weight;
                    v->timestamp = (*m_timestamp);
                    pqBack.insert( v->distBack, v, &(v->pqitemBack));
                }
                else if( v->distBack > u->distBack + k->weight )
                {
                    v->succ = u->getDescriptor();
                    v->distBack = u->distBack + k->weight;
                    pqBack.decrease( v->distBack, &(v->pqitemBack));
                }
            }
        }
    }
    
    /**
     * @brief Runs a shortest path query between a source node s and a target node t
     *
     * @param s The source node
     * @param t The target node
     * @return The distance of the target node
     */
    WeightType runQuery( const typename GraphType::NodeIterator& s, const typename GraphType::NodeIterator& t)
    {
        NodeIterator u,v,lastNode;
        InEdgeIterator k,lastInEdge;
        
        pqBack.clear();
        ++(*m_timestamp);

        t->distBack = 0;
        t->timestamp = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();;

        pqBack.insert( t->distBack, t, &(t->pqitemBack));

        while( !pqBack.empty())
        {
            u = pqBack.minItem();
            pqBack.popMin();
            if( u == s) break;
            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                v = G.source(k);

                if( v->timestamp < (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = u->distBack + k->weight;
                    v->timestamp = (*m_timestamp);
                    pqBack.insert( v->distBack, v, &(v->pqitemBack));
                }
                else if( v->distBack > u->distBack + k->weight )
                {
                    v->succ = u->getDescriptor();
                    v->distBack = u->distBack + k->weight;
                    pqBack.decrease( v->distBack, &(v->pqitemBack));
                }
            }
        }
        
        u = s;
        t->dist = 0;
        EdgeIterator e;
        while( u->succ != G.nilNodeDescriptor())
        {
            v = G.getNodeIterator( u->succ);
            e = G.getEdgeIterator( u, v);
            t->dist += e->weight;
            v->pred = G.getNodeDescriptor(u);
            u = v;
        }
        return t->dist;
    }
    
private:
    GraphType& G;
    unsigned int* m_timestamp;
    PriorityQueueType pqBack;
};

#endif//DIJKSTRA_H

