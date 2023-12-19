#ifndef BIDASTARAVEECL_H
#define BIDASTARAVEECL_H

#include <Structs/Trees/priorityQueue.h>
#include <Utilities/geographic.h> 
#include <cmath>


/**
 * @class BidAstarAveEcl
 *
 * @brief The A* Variant of Bidirectional Dijkstra's algorithm using euclidean lower bound distances
 * and following the consistent-average approach.
 * This class supports running shortest path queries from a source node to target node.
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The distance domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType, class DistType=unsigned int>
class BidAstarAveEcl
{

 public:

    typedef NodeIterator                        NodeIterator;
    typedef NodeDescriptor                      NodeDescriptor;
    typedef EdgeIterator                        EdgeIterator;
    typedef InEdgeIterator                      InEdgeIterator;
    typedef PriorityQueue< DistType, NodeIterator, HeapStorage>     PriorityQueueType;
    
    /**
     * @brief Constructor
     * @param graph The graph to run the algorithm on.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not.
     */
    BidAstarAveEcl( GraphType& graph, unsigned int* timestamp) : G(graph), m_timestamp(timestamp)
    {
		NodeIterator u, v, lastNode;
        EdgeIterator e, lastEdge;

        m_maxSpeed = std::numeric_limits<double>::min();

        for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
        {
            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                v = G.target(e);
                double speed = euclideanDistance( u->x, u->y, v->x, v->y) / e->weight;
                if( speed > m_maxSpeed)
	    				m_maxSpeed = speed;
	    	}

        }

		std::cout << "Max speed = " << m_maxSpeed << "\n"; 
    }


    /**
     * @brief Checks if the graph has feasible potentials.
     * @param t The target node.
     * @return True if the potentials are feasible, false otherwise.
     */
    bool hasFeasiblePotentials( const NodeIterator& t)
    {
        NodeIterator u,v,lastNode;
        EdgeIterator e,lastEdge;

        DistType potential_u, potential_v;

        for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
        {
            potential_u = LBdist(u, t);

            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                v = G.target(e);
                
                potential_v = LBdist(v, t);
                //reducedCost = e->weight - potential_u + potential_v > 0
                if( (e->weight + potential_v) < potential_u )
                {
                    return false;
                }
            }
        }
        return true;
    }


    /**
     * @brief Checks if the graph has consistent potentials
     */
    bool hasConsistentPotentials()
    {
        NodeIterator u, lastNode;
        EdgeIterator e, lastEdge;

        //u = G.chooseNode();
        //for any node v : P(s,v) + P(v,t) = constant 
        const DistType constant = 0; // (LBdist(u, m_target) + LBdist(m_source, u))/2 - (LBdist(u, m_target) + LBdist(m_source, u))/2

        /* a trivial case because :  P(v,t) = ( p(v,t) - p(s,v) ) / 2   and  P(s,v) = ( p(s,v) - p(v,t) ) / 2 = -P(v,t)
        for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
        {
            if( ((LBdist(u, m_target) + LBdist(m_source, u))/2 - (LBdist(u, m_target) + LBdist(m_source, u))/2) != constant)
            return false;
        }
        */

        return true;
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
     * @brief Prints the shortest path from the source node to the target node.
     */
    void printShortestPath(const NodeIterator& s, const NodeIterator& t, 
                           const std::vector<NodeDescriptor>& ids) const
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


    void init( const NodeIterator& s, const NodeIterator& t)
    {
        m_source = s;
        m_target = t;

        m_viaNode = G.endNodes();

        m_minDistance = std::numeric_limits<DistType>::max();
        m_numSettledNodes = 0;

        pqFront.clear();
        pqBack.clear();
        ++(*m_timestamp);

        s->dist = 0;
        s->distBack = std::numeric_limits<DistType>::max();
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();
        pqFront.insert(0, s, &(s->pqitem));

        t->distBack = 0;
        t->dist = std::numeric_limits<DistType>::max();
        t->timestamp = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();
        pqBack.insert(0, t, &(t->pqitemBack));
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
        NodeIterator u, v, lastNode;
        EdgeIterator e, lastEdge;
    
        init( s, t);  

        while( !(pqFront.empty() || pqBack.empty()))
        {
            if( (pqFront.minKey() + pqBack.minKey()) >= m_minDistance)
            {
                break;
            }

            searchForward();
            searchBackward();
        }

        if(m_viaNode != G.endNodes())
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
    NodeIterator m_source, m_target;
	double m_maxSpeed;

    bool isBackwardFound( const NodeIterator& u)
    {
        return (u->timestamp == (*m_timestamp)) && (u->distBack != std::numeric_limits<DistType>::max());
    }
    

    bool isBackwardSettled( const NodeIterator& u)
    {
        return isBackwardFound(u) && (!isInBackQueue(u));
    }
    

    bool isForwardFound( const NodeIterator& u)
    {
        return (u->timestamp == (*m_timestamp)) && (u->dist != std::numeric_limits<DistType>::max());
    }
    

    bool isForwardSettled( const NodeIterator& u)
    {
        return isForwardFound(u) && (!isInFrontQueue(u));
    }
    

    bool isInBackQueue( const NodeIterator& u)
    {
        return pqBack.contains( &(u->pqitemBack));
    }
    

    bool isInFrontQueue( const NodeIterator& u)
    {
        return pqFront.contains( &(u->pqitem));
    }
    

    void searchForward()
    {
        EdgeIterator e, lastEdge;
        NodeIterator u, v;
        DistType discoveredDistance;
        
        u = pqFront.minItem();
        pqFront.popMin();
        ++m_numSettledNodes;

        for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
        {
            discoveredDistance = u->dist + e->weight;
            v = G.target(e);

            //unexplored by both the forward and backward search
            if( v->timestamp < (*m_timestamp))
            {
                v->timestamp = (*m_timestamp);
                v->distBack = std::numeric_limits<DistType>::max();
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();
                pqFront.insert( forwardLBdist(discoveredDistance, v), v, &(v->pqitem));
            }

            //already labeled or settled by the forward search
            else if( v->dist <= discoveredDistance )
                continue;

            //updated by the forward search
            else if( v->dist != std::numeric_limits<DistType>::max())
            {
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();
                pqFront.decrease( forwardLBdist(discoveredDistance, v), &(v->pqitem));

                if( ( v->distBack != std::numeric_limits<DistType>::max()) && ( discoveredDistance + v->distBack < m_minDistance))
                {
                    m_minDistance = discoveredDistance + v->distBack;
                    m_viaNode = v;
                }
            }

            //unexplored by the forward and explored by the backward search (rare - meeting point)
            else
            {
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();
                pqFront.insert( forwardLBdist(discoveredDistance, v), v, &(v->pqitem));

                if(discoveredDistance + v->distBack < m_minDistance)
                {
                    m_minDistance = discoveredDistance + v->distBack;
                    m_viaNode = v;
                }
            }
        }
    }
    

    void searchBackward()
    {
        InEdgeIterator k, lastInEdge;
        NodeIterator u, v;
        DistType discoveredDistance;
        
        u = pqBack.minItem();
        pqBack.popMin();
        ++m_numSettledNodes;

        for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
        {
            discoveredDistance = u->distBack + k->weight;
            v = G.source(k);

            //unexplored by both the forward and backward search
            if( v->timestamp < (*m_timestamp))
            {
                v->timestamp = (*m_timestamp);
                v->dist = std::numeric_limits<DistType>::max();
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();
                pqBack.insert( backwardLBdist(discoveredDistance, v), v, &(v->pqitemBack));
            }

            else if( v->distBack <= discoveredDistance)
                continue;

            else if( v->distBack != std::numeric_limits<DistType>::max())
            {
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();
                pqBack.decrease( backwardLBdist(discoveredDistance, v), &(v->pqitemBack));

                if( ( v->dist != std::numeric_limits<DistType>::max()) && ( v->dist + discoveredDistance < m_minDistance))
                {
                    m_minDistance = v->dist + discoveredDistance;
                    m_viaNode = v;
                }
            }

            //unexplored by the backward search
            else
            {
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();
                pqBack.insert( backwardLBdist(discoveredDistance, v), v, &(v->pqitemBack));

                if( v->dist + discoveredDistance < m_minDistance)
                {
                    m_minDistance = v->dist + discoveredDistance;
                    m_viaNode = v;
                }
            }
        }
    }
    

    inline DistType forwardLBdist( const DistType& discoveredDistance, const NodeIterator& v) const
    {
        return ( ( ( ( discoveredDistance << 1) + LBdist(v, m_target)) - LBdist(m_source, v)) >> 1) ;
    }


    inline DistType backwardLBdist( const DistType& discoveredDistance, const NodeIterator& v) const
    {
        return ( ( ( ( discoveredDistance << 1) + LBdist(m_source, v)) - LBdist(v, m_target)) >> 1) ;
    }
    

    /**
    * @brief Computes a lower bound distance between two nodes. 
    * @param The source node.
    * @param The sink node.
    */
    inline DistType LBdist(const NodeIterator& v, const NodeIterator& t) const
    {
        return euclideanDistance( v->x, v->y, t->x, t->y) / m_maxSpeed;
    }
    
};

#endif//BIDASTARAVEECL_H
