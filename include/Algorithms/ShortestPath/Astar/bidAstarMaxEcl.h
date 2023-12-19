#ifndef BIDASTARMAXECL_H
#define BIDASTARMAXECL_H

#include <Structs/Trees/priorityQueue.h>
#include <Utilities/geographic.h>


/**
 * @class BidAstarMaxEcl
 *
 * @brief The A* Variant of Bidirectional Dijkstra's algorithm using euclidean lower bound distances
 * and following the consistent-max approach.
 * This class supports running shortest path queries from a source node to target node.
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The distance domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType, class DistType=unsigned int>
class BidAstarMaxEcl
{

 public:

    typedef typename GraphType::NodeIterator                                   NodeIterator;
    typedef typename GraphType::NodeDescriptor                                 NodeDescriptor;
    typedef typename GraphType::EdgeIterator                                   EdgeIterator;
    typedef typename GraphType::InEdgeIterator                                 InEdgeIterator;
    typedef StaticNoDecPriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     * @param graph The graph to run the algorithm on.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not.
     */
    BidAstarMaxEcl( GraphType& graph, unsigned int* timestamp) : G(graph), m_timestamp(timestamp)
    {
        pqFront.reserve( G.getNumNodes());
        pqBack.reserve( G.getNumNodes());

		NodeIterator u, v, endNode;
        EdgeIterator e, endEdge;

        m_maxSpeed = std::numeric_limits<double>::min();

        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        {
            for( e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
            {
                v = G.target(e);
                double speed = euclideanDistance( u->x, u->y, v->x, v->y) / e->weight ; // TODO double(e->weight)
                if( speed > m_maxSpeed)
	    				m_maxSpeed = speed;
	    	}

        }

		//std::cout << "Max speed = " << m_maxSpeed << "\n"; 
    }


    /**
     * @brief Checks if the graph has feasible potentials.
     * @param t The target node.
     * @return True if the potentials are feasible, false otherwise.
     */
    bool hasFeasiblePotentials( const NodeIterator& t)
    {
        NodeIterator u,v,endNode;
        EdgeIterator e,endEdge;

        DistType potential_u, potential_v;

        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        {
            potential_u = LBdist(u, t);

            for( e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
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
    /* TODO
    bool hasConsistentPotentials()
    {
        NodeIterator u, endNode;
        EdgeIterator e, endEdge;

        //for any node v : P(s,v) + P(v,t) = constant 
        m_source = G.chooseNode();
        m_target = G.chooseNode();
        DistType offset = LBdist(m_source, m_target);

        const DistType constant = forwardLBdist(offset, m_source) + backwardLBdist(offset, m_source);

        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        {
            if( (forwardLBdist(offset, u) + backwardLBdist(offset, u)) != constant)
                return false;
        }
        
        return true;
    }*/


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
             
            if( (pqFront.minKey() + pqBack.minKey()) >= m_minReducedDistance)
            {
                break;
            }

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
    DistType m_minDistance, m_balance, m_minReducedDistance;
    unsigned int m_numSettledNodes; 
    NodeIterator m_source, m_target;
	double m_maxSpeed;


    /*
     * @brief Initializes the algorithm.
     * @param s The source node.
     * @param s The target node.
     **/
    void init( const NodeIterator& s, const NodeIterator& t)
    {
        resetTimestamps();

        m_source = s;
        m_target = t;

        m_viaNode = G.endNodes();

        m_minDistance = std::numeric_limits<DistType>::max();
        m_minReducedDistance = std::numeric_limits<DistType>::max();
        m_numSettledNodes = 0;

        //balance forward and backward search
        m_balance = LBdist(s, t);
        m_balance += ( m_balance >> 2 );

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
            v = G.target(e);

            //explored by the forward search
            if( v->timestamp < (*m_timestamp))
            {
                v->timestamp = (*m_timestamp);
                v->dist = discoveredDistance;
                v->key = forwardLBdist(discoveredDistance, v);
                v->pred = u->getDescriptor();

                pqFront.insert( v->key, v);

                //meeting point with the backward search
                if( ( v->timestampBack == (*m_timestamp)) && ( discoveredDistance + v->distBack < m_minDistance))
                {
                    m_minDistance = discoveredDistance + v->distBack;
                    m_minReducedDistance = m_minDistance + m_balance;
                    m_viaNode = v;
                }
            }

            //updated by the forward search
            else if(  v->dist > discoveredDistance)
            {
                v->key = ( v->key + discoveredDistance) - v->dist;
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();

                pqFront.insert( v->key, v);

                //meeting point with the backward search
                if( ( v->timestampBack == (*m_timestamp)) && ( discoveredDistance + v->distBack < m_minDistance))
                {
                    m_minDistance = discoveredDistance + v->distBack;
                    m_minReducedDistance = m_minDistance + m_balance;
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
                v->keyBack = backwardLBdist(discoveredDistance, v);
                v->succ = u->getDescriptor();

                pqBack.insert( v->keyBack, v);

                //meeting point with the forward search
                if( ( v->timestamp == (*m_timestamp)) && ( v->dist + discoveredDistance < m_minDistance))
                {
                    m_minDistance = v->dist + discoveredDistance;
                    m_minReducedDistance = m_minDistance + m_balance;
                    m_viaNode = v;
                }
            }

            //updated by the backward search
            else if( v->distBack > discoveredDistance)
            {
                v->keyBack = ( v->keyBack + discoveredDistance) - v->distBack;
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();

                pqBack.insert( v->keyBack, v);

                //meeting point with the forward search
                if( ( v->timestamp == (*m_timestamp)) && ( v->dist + discoveredDistance < m_minDistance))
                {
                    m_minDistance = v->dist + discoveredDistance;
                    m_minReducedDistance = m_minDistance + m_balance;
                    m_viaNode = v;
                }
            }
        }
    }


    inline DistType forwardLBdist( const DistType& discoveredDistance, const NodeIterator& v) const
    {
        //lower bound distance : P(v,t) = max(p(v,t), p(s,t) - p(s,v) + b)
        //DistType p1 = (discoveredDistance + LBdist(v, m_target)) - m_balance;
        //DistType p2 = (discoveredDistance + m_balance) - LBdist(m_source, v);

        DistType p1 = (discoveredDistance + LBdist(v, m_target));
        DistType p2 = (discoveredDistance + m_balance) - LBdist(m_source, v);

        //max
        return (((p1) > (p2)) ? (p1) : (p2));
    }


    inline DistType backwardLBdist( const DistType& discoveredDistance, const NodeIterator& v) const
    {
        //lower bound distance : P(s,v) = min(p(s,v), p(s,t) - p(v,t) + b)
        //DistType p1 = (discoveredDistance + LBdist(m_source, v)) - m_balance;
        //DistType p2 = (discoveredDistance + m_balance) - LBdist(v, m_target);

        DistType p1 = (discoveredDistance + LBdist(m_source, v));
        DistType p2 = (discoveredDistance + m_balance) - LBdist(v, m_target);

        //min
        return (((p1) < (p2)) ? (p1) : (p2));
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

#endif//BIDASTARMAXECL_H
