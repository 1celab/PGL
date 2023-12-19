#ifndef UNIASTARECL_H
#define UNIASTARECL_H

#include <Structs/Trees/priorityQueue.h>
#include <Structs/Trees/priorityQueueNoDec.h>
#include <Utilities/geographic.h>


/**
 * @class UniAstarEcl
 *
 * @brief The A* Variant of Dijkstra's algorithm using euclidean distances.
 * This class supports running shortest path queries from a source node to target node.
 *
 * @tparam GraphType The type of the graph to run the algorithm on.
 * @tparam DistType The distance domain.
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType, class DistType=unsigned int>
class UniAstarEcl
{

 public:

    typedef typename GraphType::NodeIterator                                   NodeIterator;
    typedef typename GraphType::NodeDescriptor                                 NodeDescriptor;
    typedef typename GraphType::EdgeIterator                                   EdgeIterator;
    typedef StaticNoDecPriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     * @param graph The graph to run the algorithm on.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not.
     */
    UniAstarEcl( GraphType& graph, unsigned int* timestamp) : G(graph), m_timestamp(timestamp)
    {
        pq.reserve( G.getNumNodes());

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
                    /*std::cout << "(v,u)=(" << G.getRelativePosition(v) << ", " << G.getRelativePosition(u) << ")";
                    std::cout << "\nweight=" << e->weight;
                    std::cout << "\nu->v=" << LBdist(u, v);
                    std::cout << "\nu->t=" << potential_u;
                    std::cout << "\nv->t=" << potential_v;*/
                    return false;
                }
            }
        }

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


    /*
     * @brief Initializes the algorithm.
     * @param s The source node.
     * @param s The target node.
     **/
    void init( const NodeIterator& s, const NodeIterator& t)
    {
        resetTimestamps();

        pq.clear();
        ++(*m_timestamp);

        m_numSettledNodes = 0;
        t->dist = std::numeric_limits<DistType>::max();

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


    /**
     * @brief Runs a shortest path query between a source node s and a target node t
     *
     * @param s The source node
     * @param t The target node
     * @return The distance of the target node
     */
    DistType runQuery( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u,v;
        EdgeIterator e,endEdge;
        DistType discoveredDistance;
        
        init( s, t);

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_numSettledNodes;

            if( u == t) break;

            for(e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
            {
                discoveredDistance = u->dist + e->weight;

                v = G.target(e); 

                if( v->timestamp < (*m_timestamp))
                {
                    v->timestamp = (*m_timestamp);
                    v->dist = discoveredDistance;
                    v->key = discoveredDistance + LBdist( v, t);
                    v->pred = u->getDescriptor();

                    pq.insert( v->key, v);
                }

                else if( v->dist > discoveredDistance)
                {
                    v->key = ( v->key + discoveredDistance) - v->dist;
                    v->dist = discoveredDistance;
                    v->pred = u->getDescriptor();

                    pq.insert( v->key, v);
                }
            }
        }

        return t->dist;
    }
    
 private:

    GraphType& G;
    unsigned int* m_timestamp;
    PriorityQueueType pq;
    unsigned int m_numSettledNodes;
	double m_maxSpeed;

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

#endif//UNIASTARECL_H

