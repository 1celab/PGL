#ifndef UNIASTARECL_H
#define UNIASTARECL_H

#include <Structs/Trees/priorityQueue.h>
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

    typedef NodeIterator                        NodeIterator;
    typedef NodeDescriptor                      NodeDescriptor;
    typedef EdgeIterator                        EdgeIterator;
    typedef PriorityQueue<DistType, NodeIterator, HeapStorage>      PriorityQueueType;
    
    /**
     * @brief Constructor
     * @param graph The graph to run the algorithm on.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not.
     */
    UniAstarEcl( GraphType& graph, unsigned int* timestamp) : G(graph), m_timestamp(timestamp)
    {
		NodeIterator u, v, lastNode;
        EdgeIterator e, lastEdge;

        m_maxSpeed = std::numeric_limits<double>::min();

        for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
        {
            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
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
                std::cout << "(v,u)=(" << G.getRelativePosition(v) << ", " << G.getRelativePosition(u) << ")";
                std::cout << "\nweight=" << e->weight;
                std::cout << "\nu->v=" << LBdist(u, v);
                std::cout << "\nu->t=" << potential_u;
                std::cout << "\nv->t=" << potential_v;
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
        //assert(hasFeasiblePotentials(t));
     
        pq.clear();
        ++(*m_timestamp);

        m_numSettledNodes = 0;
        t->dist = std::numeric_limits<DistType>::max();

        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();
        pq.insert( s->dist, s, &(s->pqitem));
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
        EdgeIterator e,lastEdge;
        DistType discoveredDistance;
        
        init( s, t);

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_numSettledNodes;

            if( u == t) break;

            for(e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                discoveredDistance = u->dist + e->weight;

                v = G.target(e); 

                if( v->timestamp < (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->timestamp = (*m_timestamp);
                    v->dist = discoveredDistance;

                    pq.insert( discoveredDistance + LBdist(v, t), v, &(v->pqitem));
                    
                }

                else if( v->dist > discoveredDistance )
                {
                    v->pred = u->getDescriptor();
                    v->dist = discoveredDistance;

                    pq.decrease( discoveredDistance + LBdist(v, t), &(v->pqitem));
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

