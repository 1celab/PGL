#ifndef BIDASTARAVELMK_H
#define BIDASTARAVELMK_H

#include <Structs/Trees/priorityQueue.h>
#include <Structs/Trees/priorityQueueNoDec.h>

#include <xmmintrin.h>
#include <emmintrin.h>
#include <smmintrin.h>

/**
 * @class BidAstarAveLmk
 *
 * @brief The A* Variant of Bidirectional Dijkstra's algorithm using landmark lower bound distances
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
class BidAstarAveLmk
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
    BidAstarAveLmk( GraphType& graph, unsigned int* timestamp, unsigned int numActiveLandmarks) :
    G(graph), m_timestamp(timestamp)
    {
        pqFront.reserve( G.getNumNodes());
        pqBack.reserve( G.getNumNodes());

        m_numLandmarks = G.chooseNode()->landmark.size();
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
    DistType m_minDistance;
    unsigned int m_numSettledNodes; 
    NodeIterator m_source, m_target; 

    //the total number of landmarks loaded in RAM memory
	unsigned int m_numLandmarks;

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
                    m_viaNode = v;
                }
            }

            //updated by the forward search
            else if( v->dist > discoveredDistance)
            {
                v->key = ( v->key + discoveredDistance) - v->dist;
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();

                pqFront.insert( v->key, v);

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
                v->keyBack = backwardLBdist(discoveredDistance, v);
                v->succ = u->getDescriptor();

                pqBack.insert( v->keyBack, v);

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
                v->keyBack = ( v->keyBack + discoveredDistance) - v->distBack;
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();

                pqBack.insert( v->keyBack, v);

                //meeting point with the forward search
                if( ( v->timestamp == (*m_timestamp)) && ( v->dist + discoveredDistance < m_minDistance))
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
    DistType LBdist( const NodeIterator& v, const NodeIterator& target) const
    {
        DistType tighestLowerBoundDistance = 0;

        __m128i max_vec =_mm_setzero_si128();

        for( unsigned int i=0, size = v->landmark.size(); i < size; i+=4)
        {
            __m128i* vL = (__m128i*) &( v->landmark[i]);
            __m128i* tL = (__m128i*) &( target->landmark[i]);
            __m128i res;

            //undirected case
            res = _mm_sub_epi32( *vL, *tL);
            res = _mm_abs_epi32( res); 
            max_vec = _mm_max_epi32( max_vec, res);
        }

	    unsigned int* maxV = (unsigned int *) &max_vec;

	    tighestLowerBoundDistance = maxV[0];
        for( unsigned int i=1, size = 4; i < size; i+=1)
        {
            if( tighestLowerBoundDistance < maxV[i])
                tighestLowerBoundDistance = maxV[i];
        }

        return tighestLowerBoundDistance;
    }
};

#endif//BIDASTARAVELMK_H

