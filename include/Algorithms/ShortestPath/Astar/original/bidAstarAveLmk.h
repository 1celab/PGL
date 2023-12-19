#ifndef BIDASTARAVELMK_H
#define BIDASTARAVELMK_H

#include <Structs/Trees/priorityQueueNoDec.h>
#include <list>
#include <vector>
#include <xmmintrin.h>
#include <emmintrin.h>
#include <smmintrin.h>


#include <Algorithms/ShortestPath/Astar/MaxHeapPriorityQueue.h>


/**
 * @class BidAstarAveLmk
 *
 * @brief The A* Variant of Bidirectional Dijkstra's algorithm using landmark lower bound distances
 * and following the consistent-average approach.
 *
 * This class supports running queries between source and target nodes
 *
 * @tparam GraphType The type of the graph to run the algorithm on
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType>
class BidAstarAveLmk
{

 public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef typename GraphType::InEdgeIterator                      InEdgeIterator;
    typedef typename GraphType::SizeType                            SizeType;
    typedef unsigned int                                            WeightType;
    typedef StaticNoDecPriorityQueue< WeightType, NodeIterator, HeapStorage>   PriorityQueueType;
    typedef MaxHeapPQ<unsigned short, WeightType> LandmarkMaxHeap;


    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    BidAstarAveLmk( GraphType& graph, unsigned int* timestamp, unsigned int numActiveLandmarks) : G(graph), m_timestamp(timestamp),        
                                                                                                   m_numActiveLandmarks(numActiveLandmarks)
    {
        pqFront.reserve( G.getNumNodes());
        pqBack.reserve( G.getNumNodes());

        m_numLandmarks = G.chooseNode()->landmark.size();
        m_landmarkHeap.resize(m_numLandmarks);

        if( m_numActiveLandmarks <= 0 || m_numActiveLandmarks > m_numLandmarks)
            m_numActiveLandmarks = m_numLandmarks;

        m_activeLandmark.resize(m_numActiveLandmarks);

        //set the active landmarks
        for(unsigned int i = 0; i < m_numActiveLandmarks; i++)
        {
           m_activeLandmark[i] = m_landmarkHeap.dataOfMax();
           m_landmarkHeap.pop();
        }
    }


    /**
     * @brief Checks if the graph has feasible potentials
     *
     * @param t The target node
     * @return True if the potentials are feasible, false otherwise
     */
    bool hasFeasiblePotentials( const typename GraphType::NodeIterator& t)
    {
        NodeIterator u,v,lastNode;
        EdgeIterator e,lastEdge;

        WeightType potential_u, potential_v;

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
     * @brief Returns the number of the nodes that have have been settled (enqueued and dequeued) by the algorithm.
     * @return The number of the settled nodes.
     */
    const unsigned int& getNumSettledNodes()
    {
        return m_numSettledNodes;
    }


    const unsigned int getNumVisitedNodes()
    {
        // each node that was visited by both the forward and backward search counts for 2 visits
        return ( m_numSettledNodes + pqFront.size() + pqBack.size());
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


    void init( const typename GraphType::NodeIterator& s, const typename GraphType::NodeIterator& t)
    {
        m_source = s;
        m_target = t;

        m_viaNode = G.endNodes();

        m_minDistance = std::numeric_limits<WeightType>::max();
        m_numSettledNodes = 0;

        pqFront.clear();
        pqBack.clear();
        ++(*m_timestamp);

        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();
        pqFront.insert(0, s);

        t->distBack = 0;
        t->timestampBack = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();
        pqBack.insert(0, t);

        /*m_landmarkHeap.reset();

        //choose the best landmarks for the current query
        for(unsigned int i=0; i<m_numLandmarks; i++)
        {
            WeightType estimatedDistance = 0;

            if( (s->landmark[i].distanceToLandmark) > (t->landmark[i].distanceToLandmark) )
                estimatedDistance = (s->landmark[i].distanceToLandmark) - (t->landmark[i].distanceToLandmark);
            else
                estimatedDistance = (t->landmark[i].distanceFromLandmark) - (s->landmark[i].distanceFromLandmark);

            m_landmarkHeap.push(estimatedDistance, i);
        }

        //set the active landmarks
	//std::cout << "\nlmks: ";
        for(unsigned int i = 0; i<m_numActiveLandmarks; i++)
        {
	   //std::cout << m_landmarkHeap.dataOfMax() << " ";
           m_activeLandmark[i] = m_landmarkHeap.dataOfMax();
           m_landmarkHeap.pop();
        }*/
    }

    /**
     * @brief Runs a shortest path query between a source node s and a target node t
     *
     * @param s The source node
     * @param t The target node
     * @return The distance of the target node
     */
    WeightType runQuery( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u, v, lastNode;
        EdgeIterator e, lastEdge;
        //WeightType offset = LBdist( s, t) * 2;

        init( s, t);

        while( !(pqFront.empty() || pqBack.empty()))
        {
            if( (pqFront.minKey() + pqBack.minKey()) >=  m_minDistance)
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

        return std::numeric_limits<WeightType>::max();
    }
    

 private:

    GraphType& G;
    unsigned int* m_timestamp;
    PriorityQueueType pqFront, pqBack;
    NodeIterator m_viaNode;
    WeightType m_minDistance;
    unsigned int m_numSettledNodes; 
    NodeIterator m_source, m_target; 
    unsigned int m_numActiveLandmarks;

    //the container with the landmarks with the current tighter lower bound distances
    LandmarkMaxHeap m_landmarkHeap;

    //the total number of landmarks loaded in RAM memory
	unsigned int m_numLandmarks;

    //the vector with the active landmarks
    std::vector<unsigned short> m_activeLandmark;
    

    void searchForward()
    {
        EdgeIterator e, lastEdge;
        NodeIterator u, v;
        WeightType discoveredDistance, pqkey;
        
        u = pqFront.minItem();
        pqkey = pqFront.minKey();
        pqFront.popMin();

        //if( u->key == pqFront.minKey())
        {
            ++m_numSettledNodes;

            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                //if( e->weight == std::numeric_limits<unsigned int>::max()) continue;
                discoveredDistance = u->dist + e->weight;
                v = G.target(e);

                //unexplored by both the forward and backward search
                if( v->timestamp < (*m_timestamp))
                {
                    v->timestamp = (*m_timestamp);
                    v->dist = discoveredDistance;
                    v->pred = u->getDescriptor();
                    pqkey = forwardLBdist(discoveredDistance, v);
                    pqFront.insert( pqkey, v);

                    if( ( v->timestampBack == (*m_timestamp)) && ( discoveredDistance + v->distBack < m_minDistance))
                    {
                        m_minDistance = discoveredDistance + v->distBack;
                        m_viaNode = v;
                    }
                }

                //unexplored by the backward search
                else if(  v->dist > discoveredDistance)
                {
                    pqkey = ( pqkey + discoveredDistance) - v->dist;
                    v->dist = discoveredDistance;
                    v->pred = u->getDescriptor();

                    pqFront.insert( pqkey, v);

                    if( ( v->timestampBack == (*m_timestamp)) && ( discoveredDistance + v->distBack < m_minDistance))
                    {
                        m_minDistance = discoveredDistance + v->distBack;
                        m_viaNode = v;
                    }
                }
            }
        }

    }
    

    void searchBackward()
    {
        InEdgeIterator k, lastInEdge;
        NodeIterator u, v;
        WeightType discoveredDistance, pqkey;
        
        u = pqBack.minItem();
        pqBack.popMin();

        //if( u->keyBack == pqBack.minKey())
        {
            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                //if( k->weight == std::numeric_limits<unsigned int>::max()) continue;
                discoveredDistance = u->distBack + k->weight;
                v = G.source(k);

                //unexplored by both the forward and backward search
                if( v->timestampBack < (*m_timestamp))
                {
                    v->timestampBack = (*m_timestamp);
                    v->distBack = discoveredDistance;
                    v->succ = u->getDescriptor();
                    pqkey = backwardLBdist( discoveredDistance, v);
                    pqBack.insert( pqkey, v);

                    if( ( v->timestamp == (*m_timestamp)) && ( v->dist + discoveredDistance < m_minDistance))
                    {
                        m_minDistance = v->dist + discoveredDistance;
                        m_viaNode = v;
                    }
                }

                else if( v->distBack > discoveredDistance)
                {
                    pqkey = ( pqkey + discoveredDistance) - v->distBack;
                    v->distBack = discoveredDistance;
                    v->succ = u->getDescriptor();

                    pqBack.insert( pqkey, v);

                    if( ( v->timestamp == (*m_timestamp)) && ( v->dist + discoveredDistance < m_minDistance))
                    {
                        m_minDistance = v->dist + discoveredDistance;
                        m_viaNode = v;
                    }

                }
            }
        }

    }
    

    inline WeightType forwardLBdist( const WeightType& discoveredDistance, const NodeIterator& v) const
    {
	/*if( LBdist(v, m_target) != LBdistS(v, m_target) || LBdist(m_source, v) != LBdistS(m_source, v))
		{
		std::cout << "diff" << "Lvt:" << LBdist(v, m_target) << " vs SSE:" << LBdistS(v, m_target);
		std::cout << "diff" << "Lvt:" << LBdist(m_source, v) << " vs SSE:" << LBdistS(m_source, v);
		 exit(1);
		}*/

        return ( ( ( ( discoveredDistance << 1) + LBdist(v, m_target)) - LBdist(m_source, v)) >> 1) ;
    }


    inline WeightType backwardLBdist( const WeightType& discoveredDistance, const NodeIterator& v) const
    {
	/*if( LBdist(v, m_target) != LBdistS(v, m_target) || LBdist(m_source, v) != LBdistS(m_source, v))
		{
		std::cout << "diff" << "Lvt:" << LBdist(v, m_target) << " vs SSE:" << LBdistS(v, m_target);
		std::cout << "diff" << "Lvt:" << LBdist(m_source, v) << " vs SSE:" << LBdistS(m_source, v);
		 exit(1);
		}*/

        return ( ( ( ( discoveredDistance << 1) + LBdist(m_source, v)) - LBdist(v, m_target)) >> 1) ;
    }


    /**
     * @brief Computes a lower bound distance between two nodes. 
     * @param The source node.
     * @param The sink node.
     */
    WeightType LBdistR(const NodeIterator& v, const NodeIterator& target) const
    {
        WeightType tighestLowerBoundDistance = 0;
        WeightType lowerBoundDistance;

        //computation time : O(k), where k is the number of the active landmarks
        std::vector<unsigned short>::const_iterator it = m_activeLandmark.begin(), end = m_activeLandmark.end();

        //std::cout << "\nLb: ";
        for(;  it != end; ++it)
        {
            /*lower bound of dist(v,target) = dist(v, L) - dist(target, L)

              (source) -----> (v) ----------------> (target) -------> (landmark L)
              |                |                                                 |
              |                <—————————————————— dist(v, L) ——————————————————>|
              |                |                     |                           |
              |                |                     |<———— dist(target, L) ————>|
              |                |                     |                           | 
              |                | ↓↓   ↓↓   ↓↓   ↓↓ |                           | 
              |                |    dist(v,target)   |                           |
              |                |     lower bound     |                           |*/

            if( (v->landmark[*it].distanceToLandmark) > (target->landmark[*it].distanceToLandmark) )
            {
                lowerBoundDistance = (v->landmark[*it].distanceToLandmark) - (target->landmark[*it].distanceToLandmark);

		//std::cout << "(" << lowerBoundDistance << ")["<< v->landmark[*it].distanceToLandmark <<"][" << target->landmark[*it].distanceToLandmark << "] ";

                if(tighestLowerBoundDistance < lowerBoundDistance)
                  tighestLowerBoundDistance = lowerBoundDistance;
            }

            /*lower bound of dist(v,target) = dist(L, target) - dist(L, v)

              (landmark L) -------> (source) -----> (v) ----------------> (target)
              |                                                                  |
              |<———————————————————————— dist(L, target) ———————————————————————>|
              |                                                                  |
              |<———————————— dist(L, v) ————————————>|                           | 
                                                     |   ↓↓    ↓↓    ↓↓    ↓↓    | 
                                                     |      dist(v,target)       | 
                                                     |       lower bound         |*/                         
            //else
            if( (target->landmark[*it].distanceFromLandmark) > (v->landmark[*it].distanceFromLandmark))
            {
                lowerBoundDistance = (target->landmark[*it].distanceFromLandmark) - (v->landmark[*it].distanceFromLandmark);

		//std::cout << "(" << lowerBoundDistance << ")["<< target->landmark[*it].distanceFromLandmark <<"][" << v->landmark[*it].distanceToLandmark << "] ";

                if(tighestLowerBoundDistance < lowerBoundDistance)
                  tighestLowerBoundDistance = lowerBoundDistance;
            }

            //if(tighestLowerBoundDistance < lowerBoundDistance)
            //    tighestLowerBoundDistance = lowerBoundDistance;
        }

	//std::cout << "\nRlmk:" << tighestLowerBoundDistance;
	//static int x =0;
	//x++;
	//if( x > 10) exit(1);
        return tighestLowerBoundDistance;
    }


    WeightType LBdist(const NodeIterator& v, const NodeIterator& target) const
    {
        WeightType tighestLowerBoundDistance = 0;
        WeightType lowerBoundDistance;

       // __m128i max_vec; //= _mm_set_epi32( 0, 0, 0, 0);//_mm_setzero_si128();
        //max_vec[0] = max_vec[1] = max_vec[2] = max_vec[3] = 0;
	__m128i max_vec =_mm_setzero_si128();

	//std::cout << "\nvToL: ";
        //for( unsigned int j=0, sizej = 16; j < sizej; j+=1)
	  //  std::cout << v->landmark[j].distanceToLandmark << " ";

	//std::cout << "\nvFromL: ";
        //for( unsigned int j=0, sizej = 16; j < sizej; j+=1)
	//    std::cout << v->landmark[j].distanceFromLandmark << " ";

        //std::cout << "\nvL: ";
        //for( unsigned int j=0, sizej = 16; j < sizej; j+=1)
	  //  std::cout << target->landmark[j].distanceToLandmark << " ";

        for( unsigned int i=0, size = v->landmark.size(); i < size; i+=4)
        {

            __m128i* vL = (__m128i*) &( v->landmark[i]);
            __m128i* tL = (__m128i*) &( target->landmark[i]);
            __m128i res;

            res = _mm_sub_epi32( *vL, *tL);

	    //std::cout << "\nLFrom: ";
            //for( unsigned int j=0, sizej = 4; j < sizej; j+=1)
		//std::cout << dist[j] << " ";

            //res2 = _mm_sub_epi32( *tL, *vL);

            //undirected approach

            res = _mm_abs_epi32( res);

            //res3 = _mm_max_epi32( res1, res2);

	    //std::cout << "\nMax: ";
            //for( unsigned int j=0, sizej = 4; j < sizej; j+=1)
	//	std::cout << dist[j] << " ";

            max_vec = _mm_max_epi32( max_vec, res);

	    //std::cout << "\nMax: ";
            //for( unsigned int j=0, sizej = 4; j < sizej; j+=1)
	//	std::cout << dist[j] << " ";
        }

	int* maxV = (int *) &max_vec;

	int temp = maxV[0];
        //std::cout << " ~" << temp;
        for( unsigned int i=1, size = 4; i < size; i+=1)
        {
            if( temp < maxV[i])
	    {
                temp = maxV[i]; //max_vec[i];
	    }
        }

        tighestLowerBoundDistance = temp;
	//std::cout << "\nSlmk:" << tighestLowerBoundDistance;
	//static int x =0;
	//x++;
	//if( x > 10) exit(1);
        return tighestLowerBoundDistance;
    }

};

#endif//BIDASTARAVELMK_H

