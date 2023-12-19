#ifndef BIDASTARSYMLMK_H
#define BIDASTARSYMLMK_H

#include <Structs/Trees/priorityQueue.h>
#include <list>
#include <vector>

#include <Algorithms/ShortestPath/Astar/MaxHeapPriorityQueue.h>


/**
 * @class BidAstarSymLmk
 *
 * @brief The A* Variant of Bidirectional Dijkstra's algorithm using landmark lower bound distances
 * and following the symmetric approach.
 *
 * This class supports running queries between source and target nodes
 *
 * @tparam GraphType The type of the graph to run the algorithm on
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType>
class BidAstarSymLmk
{

 public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef typename GraphType::InEdgeIterator                      InEdgeIterator;
    typedef typename GraphType::SizeType                            SizeType;
    typedef unsigned int                                            WeightType;
    typedef PriorityQueue< WeightType, NodeIterator, HeapStorage>   PriorityQueueType;
    typedef MaxHeapPQ<unsigned short, WeightType> LandmarkMaxHeap;
    

    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    BidAstarSymLmk( GraphType& graph, unsigned int* timestamp, unsigned int numActiveLandmarks) : G(graph), m_timestamp(timestamp),        
                                                                                                   m_numActiveLandmarks(numActiveLandmarks)
    {
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
        s->distBack = std::numeric_limits<WeightType>::max();
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();
        pqFront.insert(0, s, &(s->pqitem));

        t->distBack = 0;
        t->dist = std::numeric_limits<WeightType>::max();
        t->timestamp = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();
        pqBack.insert(0, t, &(t->pqitemBack));

        m_landmarkHeap.reset();

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
        for(unsigned int i = 0; i<m_numActiveLandmarks; i++)
        {
           m_activeLandmark[i] = m_landmarkHeap.dataOfMax();
           m_landmarkHeap.pop();
        }
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

        init( s, t);
  
        while( !(pqFront.empty() || pqBack.empty()))
        {
             
            if( pqFront.minKey() >= m_minDistance || pqBack.minKey() >= m_minDistance )
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


    bool isBackwardFound( const NodeIterator& u)
    {
        return ( u->timestamp == (*m_timestamp)) && (u->distBack != std::numeric_limits<WeightType>::max());
    }


    bool isBackwardSettled( const NodeIterator& u)
    {
        return isBackwardFound(u) && (!isInBackQueue(u));
    }


    bool isForwardFound( const NodeIterator& u)
    {
        return ( u->timestamp == (*m_timestamp)) && (u->dist != std::numeric_limits<WeightType>::max());
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
        WeightType discoveredDistance;
        
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
                v->distBack = std::numeric_limits<WeightType>::max();
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();
                pqFront.insert( discoveredDistance + LBdist(v, m_target), v, &(v->pqitem));
            }

            else if( v->dist <= discoveredDistance )
                continue;

            //unexplored by the backward search
            else if( v->dist != std::numeric_limits<WeightType>::max())
            {
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();
                pqFront.decrease( discoveredDistance + LBdist(v, m_target), &(v->pqitem));

                if( ( v->distBack != std::numeric_limits<WeightType>::max()) && ( discoveredDistance + v->distBack < m_minDistance))
                {
                    m_minDistance = discoveredDistance + v->distBack;
                    m_viaNode = v;
                }
            }

            //unexplored by the forward search
            else
            {
                v->dist = discoveredDistance;
                v->pred = u->getDescriptor();
                pqFront.insert( discoveredDistance + LBdist(v, m_target), v, &(v->pqitem));

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
        WeightType discoveredDistance;
        
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
                v->dist = std::numeric_limits<WeightType>::max();
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();
                pqBack.insert( discoveredDistance + LBdist(m_source, v), v, &(v->pqitemBack));
            }

            else if( v->distBack <= discoveredDistance)
                continue;

            else if( v->distBack != std::numeric_limits<WeightType>::max())
            {
                v->distBack = discoveredDistance;
                v->succ = u->getDescriptor();
                pqBack.decrease( discoveredDistance + LBdist(m_source, v), &(v->pqitemBack));

                if( ( v->dist != std::numeric_limits<WeightType>::max()) && ( v->dist + discoveredDistance < m_minDistance))
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
                pqBack.insert( discoveredDistance + LBdist(m_source, v), v, &(v->pqitemBack));

                if( v->dist + discoveredDistance < m_minDistance)
                {
                    m_minDistance = v->dist + discoveredDistance;
                    m_viaNode = v;
                }
            }
        }
    }
    

    /**
     * @brief Computes a lower bound distance between two nodes. 
     * @param The source node.
     * @param The sink node.
     */
    WeightType LBdist(const NodeIterator& v, const NodeIterator& target) const
    {
        WeightType tighestLowerBoundDistance = 0;
        WeightType lowerBoundDistance;

        //computation time : O(k), where k is the number of the active landmarks
        std::vector<unsigned short>::const_iterator it = m_activeLandmark.begin(), end = m_activeLandmark.end();

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
            else
            {
                lowerBoundDistance = (target->landmark[*it].distanceFromLandmark) - (v->landmark[*it].distanceFromLandmark);
            }

            if(tighestLowerBoundDistance < lowerBoundDistance)
                tighestLowerBoundDistance = lowerBoundDistance;
        }

        return tighestLowerBoundDistance;
    }

};

#endif//BIDASTARSYMLMK_H
