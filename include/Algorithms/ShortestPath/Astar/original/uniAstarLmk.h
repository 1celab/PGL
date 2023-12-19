#ifndef UNIASTARLMK_H
#define UNIASTARLMK_H

#include <Structs/Trees/priorityQueue.h>
#include <list>
#include <vector>

#include <Algorithms/ShortestPath/Astar/MaxHeapPriorityQueue.h>

/**
 * @class UniAstarLmk
 *
 * @brief The A* Variant of Dijkstra's algorithm using landmark lower bound distances
 *
 * This class supports running queries between source and target nodes
 *
 * @tparam GraphType The type of the graph to run the algorithm on
 *
 * @author Andreas Paraskevopoulos
 *
 */

template<class GraphType>
class UniAstarLmk
{

 public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
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
    UniAstarLmk( GraphType& graph, unsigned int* timestamp, unsigned int numActiveLandmarks) : G(graph), m_timestamp(timestamp),        
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


    void init( const typename GraphType::NodeIterator& s, const typename GraphType::NodeIterator& t)
    {
        pq.clear();
        ++(*m_timestamp);

        m_numSettledNodes = 0;

        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();
        pq.insert( 0, s, &(s->pqitem));
        
        m_landmarkHeap.reset();

        //assert(hasFeasiblePotentials(t));

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
    WeightType runQuery( const typename GraphType::NodeIterator& s, const typename GraphType::NodeIterator& t)
    {
        NodeIterator u,v;
        EdgeIterator e,lastEdge;
        WeightType discoveredDistance;
        
        init(s, t);
     
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

                else if( v->dist > discoveredDistance)
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
    unsigned int m_numActiveLandmarks;

    //the container with the landmarks with the current tighter lower bound distances
    LandmarkMaxHeap m_landmarkHeap;

    //the total number of landmarks loaded in RAM memory
	unsigned int m_numLandmarks;

    //the vector with the active landmarks
    std::vector<unsigned short> m_activeLandmark;


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

#endif//UNIASTARLMK_H
