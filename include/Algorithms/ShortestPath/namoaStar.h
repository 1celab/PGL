#ifndef NAMOASTAR_H
#define NAMOASTAR_H

#include <Structs/Trees/priorityQueue.h>
#include <Utilities/geographic.h>

template<class GraphType>
class NamoaStarDijkstra
{
public:
	typedef typename GraphType::NodeIterator    NodeIterator;
	typedef typename GraphType::EdgeIterator    EdgeIterator;
	typedef typename GraphType::SizeType        SizeType;
	typedef typename GraphType::NodeData        NodeData;

	typedef PriorityQueue< CriteriaList, NodeIterator, HeapStorage> PriorityQueueType;
	typedef typename PriorityQueueType::PQItem PQItem;   
	
    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    NamoaStarDijkstra( GraphType& graph, unsigned int numCriteria, unsigned int* timestamp):G(graph),m_numCriteria(numCriteria),m_timestamp(timestamp)
    {
    }
    
    /**
     * @brief Builds a shortest path tree routed on a source node
     *
     * @param s The source node
     */
    void runQuery( const typename GraphType::NodeIterator& s, const typename GraphType::NodeIterator& t)
    {
		NodeIterator u,v,lastNode;
		EdgeIterator e,lastEdge;

		assert( hasFeasiblePotentials(t));
		++(*m_timestamp);
		init();

		unsigned int* pqitem = new unsigned int();
		s->labels.push_back(Label( CriteriaList(m_numCriteria), 0, pqitem));
		pq.insert( CriteriaList(m_numCriteria) + getHeuristic(s, t), s, pqitem);

		while( !pq.empty())
		{
		    CriteriaList minCriteria = pq.min().key;
		    u = pq.minItem();
		    pq.popMin();

			CriteriaList g_u = minCriteria - getHeuristic(u, t);
			moveToClosed( g_u, u);

			if ( isDominatedByNodeLabels( t, minCriteria)) continue;

		    //std::cout << "extracting " << u->id << " with label ";
		    //label.print(std::cout, G);
		    //std::cout << std::endl; 

		  	for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
		    {
				v = G.target(e);
				
				/*if ( v->timestamp != (*m_timestamp))
				{
					v->labels.clear();
					v->timestamp = (*m_timestamp);
				}*/
				CriteriaList g_v = g_u + e->criteriaList;
				CriteriaList heuristicCost = g_v + getHeuristic(v, t);

				if ( distanceExistsInNode( v, g_v))
				{
					v->labels.push_back( Label( g_v, u->getDescriptor(), 0) );
				}
				else	
				{
					if( isDominatedByNodeLabels( v, g_v)) continue;
					eraseDominatedLabels( v, g_v);
					if( isDominatedByNodeLabels(t, heuristicCost)) continue;
					unsigned int* pqitem = new unsigned int();
					v->labels.push_back( Label( g_v, u->getDescriptor(), pqitem) );
				
					pq.insert( heuristicCost, v, pqitem);
				}
		    }
		}
    }

    const unsigned int& getSettledNodes()
    {
        return m_settled;
    }
    
    /**
     * @brief Runs a shortest path query between a source node s and a target node t
     *
     * @param s The source node
     * @param t The target node
     * @return The distance of the target node
     *
    WeightType runQuery( const typename GraphType::NodeIterator& s, const typename GraphType::NodeIterator& t)
    {

    }*/

private:
    GraphType& G;
    PriorityQueueType pq;
    unsigned int m_settled;
	unsigned int m_numCriteria;
	unsigned int* m_timestamp;

	bool distanceExistsInNode( const NodeIterator& v, const CriteriaList& g_v)
	{
		for ( std::vector<Label>::iterator it = v->labels.begin(); it != v->labels.end(); ++it)
		{
			if ( it->getCriteriaList() == g_v )
		    {
				return true;
		    }
		}
		return false;		
	}

	void eraseDominatedLabels( const NodeIterator& v, const CriteriaList& g_v)
	{
		std::vector<Label>::iterator it = v->labels.begin();
		while ( it != v->labels.end() )
		{
			if ( it->getCriteriaList().isDominatedBy(g_v) )
		    {
				if( it->isInQueue())
				{
					pq.remove( it->getPQitem());
					moveToClosed( it->getCriteriaList(), v);
				}
				it = v->labels.erase( it );
		    }
		    else 
		    {
			    ++it;
		    }
		}
	}

	CriteriaList getHeuristic( const NodeIterator& u, const NodeIterator& t)
	{
		//return CriteriaList( m_numCriteria, u->heuristic);
		std::vector< unsigned int> h;

		unsigned int flyingDist = euclideanDistance( u->x, u->y, t->x, t->y);
		unsigned int maxSpeed = 90;
		h.push_back( flyingDist);
		h.push_back( (unsigned int)(  ceil(double(3.6)*flyingDist)/ (maxSpeed) ));
		return CriteriaList( h);
	}

    bool hasFeasiblePotentials( const typename GraphType::NodeIterator& t)
    {
        NodeIterator u,v,lastNode;
        EdgeIterator e,lastEdge;

        for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
        {
            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                v = G.target(e);
				CriteriaList c_e = e->criteriaList;
				CriteriaList c_v = getHeuristic(v, t);
				CriteriaList c_u = getHeuristic(u, t);
                if( e->criteriaList + getHeuristic(v, t) < getHeuristic(u, t))
                {
                    return false;
                }
            }       
        }
        return true;
    }

	void init()
	{
		NodeIterator u, lastNode;
		for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
		{
		    u->labels.clear();
		}
		pq.clear();
	}

	bool isDominatedByNodeLabels( const NodeIterator& v, const CriteriaList& g_v)
	{
		for ( std::vector<Label>::iterator it = v->labels.begin(); it != v->labels.end(); ++it)
		{
			if ( it->getCriteriaList().dominates(g_v) )
		    {
				return true;
		    }
		}
		return false;
	}

	void moveToClosed( const CriteriaList& g_u, const NodeIterator& u)
	{
		for ( std::vector<Label>::iterator it = u->labels.begin(); it != u->labels.end(); ++it)
		{
			if ( (it->isInQueue()) && (it->getCriteriaList() == g_u) )
		    {
				it->deletePQitem();
		    }
		}		
	}
	
};


#endif//NAMOASTAR_H

