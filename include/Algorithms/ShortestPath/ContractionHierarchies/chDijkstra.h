#ifndef CHDIJKSTRA_H
#define CHDIJKSTRA_H

#include <Structs/Trees/priorityQueue.h>
#include <math.h>



template<class GraphType>
class CHPreprocessing
{
public:
    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef typename GraphType::InEdgeIterator                      InEdgeIterator;
	typedef typename GraphType::EdgeDescriptor                      EdgeDescriptor;
    typedef typename GraphType::SizeType                            SizeType;
    typedef unsigned int                                            WeightType;
    typedef PriorityQueue< WeightType, NodeIterator, HeapStorage>   PriorityQueueType;
    
	class Partition
    {
    public:
        
        Partition()
        {
        }
        
	    Partition( unsigned int xmin, unsigned int xmax, unsigned int ymin, unsigned int ymax, unsigned int div, unsigned int l = 1):
	                        m_xmin(xmin),
	                        m_xmax(xmax + 1),
	                        m_ymin(ymin),
	                        m_ymax(ymax + 1),
	                        m_div(div),
	                        m_numLevels(l)
	    {    
        }

        unsigned int getCell( unsigned int x, unsigned int y, unsigned int l = 0)
	    {   
	        unsigned int column = getOffsetX(x, l) / getStepXofLevel(l);
		    unsigned int row = getOffsetY(y, l) / getStepYofLevel(l);
		    return row*m_div + column;
        }
    
        unsigned int getMaxLevel () const
        {
            return getNumLevels() - 1;
        }
    
        unsigned int getNumCells ( const unsigned int& l = 0) const
        {
            return m_div * m_div;
        }
    
        unsigned int getNumLevels () const
        {
            return m_numLevels;
        }
	
	    unsigned int getOffsetX ( const unsigned int& x, unsigned int l)
        {
            if ( l == getMaxLevel())
                return x-m_xmin;
            else
                return (x-m_xmin)%getStepXofLevel(l+1);
        }
    
        unsigned int getOffsetY ( unsigned int y, unsigned int l)
        {
            if ( l == getMaxLevel())
                return y-m_ymin;
            else
                return (y-m_ymin)%getStepYofLevel(l+1);
        }
    
        unsigned int getOnMask( unsigned int index)
	    {
	        return 1 << index;
	    }
	
	    unsigned int getOffMask( unsigned int index)
	    {
            return ~getOnMask( index);
        }
	
	    unsigned int getStepXofLevel(unsigned int l)
        {
            //std::cout << "Step X at level " << l << " is " << (m_xmax-m_xmin)/pow( (double)m_div, (double)m_numLevels-l) << std::endl;
            return (m_xmax-m_xmin)/pow( (double)m_div, (double)m_numLevels-l);
        }
    
        unsigned int getStepYofLevel(unsigned int l)
        {
            //std::cout << "Step Y at level " << l << " is " << (m_ymax-m_ymin)/pow( (double)m_div, (double)m_numLevels-l) << std::endl;
            return (m_ymax-m_ymin)/pow( (double)m_div, (double)m_numLevels-l);
        }
        
        void reset( unsigned int xmin, unsigned int xmax, unsigned int ymin, unsigned int ymax, unsigned int div, unsigned int l = 1)
        {
            m_xmin = xmin;
            m_xmax = xmax + 1;
            m_ymin = ymin;
            m_ymax = ymax + 1;
            m_div = div;
            m_numLevels = l;
        }
	
    private:
        unsigned int m_xmin, m_xmax, m_ymin, m_ymax, m_div, m_numLevels;
    };

    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    CHPreprocessing( GraphType& graph, unsigned int* timestamp):G(graph),m_timestamp(timestamp)
    {
    }
    

    void buildSubTree( const typename GraphType::NodeIterator& s, const WeightType& maxDist)
    {
        NodeIterator u,v,lastNode;
        EdgeIterator e,lastEdge;
        
        pq.clear();
        m_settled = 1;
        ++(*m_timestamp);
        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();

        pq.insert( s->dist, s, &(s->pqitem));


        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            ++m_settled;

            if( u->dist >= maxDist) break;
            
            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                v = G.target(e);
                if( v->isContracted) continue;
                if( v->timestamp < (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->dist = u->dist + e->weight;
                    v->timestamp = (*m_timestamp);
                    pq.insert( v->dist, v, &(v->pqitem));
                }
                else if( v->dist > u->dist + e->weight )
                {
                    v->pred = u->getDescriptor();
                    v->dist = u->dist + e->weight;
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
    void process()
    {
		
		NodeIterator u, last;
		unsigned int minRank = 0;
        createPartition();
		/*for( unsigned int i = 0; i < m_partition.getNumCells(); ++i)
		{
			std::cout << "Ranking nodes in cell " << i << std::endl; 
			rankNodesInCell(i, minRank);
		}*/
		PriorityQueue< unsigned int, NodeIterator, HeapStorage> priority;

		for ( u = G.beginNodes(), last = G.endNodes(); u != last; ++u)
        {
			priority.insert( G.degree(u), u);
		}

		ProgressStream contraction_progress( G.getNumNodes());
		contraction_progress.label() << "\tContracting " << G.getNumNodes() << " nodes";

		NodeSelection<GraphType> selection(&G);

		while( !priority.empty())
		{
			u = priority.minItem();
            priority.popMin();
			u->rank = minRank++;
			selection.select(u);
			
		}

		for( typename std::vector<NodeDescriptor>::iterator it = selection.begin(); it != selection.end(); ++it)
		{
			contract( G.getNodeIterator(*it));
			++contraction_progress;
		}

    }   
    
private:
    GraphType& G;
    unsigned int* m_timestamp;
    PriorityQueueType pq;
    WeightType curMin, minDistance;
    Partition m_partition;
    unsigned int m_settled;

	void contract( const NodeIterator& u)
	{
		NodeIterator v, w, viaNode;
		EdgeIterator e, e1, e2;
		EdgeDescriptor eD;
		InEdgeIterator k;
		std::vector<NodeDescriptor> outNeighbors = getOutNeighbors( G, u);
		std::vector<NodeDescriptor> inNeighbors = getInNeighbors( G, u);
		unsigned int witnessDist = 0;

		for( unsigned int i = 0; i < outNeighbors.size(); ++i)
		{
			for( unsigned int j = 0; j < inNeighbors.size(); ++j)
			{
				v = G.getNodeIterator( inNeighbors[j]);
				w = G.getNodeIterator( outNeighbors[i]);

				if( w == v) continue;
				if( (v->rank < u->rank) || (w->rank < u->rank) ) continue;

				e1 = G.getEdgeIterator( v, u);
				e2 = G.getEdgeIterator( u, w);

				witnessDist = findDistance( v, w);

				//std::cout << e1->weight << " + " << e2->weight << " ? " << witnessDist << std::endl;

				if( e1->weight + e2->weight < witnessDist)
				{
					std::cout << "adding shortcut\n";
					eD = G.insertEdge( inNeighbors[j], outNeighbors[i]);
					e = G.getEdgeIterator( eD);
					k = G.getInEdgeIterator( e);
					e->weight = e1->weight + e2->weight;
					e->viaNode = G.getNodeDescriptor( u);
					k->weight = e1->weight + e2->weight;
					k->viaNode = G.getNodeDescriptor( u);
				}
			}
		}
	}

	void createPartition()
    {
        std::cout << "Creating Partition...\n";
        unsigned int xmax = 0, xmin = std::numeric_limits<unsigned int>::max(), ymax = 0, ymin = std::numeric_limits<unsigned int>::max();
        NodeIterator u, last;
        EdgeIterator e, lastEdge;
        InEdgeIterator k;
        for ( u = G.beginNodes(), last = G.endNodes(); u != last; ++u)
        {
            if ( u->x > xmax) xmax = u->x;
            if ( u->x < xmin) xmin = u->x;
            if ( u->y > ymax) ymax = u->y;
            if ( u->y < ymin) ymin = u->y;
        }
        m_partition.reset( xmin, xmax, ymin, ymax, 4);
    }

	WeightType findDistance( const typename GraphType::NodeIterator& s, const typename GraphType::NodeIterator& t, const unsigned int maxHops = 6)
    {
        NodeIterator u,v,lastNode;
        EdgeIterator e,lastEdge;
        
        pq.clear();
        ++(*m_timestamp);
        s->dist = 0;
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();;

        pq.insert( s->dist, s, &(s->pqitem));

        while( !pq.empty())
        {
            u = pq.minItem();
            pq.popMin();
            if( u == t) break;
            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                v = G.target(e);

                if( v->timestamp < (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->dist = u->dist + e->weight;
                    v->timestamp = (*m_timestamp);
                    pq.insert( v->dist, v, &(v->pqitem));
                }
                else if( v->dist > u->dist + e->weight )
                {
                    v->pred = u->getDescriptor();
                    v->dist = u->dist + e->weight;
                    pq.decrease( v->dist, &(v->pqitem));
                }
            }
        }
        return t->dist;
    }

	std::vector<NodeIterator> getCoveringInNeighbors( const NodeIterator& root)
	{
		NodeIterator u,v;
		EdgeIterator k, end;
		++(*m_timestamp);
		std::stack< NodeIterator> S;
		S.push(root);
		root->timestamp = (*m_timestamp);
		std::vector<NodeIterator> covering;

		while( !S.empty())
		{
		    u = S.top();
		    S.pop();      
			if( u->rank > root->rank)
			{
				covering.push_back(u);
				continue;
			}
		    for( k = G.beginInEdges(u), end = G.endInEdges(u); k != end; ++k)
		    {
		        v = G.source( k);
		        if( v->timestamp != (*m_timestamp))
		        {
		            v->timestamp = (*m_timestamp);
		            S.push(v);
		        }
		    }
		} 
		return covering;
	}

	std::vector<NodeIterator> getCoveringOutNeighbors( const NodeIterator& root)
	{
		NodeIterator u,v;
		EdgeIterator e, end;
		++(*m_timestamp);
		std::stack< NodeIterator> S;
		S.push(root);
		root->timestamp = (*m_timestamp);
		std::vector<NodeIterator> covering;

		while( !S.empty())
		{
		    u = S.top();
		    S.pop();      
			if( u->rank > root->rank)
			{
				covering.push_back(u);
				continue;
			}
		    for( e = G.beginEdges(u), end = G.endEdges(u); e != end; ++e)
		    {
		        v = G.target( e);
		        if( v->timestamp != (*m_timestamp))
		        {
		            v->timestamp = (*m_timestamp);
		            S.push(v);
		        }
		    }
		} 
		return covering;
	}

	bool isBoundaryNode( const NodeIterator& u)
	{
		NodeIterator v;
		std::vector<NodeDescriptor> neighbors = getNeighbors( G, u);
	
		for( unsigned int i = 0; i < neighbors.size(); ++i)
		{
			v = G.getNodeIterator(neighbors[i]);
			if ( m_partition.getCell( u->x, u->y) != m_partition.getCell( v->x, v->y) )
				return true;
		}
		return false;
	}

	void rankNodesInCell( const unsigned int& cell, unsigned int& minRank)
	{
		NodeIterator u, last;
		PriorityQueue< unsigned int, NodeIterator, HeapStorage> priority;
		unsigned int edgeDifference;
		for ( u = G.beginNodes(), last = G.endNodes(); u != last; ++u)
        {	
			u->rank = minRank++;
			//edgeDifference = getNumCoveringNeighbors(u);
            //priority.insert( edgeDifference, u);
        }
		/*while( !priority.empty())
		{
			u = priority.minItem();
            priority.popMin();
			u->rank = minRank++;
		}*/
	}
};


template<class GraphType>
class CHDijkstra
{
public:
    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef typename GraphType::InEdgeIterator                    	InEdgeIterator;
    typedef typename GraphType::SizeType                            SizeType;
    typedef unsigned int                                            WeightType;
    typedef PriorityQueue< WeightType, NodeIterator, HeapStorage>   PriorityQueueType;

    /**
     * @brief Constructor
     *
     * @param graph The graph to run the algorithm on
     * @param timestamp An address containing a timestamp. A timestamp must be given in order to check whether a node is visited or not
     */
    CHDijkstra( GraphType& graph, unsigned int* timestamp):G(graph),m_timestamp(timestamp)
    {
    }
    
    const unsigned int& getNumSettledNodes()
    {
        return m_settled;
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
        EdgeIterator e,lastEdge;
        
        pqFront.clear();
        pqBack.clear();
        ++(*m_timestamp);

        minDistance = std::numeric_limits<WeightType>::max();

        m_settled = 2;

        s->dist = 0;
        s->distBack = std::numeric_limits<WeightType>::max();//<// 
        s->timestamp = (*m_timestamp);
        s->pred = G.nilNodeDescriptor();

        t->dist = std::numeric_limits<WeightType>::max(); //<// 
        t->distBack = 0;
        t->timestamp = (*m_timestamp);
        t->succ = G.nilNodeDescriptor();

        pqFront.insert( s->dist, s, &(s->pqitem));
        pqBack.insert( t->distBack, t, &(t->pqitemBack));

        while( ! ( pqFront.empty() && pqBack.empty()))
        {
            curMin = 0;
            if( !pqFront.empty()) curMin += pqFront.minKey();
            if( !pqBack.empty()) curMin += pqBack.minKey();
            if( curMin > minDistance)
            {
                break;
            }
            searchForward();
            searchBackward();
        }
        
        u = viaNode;
        t->dist = u->dist;
        while( u->succ != G.nilNodeDescriptor())
        {
            v = G.getNodeIterator(u->succ);
            v->pred = G.getNodeDescriptor( u);
            e = G.getEdgeIterator( u, v);
            t->dist += e->weight;
            u = v;
        }

        return t->dist;
    }   
    
private:
    GraphType& G;
    unsigned int* m_timestamp;
    PriorityQueueType pqFront, pqBack;
    NodeIterator viaNode;
    WeightType curMin, minDistance;
    unsigned int m_settled;    

    bool isBackwardFound( const NodeIterator& u)
    {
        return (u->timestamp == (*m_timestamp)) && (u->distBack != std::numeric_limits<WeightType>::max());
    }
    
    bool isBackwardSettled( const NodeIterator& u)
    {
        return isBackwardFound(u) && (!isInBackQueue(u));
    }
    
    bool isForwardFound( const NodeIterator& u)
    {
        return (u->timestamp == (*m_timestamp)) && (u->dist != std::numeric_limits<WeightType>::max());
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
        EdgeIterator e,lastEdge;
        NodeIterator u,v;
        if( !pqFront.empty())
        {
            u = pqFront.minItem();
            pqFront.popMin();
            ++m_settled;
            for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
            {
                v = G.target(e);
				if( v->rank < u->rank) continue;

                if( v->timestamp != (*m_timestamp))
                {
                    v->pred = u->getDescriptor();
                    v->dist = u->dist + e->weight;
                    v->timestamp = (*m_timestamp);
                    v->distBack = std::numeric_limits<WeightType>::max();
                    pqFront.insert( v->dist, v, &(v->pqitem));
                }
                else if( v->dist == std::numeric_limits<WeightType>::max())
                {
                    v->pred = u->getDescriptor();
                    v->dist = u->dist + e->weight;
                    pqFront.insert( v->dist, v, &(v->pqitem));
                }
                else if( v->dist > u->dist + e->weight )
                {
                    v->pred = u->getDescriptor();
                    v->dist = u->dist + e->weight;
                    pqFront.decrease( v->dist, &(v->pqitem));
                }

                
                if( isBackwardFound(v) && ( u->dist + e->weight + v->distBack < minDistance))
                {
                    minDistance = u->dist +e->weight + v->distBack;
                    //std::cout << "Settled " << G.getRelativePosition(v) << " from front with " << minDistance << " (" << u->dist << "+" << v->distBack << ")!\n";
                    std::cout << "Settled from front with " << minDistance << " (" << u->dist << "+" << v->distBack << ")!\n";
                    viaNode = v;
                }
            }
        }
    }
    
    void searchBackward()
    {
        InEdgeIterator k,lastInEdge;
        NodeIterator u,v;
        if( !pqBack.empty())
        {
            u = pqBack.minItem();
            pqBack.popMin();
            ++m_settled;
            for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
            {
                v = G.source(k);
				if( v->rank < u->rank) continue;

                if( v->timestamp != (*m_timestamp))
                {
                    v->succ = u->getDescriptor();
                    v->distBack = u->distBack + k->weight;
                    v->timestamp = (*m_timestamp);
                    v->dist = std::numeric_limits<WeightType>::max();
                    pqBack.insert( v->distBack, v, &(v->pqitemBack));
                }
                else if( v->distBack == std::numeric_limits<WeightType>::max())
                {
                    v->succ = u->getDescriptor();
                    v->distBack = u->distBack + k->weight;
                    pqBack.insert( v->distBack, v, &(v->pqitemBack));
                }
                else if( v->distBack > u->distBack + k->weight )
                {
                    v->succ = u->getDescriptor();
                    v->distBack = u->distBack + k->weight;
                    pqBack.decrease( v->distBack, &(v->pqitemBack));
                }

                if( isForwardFound(v) && ( v->dist + k->weight + u->distBack < minDistance))
                {
                    minDistance = v->dist +k->weight + u->distBack;
                    //std::cout << "Settled " << G.getRelativePosition(v) << " from back with " << minDistance << " (" << v->dist << "+" << u->distBack << ")!\n";
                    std::cout << "Settled from back with " << minDistance << " (" << v->dist << "+" << u->distBack << ")!\n";
                    viaNode = v;
                }
            }
        }
    }
};

#endif//CHDIJKSTRA_H

