#include <Structs/Trees/priorityQueue.h>
#include <Utilities/geographic.h>
#include <iostream>
#include <fstream>
#include <queue>
#include <stdio.h>
#include <string>
#include <sstream>
#define MAX_HOPS 10
#define CONTRACTION_PARAMETER 2.5


/**
 * @class PreprocessingSharc
 *
 * @brief Preprocessing of Sharc algorithm implementation
 *
 * This class supports the preprocessing of SHARC-Routing in static scenarios. Divided into three phases,
 * the initialization phase, an iterative process and  the finalization phase.
 *
 * @tparam GraphType The type of the graph to run the algorithm on
 * @author Nikos Rousias
 *
 */

template < typename GraphType>
class PreprocessingSharc
{
	public:
		typedef typename GraphType::NodeIterator                        NodeIterator;
		typedef typename GraphType::NodeDescriptor 						NodeDescriptor;
		typedef typename GraphType::EdgeIterator                        EdgeIterator;
        typedef typename GraphType::EdgeDescriptor                      EdgeDescriptor;
		typedef typename GraphType::SizeType                            SizeType;
		typedef unsigned int                                            WeightType;
		typedef PriorityQueue< WeightType, NodeIterator, HeapStorage>   PriorityQueueType;
	

        /**
         * @brief Constructor
         *
         * @param G The graph to run the algorithm on
         * @param numLevels The number of levels of the graph partition
         * @param timestamp An address containing a timestamp
         * @param parts The number of cells on each level   
         */
		PreprocessingSharc( GraphType& G, unsigned short numLevels, unsigned int* timestamp, std::vector<unsigned short> &parts): m_G(G),m_numLevels(numLevels),m_timestamp(timestamp),m_parts(parts)
		{
        }
	
		/**
         * @brief Calculation of weights and initialization of flags for each edge
         */
		void calcWeightsAndInitFlags()
		{
			NodeIterator u, v;
			EdgeIterator e, lastEdge;
            std::vector<NodeDescriptor> neighbors;
            std::string map ="luxembourg";
            std::ifstream in;
            
			for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
			{  
                u->cell.resize( m_numLevels);
                for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				{
					v = m_G.target(e);
                    e->weight = ceil(euclideanDistance( u->x, u->y, v->x, v->y));
                    e->flag.resize( m_numLevels);
                    
                    for( unsigned int l = 0; l < m_numLevels; ++l)
                        e->flag[l].resize( m_parts[l]);
                }
			}

            for( unsigned int i = 0; i < m_numLevels; ++i)
            {
                std::ostringstream ostr;
                ostr << m_parts[i];
                std::string file = std::string(getenv("HOME")) + "/Projects/Graphs/DIMACS10/" + map + "/parts/" + map + ".osm.graph.part." + ostr.str();
                in.open(file.c_str());
                for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
                    in >> u->cell[i];
                in.close();
            }
            std::cout<<"\tcalcWeights...complete" << std::endl;
        }
        
        /**
         * @brief Finds the neighbors of a node
         *
         * @param u The node for which will be calculated neighbors
         * @param neighbors The vector of calculated neighbors   
         */
        void getOutNeighbors( NodeIterator u, std::vector<NodeDescriptor> &neighbors)
        {
            NodeIterator v;
            EdgeIterator e, lastEdge;
            
            neighbors.clear();
               
            for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
            {
                v = m_G.target(e);
                neighbors.push_back(m_G.getNodeDescriptor(v));
            }
        }

        /**
         * @brief The contraction routine
         *
         * @param l The number of the current level
         */            
        void contractNodes( unsigned int l)
		{
			NodeIterator u, v, w, temp;
			EdgeIterator e, e1, e2, lastEdge;
			unsigned int components = 0, core = 0, shortcuts = 0;
			std::vector<NodeDescriptor> neighbors, discovered;	
            std::stack<NodeIterator> Q;	

            std::cout << "\tCounting components and core nodes at level " << l << std::endl;	
            pq.clear();
            ++(*m_timestamp);

            unsigned int numActiveNodes = 0;
            for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
            {
                if( (u->isOneShell == false) && (isActiveNode(u, l))) 
                {  
                    ++numActiveNodes;
                    u->timestamp = (*m_timestamp);
                }
            }
            std::cout<<"\tActive Nodes ->" << numActiveNodes << std::endl;

            while( discovered.size() != numActiveNodes)
            {
                for( temp = m_G.beginNodes(); temp != m_G.endNodes(); ++temp)
                {
                    if( temp->timestamp == (*m_timestamp))    break;
                }

                temp->timestamp = (*m_timestamp) - 1;
                Q.push(temp);
                
                while( !Q.empty())
                {
                    u = Q.top();
                    Q.pop();
                    discovered.push_back(m_G.getNodeDescriptor(u));

                    if( isBypassableNode( u, l))
			        {
                        u->inactiveLevel = l;
                        pq.insert( u->key, u, &(u->pqitem));
				        ++components;
			        }
			        else
				        ++core;
                    
                    for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
                    {
                        v = m_G.target(e);
                        if( v->timestamp == (*m_timestamp))
                        {
                            v->timestamp = (*m_timestamp) - 1;
                            Q.push(v);
                        }
                    }
                }	
            }            
			std::cout<<"\tAt Level "<< l <<" component nodes: "<< components <<" core nodes: " << core << std::endl;
			discovered.clear();

            for( temp = m_G.beginNodes(); temp != m_G.endNodes(); ++temp)
            {
                if( temp->inactiveLevel == l)    temp->inactiveLevel = std::numeric_limits<unsigned int>::max();
            }

            ProgressStream node_progress2( pq.size());
            node_progress2.label() << "\tContracting nodes at level " << l;			
			while( !pq.empty())
			{
				++node_progress2;
				u = pq.minItem();
                pq.popMin();
                u->inactiveLevel = l;

                getOutNeighbors( u, neighbors);
			    
                for( unsigned int i = 0;  i < neighbors.size(); ++i)
				{
					w = m_G.getNodeIterator(neighbors[i]);
                    if( (w->isOneShell == true) || (!isActiveNode(w, l)))  continue;
                    e1 = m_G.getEdgeIterator( w, u);                    
                    
                    for( unsigned int y = 0; y < neighbors.size(); ++y)
					{
						v = m_G.getNodeIterator(neighbors[y]);
                        if( (v->isOneShell == true) || (!isActiveNode(v, l)))  continue;
						if( v == w)   continue;           
                        e2 = m_G.getEdgeIterator( u, v);
                        unsigned int viaWeight = e1->weight + e2->weight;
						unsigned short numhops = e1->hops + e2->hops;
												
						if( m_G.hasEdge( w, v))
						{
                            e = m_G.getEdgeIterator( w, v);
							e->isActive = true;
							//e->viaNode = u->getDescriptor();
							
							if( e->weight > viaWeight)
							{                        
                                ++shortcuts;
								e->weight = viaWeight;
                                e->hops = numhops;
							}
						}
						else
						{
                            ++shortcuts;
						    e = m_G.getEdgeIterator(m_G.insertEdge( neighbors[i], neighbors[y]));
							e->weight = viaWeight;
                            e->levelInsert = l;
							e->hops = numhops;
                            //e->viaNode = u->getDescriptor();
                            e->flag.resize( m_numLevels);
                            
                            for( unsigned int l = 0; l < m_numLevels; ++l)
                                e->flag[l].resize( m_parts[l]);
						}
                        e2->isActive = false;
                        if( (e2->isActive == false) && (e2->levelInsert == l))
                        {
                            if( (u->inactiveLevel == l) && (v->inactiveLevel == l))
                            {
                                m_G.eraseEdge(m_G.getEdgeDescriptor(e2));
                                --shortcuts;
                            }
                        }                
					}
                    e1->isActive = false;
                    if( (e1->isActive == false) && (e1->levelInsert == l))
                    {
                        if( (w->inactiveLevel == l) && (u->inactiveLevel == l))
                        {                       
                            --shortcuts;
                            m_G.eraseEdge(m_G.getEdgeDescriptor(e1));
                        }
                    }
				}
			}
            std::cout<<"\tThe number of added shortcuts: " << shortcuts << std::endl;
            std::vector<EdgeDescriptor> edges;	
            unsigned int removed = 0;
            for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
			{
                for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				{
					v = m_G.target(e);
                    if( (e->levelInsert == l) && (u->inactiveLevel == l) && (v->inactiveLevel == l))
                    {
                        edges.push_back(m_G.getEdgeDescriptor(e));
                        ++removed;
                    }                    
                }    
            }
            while( !edges.empty())
            {
                m_G.eraseEdge(edges.back());
                edges.pop_back();
            }
            std::cout<<"\tRemoved shortcuts  " << removed << " Remained  " << shortcuts - removed << std::endl;		
		}
		
		/**
         * @brief The Edge Reduction routine
         *
         * @param l The number of the current level
         */ 		
		void edgeReduction( unsigned int l)
		{
			NodeIterator u, v;
			EdgeIterator e;
            unsigned int count = 0;

            ProgressStream node_progress( m_G.getNumNodes());
            node_progress.label() << "\tedge Reduction at level " << l;	
			std::vector<NodeDescriptor> neighbors;
            
			for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
			{
				++node_progress;
				if( (u->isOneShell == true) || (hasBeenBypassed( u, l)))   continue;
				findWitnesses( u, l);
				neighbors.clear();
                getOutNeighbors( u, neighbors);
				
				for( unsigned int i = 0;  i < neighbors.size(); ++i)
				{
                    v = m_G.getNodeIterator( neighbors[i]);
                    if( (v->isOneShell == true) || (hasBeenBypassed( v, l)))   continue;
                    e = m_G.getEdgeIterator( u, v);
					if( v->dist < u->dist + e->weight)
					{
                        if( e->levelInsert == l)
                        {
                            m_G.eraseEdge(m_G.getEdgeDescriptor(e));    //remove edge (u, neighbors[i])
                            ++count;
                        }
					}
				}				
			}
			std::cout<<"\tAt edgeReduction removed " << count << " edges " << std::endl;	
		}
		
		/**
         * @brief Builds a shortest path tree routed on node s
         *
         * @param s The source node s
         * @param l The number of the current level
         */ 	
		void findWitnesses( const NodeIterator& s, unsigned int l)
		{
		    NodeIterator u, v, tempS = s;
			EdgeIterator e, lastEdge;
            init(tempS);
            unsigned int counter1 = 0, counter2 = 0, neighRemaining = getNumActiveNeigh( s, l);
                        
			while( (!pq.empty()) && (counter1 < 10000) && (counter2 < neighRemaining))
			{
				u = pq.minItem();
				pq.popMin();
				++counter1;

				if( m_G.hasEdge( tempS, u))    ++counter2;
				
				for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				{
					v = m_G.target(e);
                    if( (v->isOneShell == true) || (hasBeenBypassed( v, l)))   continue;
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
         * @brief Finds the number of active neighbors of a node u
         *
         * @param u The node u
         * @param l The number of the current level
         *   
         * @return The number of active neighbors of u
         */ 	
		unsigned int getNumActiveNeigh( NodeIterator u, unsigned int l)
		{
			NodeIterator temp;
			unsigned int numNeighbors = 0;
            std::vector<NodeDescriptor> neighbors;
			getOutNeighbors( u, neighbors);

            for( unsigned int i = 0;  i < neighbors.size(); ++i)
			{   
            	temp = m_G.getNodeIterator(neighbors[i]);
                if( (temp->isOneShell == true) || (hasBeenBypassed( temp, l)))   continue;
                ++numNeighbors;
            }

            return numNeighbors;
		}
		
		/**
         * @brief Finds the number of boundary nodes of a cell
         *
         * @param s The node s for which we calculate the cell`s boundary nodes 
         * @param l The number of the current level
         *
         * @return The number of boundary nodes of the cell
         */ 
        unsigned int getNumOfBoundaryOfCell( NodeIterator s, unsigned int l)
		{
			NodeIterator u;
			unsigned int count = 0;
			
			for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
			{
                if( (u->isOneShell == true) || (hasBeenBypassed( u, l)) || (u->cell[l] != s->cell[l]))   continue;
                
				if( isBoundaryNode( u, l))
				    ++count;
			}
			return count;	
		}

		/**
         * @brief Calculate the number of edges for a shortcut
         *
         * @param s The source node s of the shortcut 
         * @param t The target node t of the shortcut 
         *
         * @return The number of edges for the shortcut
         */ 
		unsigned int getNumOfHops( NodeIterator s, NodeIterator t)
		{
			NodeIterator u, v, tempS = s, tempT = t;
			EdgeIterator e, lastEdge;
			unsigned short numOfHops = 0;
		    pqB.clear();
             
            ++(*m_timestamp);
            tempS->dist = 0;
            tempS->timestamp = (*m_timestamp);
            tempS->pred = m_G.nilNodeDescriptor();
            pqB.insert( tempS->dist, tempS, &(tempS->pqBitem));
						
			while( !pqB.empty())
			{
				u = pqB.minItem();
				pqB.popMin();
				if( u == t) break;
							
				for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				{
					v = m_G.target(e);
					if( v->timestamp < (*m_timestamp))
					{
						v->pred = u->getDescriptor();
						v->dist = u->dist + e->weight;
						v->timestamp = (*m_timestamp);
						pqB.insert( v->dist, v, &(v->pqBitem));
					}
					else if( v->dist > u->dist + e->weight)
					{
						v->pred = u->getDescriptor();
						v->dist = u->dist + e->weight;
						pqB.update( v->dist, &(v->pqBitem));
					}
				}
			}
            
			while( tempT != tempS)
            {
                u = m_G.getNodeIterator((NodeDescriptor)tempT->pred);
                e = m_G.getEdgeIterator( u, tempT);
                numOfHops += e->hops;
                tempT = u;
            }
			return numOfHops;
		}

		/**
         * @brief Calculate the total weight of edges for a shortcut
         *
         * @param s The source node s of the shortcut 
         * @param t The target node t of the shortcut
         * @param l The number of the current level
         *
         * @return The total weight of edges for the shortcut
         */ 
		unsigned int getWeightOfHops( NodeIterator s, NodeIterator t, unsigned int l)
		{
			NodeIterator u, v, tempS = s, tempT = t;
			EdgeIterator e, lastEdge;
            pqB.clear();
            
            ++(*m_timestamp);
            tempS->dist = 0;
            tempS->timestamp = (*m_timestamp);
            tempS->pred = m_G.nilNodeDescriptor();
            pqB.insert( tempS->dist, tempS, &(tempS->pqBitem));
						
			while( !pqB.empty())
			{
				u = pqB.minItem();
				pqB.popMin();
				if( u == t) break;
				
				for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				{
					v = m_G.target(e);
                    if( (v->isOneShell == true) || (!isActiveNode(v, l)))  continue;
					if( v->timestamp < (*m_timestamp))
					{
						v->pred = u->getDescriptor();
						v->dist = u->dist + e->weight;
						v->timestamp = (*m_timestamp);
						pqB.insert( v->dist, v, &(v->pqBitem));
					}
					else if( (v->dist > u->dist + e->weight) )
					{
						v->pred = u->getDescriptor();
						v->dist = u->dist + e->weight;
						pqB.decrease( v->dist, &(v->pqBitem));
					}
				}
			}
			return tempT->dist;
		}
		        
        /**
         * @brief Calculate flags for the removed edges during contraction
         *
         * @param l The number of the current level
         */ 
        void giveFlagsToComponent( unsigned int l)
		{
			NodeIterator u, v;
			EdgeIterator e, lastEdge;
			ProgressStream node_progress( m_G.getNumNodes());
            node_progress.label() << "\tgive Flags To Component at level " << l;
			
			for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
			{
			    ++node_progress;
			    if( u->isOneShell == true)  continue;
				
				if( u->inactiveLevel == l)
				{   
					for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
					{
                        v = m_G.target(e);
                        if( v->isOneShell == true)  continue;
					    for( unsigned int level = l; level < m_numLevels; ++level)
						{
                            for( unsigned int y = 0; y < m_parts[level]; ++y)
                                e->flag[level][y] = 1;
						}                    
					}
				}        	
				else if( u->inactiveLevel > l)
				{
                    for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
					{   
						v = m_G.target(e);
                        if( v->inactiveLevel == l)
						{   
							for( unsigned int level = l; level < m_numLevels; ++level)
							    e->flag[level][v->cell[level]] = 1;
                        }
                	}
                }
            }
            for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
			{
			    for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				{
                    v = m_G.target(e);
                    if( (e->levelInsert == l) && (u->inactiveLevel > l) && (v->inactiveLevel > l))
					    e->flag[l][v->cell[l]] = 0;
                }
            }
		}
		
		/**
         * @brief Calculate flags for the remained edges after the contraction, creating a shortest path tree for every boundary node of each cell
         * tree for every boundary node of each cell
         *
         * @param l The number of the current level
         */         
        void giveFlagsToBoundaryNode( unsigned int l)
		{
			NodeIterator u, v, w, tempU, tempV;
			EdgeIterator e, lastEdge;
			std::vector<NodeDescriptor> boundaryNodes;
            unsigned int tempMin = 0;
            ProgressStream cell_progress( m_parts[l]);
            cell_progress.label() << "\tGiving flags to Core nodes at level " << l;

            for( unsigned short c = 0; c < m_parts[l]; ++c)
			{
                ++cell_progress;
                boundaryNodes.clear();
                pq.clear();                   
                unsigned int counter = 0;
                
                for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
                {			                            
                    u->labels.clear();
                    u->labels.resize( 50, std::numeric_limits<unsigned int>::max());
                    
                    if( (u->isOneShell == true) || (hasBeenBypassed( u, l)) || (u->cell[l] != c))   continue;
                                        
                    if( isBoundaryNode( u, l))
                    {
                        u->idBoundary = counter++;
                        boundaryNodes.push_back(m_G.getNodeDescriptor(u));    
                    }
                }
                
                unsigned int loops = (boundaryNodes.size() / 50) + 1;
                
                for( unsigned int z = 0; z < loops; ++z)
                {
                    if( z == loops - 1)
                    {
                        for( unsigned int i = 0; i < (boundaryNodes.size() - z*50); ++i)
                        {                        
                            u = m_G.getNodeIterator(boundaryNodes[z*50 + i]);
                            u->labels[i] = 0;

                            for( unsigned int y = 0; y < (boundaryNodes.size() - z*50); ++y)
                            {
                                v = m_G.getNodeIterator(boundaryNodes[z*50 + y]);
                                if( u == v)     continue;
                                u->labels[y] = getWeightOfHops( v, u, l);
                            }
                        }
                    }
                    else
                    {
                        for( unsigned int i = 0; i < 50; ++i)
                        {                        
                            u = m_G.getNodeIterator(boundaryNodes[z*50 + i]);
                            u->labels[i] = 0;

                            for( unsigned int y = 0; y < 50; ++y)
                            {
                                v = m_G.getNodeIterator(boundaryNodes[z*50 + y]);
                                if( u == v)     continue;
                                u->labels[y] = getWeightOfHops( v, u, l);
                            }
                        }
                    }

                    ++(*m_timestamp);
                    if( z == loops - 1)
                    {
                        for( unsigned int i = 0; i < (boundaryNodes.size() - z*50); ++i)
                        {
                            u = m_G.getNodeIterator(boundaryNodes[z*50 + i]);
                            u->timestamp = (*m_timestamp);
                            pq.insert( u->labels[i], u, &(u->pqitem));
                        }
                    }
                    else
                    {
                        for( unsigned int i = 0; i < 50; ++i)
                        {
                            u = m_G.getNodeIterator(boundaryNodes[z*50 + i]);
                            u->timestamp = (*m_timestamp);
                            pq.insert( u->labels[i], u, &(u->pqitem));
                        }
                    }

                    while( !pq.empty())
			        {
				        u = pq.minItem();
				        pq.popMin();

				        for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				        {
					        v = m_G.target(e);
                            if( (v->isOneShell == true) || (!isActiveNode(v, l)))  continue;
                        
                            if( v->timestamp < (*m_timestamp))
					        {
                                tempMin = std::numeric_limits<unsigned int>::max();
                                if( z == loops - 1)
                                {
                                    for( unsigned int b = 0; b < (boundaryNodes.size() - z*50); ++b)
                                    {
                                        v->labels[b] = u->labels[b] + e->weight;
                                        if( tempMin > v->labels[b])    tempMin = v->labels[b]; 
                                    }
                                }
                                else
                                {
                                    for( unsigned int b = 0; b < 50; ++b)
                                    {
                                        v->labels[b] = u->labels[b] + e->weight;
                                        if( tempMin > v->labels[b])    tempMin = v->labels[b]; 
                                    }
                                }
						        v->timestamp = (*m_timestamp);
						        pq.insert( tempMin, v, &(v->pqitem));
					        }
					        else
					        {
                                tempMin = std::numeric_limits<unsigned int>::max();
                                if( z == loops - 1)
                                {
                                    for( unsigned int b = 0; b < (boundaryNodes.size() - z*50); ++b)
                                    {
                                        if( v->labels[b] > u->labels[b] + e->weight)
                                        {
                                            v->labels[b] = u->labels[b] + e->weight; 
                                            if( tempMin > v->labels[b])    tempMin = v->labels[b];
                                        } 
                                    }
                                }
                                else
                                {
                                    for( unsigned int b = 0; b < 50; ++b)
                                    {
                                        if( v->labels[b] > u->labels[b] + e->weight)
                                        {
                                            v->labels[b] = u->labels[b] + e->weight; 
                                            if( tempMin > v->labels[b])    tempMin = v->labels[b];
                                        } 
                                    }
                                }
                                
                                if( tempMin != std::numeric_limits<unsigned int>::max())
                                {
                                    if( pq.isMember(&(v->pqitem)))
                                        pq.decrease( tempMin, &(v->pqitem));
                                    else
                                        pq.insert( tempMin, v, &(v->pqitem));
                                }
                            }
				        }
			        }
                     
                    for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
                    {
                        for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				        {
                            v = m_G.target(e);
                            if( (v->isOneShell == true) || (!isActiveNode(v, l)))  continue;

                            if( z == loops - 1)
                            {
                                for( unsigned int b = 0; b < (boundaryNodes.size() - z*50); ++b)
                                {
                                    if( u->labels[b] + e->weight == v->labels[b])
                                    {
                                        if( m_G.hasEdge(v, u))
                                            m_G.getEdgeIterator( v, u)->flag[l][c] = 1;                               
                                    }
                                }
                            }
                            else
                            {
                                for( unsigned int b = 0; b < 50; ++b)
                                {
                                    if( u->labels[b] + e->weight == v->labels[b])
                                    {
                                        if( m_G.hasEdge(v, u))
                                            m_G.getEdgeIterator( v, u)->flag[l][c] = 1;                               
                                    }
                                }
                            }
                            if( (u->cell[l] == c) && (v->cell[l] == c))     e->flag[l][c] = 1;
                        }
                    } 
                }
            }  
        }
    
		/**
         * @brief Finds if node u has been contracted at level l
         *
         * @param u The node u
         * @param l The number of the current level
         *
         * @return True if u has been contracted, false otherwise 
         */ 
		bool hasBeenBypassed( NodeIterator u, unsigned int l)
        {
            return u->inactiveLevel <= l;
        }
		
		/**
         * @brief Insertion of shortcuts between boundary nodes at level l
         *
         * @param l The number of the current level which is L-1
         *
         */ 
		void insertBoundaryShortcuts( unsigned int l)
		{
			NodeIterator u, v, w;
			EdgeIterator e, e2, lastEdge;
            unsigned int count = 0, maxBetweenness = 0, tempWeight = 0;
			
			std::vector< std::vector<NodeDescriptor> > boundaryNodes;
			boundaryNodes.resize( m_parts[l]);
			
			for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
			{	 
                if( (u->isOneShell == true) || (hasBeenBypassed( u, l)))   continue; 
				if( isBoundaryNode( u, l))
				{
					unsigned int maxShortcuts = getNumActiveNeigh( u, l);					
					u->key = std::numeric_limits<unsigned int>::max() - maxShortcuts;
					if( maxBetweenness < maxShortcuts)	maxBetweenness = maxShortcuts;
					boundaryNodes[u->cell[l]].push_back( m_G.getNodeDescriptor(u));
				}
			}
			for( unsigned int cell = 0; cell < m_parts[l]; ++cell)
			{
			    ProgressStream nodes_progress( boundaryNodes[cell].size());
                nodes_progress.label() << "\tinsertBoundaryShortcuts at cell " << cell;
				  
				for( unsigned int i = 0;  i < boundaryNodes[cell].size(); ++i)
				{	
				    ++nodes_progress;
					pq.clear();			
                    u = m_G.getNodeIterator( boundaryNodes[cell][i]);
					if( (std::numeric_limits<unsigned int>::max() - u->key) > (maxBetweenness/2))
					{
		                for( unsigned int y = 0;  y < boundaryNodes[cell].size(); ++y)
						{
							v = m_G.getNodeIterator(boundaryNodes[cell][y]);
							if( u == v) continue;
							v->dist = (std::numeric_limits<unsigned int>::max() - (getNumOfHops( u, v) * getNumActiveNeigh( v, l)));
							pq.insert( v->dist, v, &(v->pqitem));
						}
						unsigned int outgoingEdges = (unsigned int)(3.0 * sqrt( (double)getNumOfBoundaryOfCell( u, l)));
						
						while( (!pq.empty()) && (outgoingEdges > 0))
						{
							w = pq.minItem();
							pq.popMin();
							--outgoingEdges;    
							
                            if( m_G.hasEdge( u, w))
								++outgoingEdges;
							else            
							{
                                tempWeight = getWeightOfHops( u, w, l);
							    e = m_G.getEdgeIterator(m_G.insertEdge( m_G.getNodeDescriptor(u), m_G.getNodeDescriptor(w)));
		                        e->weight = tempWeight;
								++count;								
                                e->flag.resize( m_numLevels);

                                for( unsigned int level = 0; level < m_numLevels; ++level)
                                    e->flag[level].resize( m_parts[level]);
                            }	
						}
					}
	    		}
			}
            std::cout<<"\tSto insertBoundaryShortcuts prosthethikan " << count << " akmes." << std::endl;
		}
		
        /**
         * @brief Checks if node u is active (not contracted) at level l
         *
         * @param u The checking node
         * @param l The number of the current level
         *
         * @return True if u is active, false otherwise 
         */ 
		bool isActiveNode( NodeIterator u, unsigned int l)
		{
			if ( u->inactiveLevel > l)
				return true;
			else	
				return false;
		}
		
		/**
         * @brief Checks if node u is a boundary node
         *
         * @param u The checking node
         * @param l The number of the current level
         *
         * @return True if u is boundary node, false otherwise 
         */ 
		bool isBoundaryNode( NodeIterator u, unsigned int l)
		{
			NodeIterator v;
			std::vector<NodeDescriptor> neighbors;
			getOutNeighbors( u, neighbors);
         
			for( unsigned int i = 0; i < neighbors.size(); ++i)
			{
                v = m_G.getNodeIterator(neighbors[i]);
                if( (v->isOneShell == true) || (!isActiveNode( v, l)))   continue;
                if( u->cell[l] != v->cell[l])  return true;
			}
            return false;
		}

        /**
         * @brief Checks if node u is suitable to be contracted
         *
         * @param u The checking node
         * @param l The number of the current level
         *
         * @return True if u is suitable, false otherwise 
         */       
        bool isBypassableNode( NodeIterator u, unsigned int l)
		{
            if( isBoundaryNode( u, l))  return false;
			
            NodeIterator v, w;
			EdgeIterator e, e1, e2;
            unsigned int count = 0, maxHops = 0;
            std::vector<NodeDescriptor> neighbors;
			getOutNeighbors( u, neighbors);

            for( unsigned int i = 0;  i < neighbors.size(); ++i)
			{
				w = m_G.getNodeIterator(neighbors[i]);
				if( (w->isOneShell == true) || (!isActiveNode(w, l)))  continue;
                e1 = m_G.getEdgeIterator( w, u);

				for( unsigned int y = 0;  y < neighbors.size(); ++y)
				{
					v = m_G.getNodeIterator(neighbors[y]);
					if( (v->isOneShell == true) || (!isActiveNode(v, l)))  continue;
					if( v == w)	continue;
					e2 = m_G.getEdgeIterator( u, v); 
					unsigned int viaWeight = e1->weight + e2->weight;
                    unsigned int countHops = e1->hops + e2->hops;
					if( m_G.hasEdge( w, v))
                    {
                        if( viaWeight < m_G.getEdgeIterator( w, v)->weight)
						{
                            ++count;
                            if( countHops > maxHops)    maxHops = countHops;
                        }
                    }
                    else
                    {
                        ++count;    
                        if( countHops > maxHops)	maxHops = countHops;
                    }
				}
			}
			if ( (maxHops <= MAX_HOPS) && (count <= (CONTRACTION_PARAMETER * 2 * getNumActiveNeigh( u, l))) && (count > 0))
			{
				u->key = (( maxHops * count) / ( 2 * getNumActiveNeigh( u, l)));
				return true;
			}
           	else
				return false;
		}

        /**
         * @brief Check for cells suitable for prunning their edges
         *
         * @param l The number of the current level
         *
         */   
        void findPruningCells( unsigned int l)
        {
            NodeIterator u, v;
            EdgeIterator e, lastEdge;
			std::vector<unsigned int> pruningCells;
            std::vector<NodeDescriptor> neighbors;
            unsigned int numCells = m_parts[l];   
            unsigned int i = 0, c0 = 0;
            
            next:
            for( ; i < numCells; ++i)
			{
                for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
				{
                    if( u->cell[l] == i)
                    {
                        getOutNeighbors( u, neighbors);
                        for( unsigned int y = 0;  y < neighbors.size(); ++y)
			            {
				            v = m_G.getNodeIterator(neighbors[y]);
                            if( u->cell[l] != v->cell[l])
                            {
                                if( u->cell[l+1] != v->cell[l+1])  
                                {
                                    ++i;
                                    goto next;
                                }
                            }
                        }
                    }
                }
                pruningCells.push_back(i);
            }

            for( unsigned int c = 0;  c < pruningCells.size(); ++c)
			{
                for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
				{
                    if( (u->isOneShell == true) || (!isActiveNode(u, l)) || (u->cell[l] != pruningCells[c]))  continue;
                    for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
					{
                        v = m_G.target(e);
                        if( v->cell[l] != pruningCells[c])  continue;
                        
                        unsigned int temp = 0;
                        for( unsigned int y = 0; y < m_parts[l]; ++y)
                            temp += e->flag[l][y];
						
                        if( (temp == 1) && (e->flag[l][v->cell[l]] == 1))
						{
							++c0;
							e->isActive = false;
						}									
					}   
                }
            }
            std::cout<<"\tPruning at level " << l << " edge removed " << c0 << std::endl;
        }
		
		/**
         * @brief Routine for refinement arc-flags by propagation of arc-flags from higher
         * to lower levels
         *
         */  
		void refinementArcFlags()
		{
			NodeIterator u, v, s, t;
			EdgeIterator e, uxEdge, lastEdge;
			unsigned int count = 0;
            std::vector<NodeDescriptor> exitNodes;
			
			for( unsigned int level = 0; level < m_numLevels; ++level)
			{
			    unsigned int l = (m_numLevels - 1) - level;
                ProgressStream node_progress( m_G.getNumNodes());
				node_progress.label() << "\trefinementArcFlags at level" << l;

				for( s = m_G.beginNodes(); s != m_G.endNodes(); ++s)
				{
                    ++node_progress;
                    if( (s->isOneShell == true) || (s->inactiveLevel != l))  continue;
					exitNodes.clear();
                    exitNodes = returnExitNodes( s, l);	
                    if( exitNodes.size() == 0)	continue;
					
                    for( e = m_G.beginEdges(s), lastEdge = m_G.endEdges(s); e != lastEdge; ++e)
					{
					    v =  m_G.target(e);
						for( unsigned int lu = s->inactiveLevel; lu < m_numLevels; ++lu)
							e->flag[lu][v->cell[lu]] = 1;
					}
						
					for( unsigned int i = 0; i < exitNodes.size(); ++i)
					{
                        t = m_G.getNodeIterator(exitNodes[i]);
						while( t != s)
						{
                            u = m_G.getNodeIterator((NodeDescriptor)t->pred);
                            if( u == s)
                            {
                                uxEdge = m_G.getEdgeIterator( u, t);
                                for( e = m_G.beginEdges(m_G.getNodeIterator(exitNodes[i])); e != m_G.endEdges(m_G.getNodeIterator(exitNodes[i])); ++e)
						        {
                                    for( unsigned int y = 0; y < m_parts[l]; ++y)
                                    {
						                uxEdge->flag[l][y] = uxEdge->flag[l][y] || e->flag[l][y];
                                        ++count;
                                    }
							    }	
                            }
							t = u;
						}         
					}
				}
			}
			std::cout<<"\tAt refinementArcFlags modified " << count << " edges " << std::endl;	
		}	

        /**
         * @brief Find the exit nodes of node
         *
         * @param s The checking node
         * @param l The number of the current level
         *
         * @return A vector of exit nodes 
         */  
        std::vector<NodeDescriptor> returnExitNodes( NodeIterator s, unsigned int l)
		{
			NodeIterator u, v, w;
			EdgeIterator e, lastEdge; 
			init(s);
			std::vector<NodeDescriptor> nodes;
            nodes.clear();
					
			while( !pq.empty())
			{
				u = pq.minItem();
				pq.popMin();
				
				if( u != s)
				{  
				    w = m_G.getNodeIterator((NodeDescriptor)u->pred);
                    if( w->inactiveLevel > s->inactiveLevel)
                    {	
                        nodes.push_back(m_G.getNodeDescriptor(w));
                        continue;
                    }
				}
				for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				{
					v = m_G.target(e);
                    if( (v->isOneShell == true) || (v->inactiveLevel < s->inactiveLevel))  continue;
                    
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
            return nodes;
         }		
        
        /**
        * @brief Remove 1-shell nodes from the graph
        *
        */  
        void removeOneShellNodes()
		{
			NodeIterator u,v;
			EdgeIterator e;
			unsigned int c0 = 0, c1 = 0;
            std::vector<NodeDescriptor> neighbors;
                        
			for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
			{
				++c0;
                getOutNeighbors( u, neighbors);
				if( neighbors.size() < 2)
				{
				    u->isOneShell = true;
					++c1;
				}
			}
			std::cout<<"\tTotal nodes " << c0 << " one shell nodes " << c1 << std::endl;
			giveFlagsToOneShell();
		}

        /**
        * @brief Calculate flags for edges of 1-shell nodes
        *
        */ 
        void giveFlagsToOneShell()
		{
			NodeIterator u, v;
			EdgeIterator e, lastEdge;
            ProgressStream node_progress( m_G.getNumNodes());
            node_progress.label() << "\tgive Flags To 1-Shell Nodes";
			
			for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
		    {
				++node_progress;
                if( u->isOneShell == false)  //core
                {    
				    for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				    {
				        v = m_G.target(e);
					    if( v->isOneShell == true)
					    {
                            for( unsigned int l = 0; l < m_numLevels; ++l)
					            e->flag[l][v->cell[l]] = 1;
					    }	
                    }
                }
                else
                {
                    for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				    {          
                        for( unsigned int l = 0; l < m_numLevels; ++l)
				        {
                            for( unsigned int y = 0; y < m_parts[l]; ++y)
                                e->flag[l][y] = 1;
					    }
                    }
                }
            }
        }

    
		/**
        * @brief Performs all steps of algorithm
        *
        */ 
		void runPreprocessing()
		{	
            calcWeightsAndInitFlags();
            removeOneShellNodes();
            
            for( unsigned short l = 0; l < m_numLevels; ++l)
			{   
                std::cout<<"\n\t---------------------------------------------------------------" << std::endl;	
                contractNodes(l);
                //returnNumComponents(l);
				edgeReduction(l);
				giveFlagsToComponent(l);
				if( l == (m_numLevels - 1))
					insertBoundaryShortcuts(l);
				giveFlagsToBoundaryNode(l);
				std::cout<<"\tTotal cells " << m_parts[l] << std::endl;
                if( l < (m_numLevels - 1))
                    findPruningCells(l);
	    	}
			std::cout<<"\t---------------------------------------------------------------" << std::endl;
			refinementArcFlags();
            writeEdgesToFile();		
        }
		//////////////////////////////////////////////////////////////////////////
        
 
        /**
        * @brief Create for all edges a file with their flags and their weight
        *
        */ 
        void writeEdgesToFile()
		{
			NodeIterator u, v;
			EdgeIterator e, lastEdge;
			unsigned int c0 = 0;
            
			ProgressStream node_progress( m_G.getNumNodes());
            node_progress.label() << "\tWriting to file";

            std::ofstream out;
			out.open("EdgesWithFlags_luxembourg.txt", std::ios::out | std::ios::binary);

            for( u = m_G.beginNodes(); u != m_G.endNodes(); ++u)
			{
                ++node_progress;
				for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
				{
					++c0;
					v = m_G.target(e);
                    
                    out.write( (char *) &(u->id), sizeof(unsigned int));
                    out.write( (char *) &(v->id), sizeof(unsigned int));
                    out.write( (char *) &(e->weight), sizeof(unsigned int));
					
                    for( unsigned int l = 0; l < m_numLevels; ++l)
                    {
                        for( unsigned int y = 0; y < m_parts[l]; ++y)
                        {
                            bool flag = e->flag[l][y];
                            out.write( (char *) &(flag), sizeof(bool));
                        }
                    }
				}
			}
			out.close();
			std::cout<<"\tTotal edges " << c0 <<std::endl;
		}


        /**
         * @brief Returns the number of the nodes that have been settled (enqueued and dequeued) by the algorithm.
         * @return The number of the settled nodes.
         */
        const unsigned int& getNumSettledNodes()
        {
            return m_numSettledNodes;
        }


        /**
         * @brief Returns the number of the nodes that have been visited by the algorithm.
         * @return The number of the settled nodes.
         */
        const unsigned int getNumVisitedNodes()
        {
            return ( m_numSettledNodes + pq.size());
        }


        /**
         * @brief Returns the number of the nodes that have not been visited by the algorithm.
         * @return The number of the no settled nodes.
         */
        const unsigned int getNumUnvisitedNodes()
        {
            return ( m_G.getNumNodes() - getNumVisitedNodes());
        }


        /**
         * @brief Returns the number of the nodes of the shortest path of a query.
         * @return The number of nodes of the shortest path.
         */
        const unsigned int getNumSPNodes( const typename GraphType::NodeIterator& t)
        {
            NodeIterator u = t;
            unsigned int numSPNodes = 1;
            //s -- v -- t 
            //1 + the rest nodes in sp
            while( u->pred != m_G.nilNodeDescriptor())
            {
                numSPNodes++;
                u = m_G.getNodeIterator(u->pred);
            }

            return numSPNodes;
        }

        
        /**
         * @brief Prints the shortest path and for every node prints his cell at each level.
         */
        void printShortestPath( const NodeIterator& s, const NodeIterator& t)
        {
            unsigned int numSpNodes = 1;
            NodeIterator u = s;
            NodeIterator v = t;
            NodeIterator w;
            EdgeIterator e;
            
            std::ofstream out;
			out.open("printSP_luxembourg.txt");
            
            std::cout<<"\ts-> " << s->id << " and t-> " << t->id << std::endl;
            while( v != u)
            {
                if( v->pred == m_G.nilNodeDescriptor())     break;
                w = m_G.getNodeIterator((NodeDescriptor)v->pred);
                e = m_G.getEdgeIterator(w, v);
                out << w->id;
                for( unsigned int l = 0; l < m_numLevels; ++l)
                    out << " " << w->cell[l];
                out << " --> " << v->id;
                for( unsigned int l = 0; l < m_numLevels; ++l)
                    out << " " << v->cell[l];
                out << " - " << e->weight;
                out << std::endl;
                numSpNodes++;
                v = w;
            }
            out.close();
            std::cout << "\nSp length: " << t->dist;
            std::cout << "\nSp num nodes: " << numSpNodes << "\n";
        }


        /**
         * @brief Prints only the shortest path.
         */
        void printShortestPath2( const NodeIterator& s, const NodeIterator& t)
        {
            unsigned int numSpNodes = 1;
            NodeIterator u = s;
            NodeIterator v = t;
            NodeIterator w;
            std::vector<NodeDescriptor> nodes;
            nodes.clear();
            
            nodes.push_back(m_G.getNodeDescriptor(v));
            while( v != u)
            {
                w = m_G.getNodeIterator((NodeDescriptor)v->pred);
                nodes.push_back(m_G.getNodeDescriptor(w));
                numSpNodes++;
                v = w;
            }
            
            std::ofstream out;
			out.open("printSP_luxembourg.txt");
            for(unsigned int i = 0; i < nodes.size(); i++)
            {   
                w = m_G.getNodeIterator(nodes[i]);
                //std::cout << "[" << w->id << " || "<< w->dist << "]->\n";
                out << w->x << " " << w->y << std::endl;
            }
			out.close();
            //std::cout << "\nSp length: " << t->dist;
            std::cout << "\nSp num nodes: " << numSpNodes << "\n";
        }
        
        
        /**
         * @brief Initialization for the query
         */
        void init( const NodeIterator s)
        {
            pq.clear();
            m_numSettledNodes = 0;
            ++(*m_timestamp);
            s->dist = 0;
            s->timestamp = (*m_timestamp);
            s->pred = m_G.nilNodeDescriptor();
            pq.insert( s->dist, s, &(s->pqitem));
        }

        
        /**
         * @brief Computes the lowest level on which node1 and node2 are in the same cell
         * @return The number of the lowest level on which node1 and node2 are in common cell
         */
        unsigned int getCommonLevel( NodeIterator node1, NodeIterator node2)
	    {		
            for( unsigned int l = 0; l < m_numLevels;  ++l)
		    {
                if( node1->cell[l] == node2->cell[l])
			        return l;
		    }
		    return m_numLevels;
	    }
	
		
        /**
         * @brief The s-t query
         */
		WeightType runQuery( const NodeIterator& s, const NodeIterator& t)
	    {
		    NodeIterator u, v, lastNode;
		    EdgeIterator e, lastEdge;
		    init(s);
			WeightType discoveredDistance;
            unsigned int level = 0;

		    while( !pq.empty())
		    {
			    u = pq.minItem();
			    pq.popMin();
                ++m_numSettledNodes;
                if( u == t)    break;
                level = getCommonLevel( u, t);

                if ( level > 0)
                {
                    --level;
                    for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
			        {	
                        if( e->flag[level][t->cell[level]] == 0)   continue;

				        discoveredDistance = u->dist + e->weight;
                        v = m_G.target(e);
                        if( v->timestamp < (*m_timestamp))
                        {
                            v->pred = u->getDescriptor();
                            v->timestamp = (*m_timestamp);
                            v->dist = discoveredDistance;
                            pq.insert( discoveredDistance, v, &(v->pqitem));
                        }    
                        else if( v->dist > discoveredDistance)
                        {
                            v->pred = u->getDescriptor();
                            v->dist = discoveredDistance;
                            pq.decrease( discoveredDistance, &(v->pqitem));
                        }
			        }
                }
                else if( level == 0)
                {
			        for( e = m_G.beginEdges(u), lastEdge = m_G.endEdges(u); e != lastEdge; ++e)
                    {
                        discoveredDistance = u->dist + e->weight;
                        
                        v = m_G.target(e);
                        if( v->timestamp < (*m_timestamp))
                        {
                            v->pred = u->getDescriptor();
                            v->timestamp = (*m_timestamp);
                            v->dist = discoveredDistance;
                            pq.insert( discoveredDistance, v, &(v->pqitem));
                            
                        }
                        else if( v->dist > discoveredDistance)
                        {
                            v->pred = u->getDescriptor();
                            v->dist = discoveredDistance;
                            pq.decrease( discoveredDistance, &(v->pqitem));
                        }
                    }
				}
		    }
            //std::cout<< "\t-->Sp ["<< s->id << "] --> [" << t->id << "]<-- Dist:" << t->dist <<std::endl;
            //std::cout<<"\tSettled nodes: " << m_numSettledNodes << " Common level: " << m_partition.getLevelOfCommonCell( s->x, s->y, t->x, t->y) << std::endl;
			//std::cout<<"\tCommon level: " << m_partition.getLevelOfCommonCell( s->x, s->y, t->x, t->y) << std::endl;
            //if( t->dist == std::numeric_limits<unsigned int>::max())    std::cout<<"\n\tkeli s-> "<< s->cell[level] << " keli t-> " << t->cell[level] << std::endl;
            
            //printShortestPath(s, t);
            return t->dist;
	    }


	private:
		GraphType& m_G;
		unsigned short m_numLevels;
		unsigned int* m_timestamp;
		unsigned int m_numSettledNodes;
		PriorityQueueType pq, pqB;
        std::vector<unsigned short> &m_parts;
        
};
