#ifndef STATIC_FORWARDSTARIMPL_H
#define STATIC_FORWARDSTARIMPL_H

#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Sets/bucketSet.h>
#include <Utilities/mersenneTwister.h>
#include <vector>
#include <stack>
#include <cstring>

template<typename Vtype, typename Etype, typename InEtype>
class StaticFSNode;
template<typename Vtype, typename Etype, typename InEtype>
class StaticFSEdge;
template<typename Vtype, typename Etype, typename InEtype>
class StaticFSInEdge;

template<typename Vtype, typename Etype, typename InEtype>
class StaticForwardStarImpl
{
public:

    typedef unsigned int                                                         SizeType;
    typedef typename std::vector< StaticFSNode< Vtype, Etype, InEtype> >::iterator     NodeIterator;
    typedef typename std::vector< StaticFSEdge< Vtype, Etype, InEtype> >::iterator     EdgeIterator;
    typedef typename std::vector< StaticFSInEdge< Vtype, Etype, InEtype> >::iterator   InEdgeIterator;
    typedef NodeIterator*                                                        NodeDescriptor;

    StaticForwardStarImpl():m_numNodes(0),m_numEdges(0)
    {
    }

    ~StaticForwardStarImpl()
    {
         clear();
    }

    EdgeIterator beginEdges( const NodeIterator& u) const
    {
        return u->m_edges.begin();
    }

    InEdgeIterator beginInEdges( const NodeIterator& u) const
    {
        return u->m_inEdges.begin();
    }

    NodeIterator beginNodes()
    {
        return m_nodes.begin();
    }

    NodeIterator chooseNode()
	{
	    double random = m_random.getRandomNormalizedDouble();
        SizeType pos = m_nodes.size() * random;

        NodeIterator m_auxNodeIterator = m_nodes.begin();

        std::advance( m_auxNodeIterator, pos);
        return m_auxNodeIterator;
	}

    void clear()
    {
        m_numNodes = 0;
        m_numEdges = 0;

        m_nodes.clear();
        m_edgeSet.clear();
        m_inEdgeSet.clear();
    }

    void compress()
    {
        BucketSet< StaticFSEdge<Vtype,Etype,InEtype> > outEdgeSet( m_numEdges);
        BucketSet< StaticFSInEdge<Vtype,Etype,InEtype> >  inEdgeSet( m_numEdges);
        std::vector< StaticFSNode<Vtype,Etype,InEtype> >  nodeSet;
        nodeSet.resize( m_numNodes);
        nodeSet.shrink_to_fit();
        m_nodeIters.resize( m_numNodes);
        m_nodeIters.shrink_to_fit();
        std::map<NodeIterator, NodeIterator> old2new;

        for( std::size_t i=0, j=0; i<m_nodes.size(); i++)
        {
            if( m_nodes[i].getDescriptor() != 0)
            {
                NodeIterator src = (m_nodes.begin()+i);

                std::memcpy( (void*) &(nodeSet[j]), (void*)& (*src), sizeof( *src));
                NodeIterator dst = (nodeSet.begin() + j);
                m_nodeIters[j] = dst;
                old2new[src] = dst;

                NodeDescriptor dsc = &( m_nodeIters[j]);
                dst->setDescriptor( dsc);
                j++;

                std::size_t numOutEdges = outdeg(src);
                outEdgeSet.registerBucket(&(dst->m_edges), numOutEdges);
                (src->m_edges).copyTo(dst->m_edges);
                (dst->m_edges).m_bucketSet = &m_edgeSet;

                std::size_t numInEdges = indeg(src);
                inEdgeSet.registerBucket(&(dst->m_inEdges), numInEdges);
                (src->m_inEdges).copyTo(dst->m_inEdges);
                (dst->m_inEdges).m_bucketSet = &m_inEdgeSet;
            }
        }

        m_nodes.swap( nodeSet);
        m_edgeSet.swap( outEdgeSet);
        m_inEdgeSet.swap( inEdgeSet);

        for( NodeIterator u=beginNodes(), endNode=endNodes(); u != endNode; ++u)
        {
            for( EdgeIterator e = beginEdges(u), endEdge = endEdges(u); e != endEdge; ++e)
                e->m_adjacentNode = old2new[e->m_adjacentNode];

            for( InEdgeIterator k = beginInEdges(u), endInEdge = endInEdges(u); k != endInEdge; ++k)
                k->m_adjacentNode = old2new[k->m_adjacentNode];
        }
    }

    void dfsCompress_orig()
    {
        std::vector<NodeIterator> reorderedNodes;
        reorderedNodes.reserve( m_numNodes);

        std::map<NodeIterator, bool> isVisited;
        for( NodeIterator u=beginNodes(), endNode=endNodes(); u != endNode; ++u)
            isVisited[u]=false;

        NodeIterator x=beginNodes(), endNode=endNodes();

        std::stack<NodeIterator> S;
        NodeIterator s = (beginNodes() + (m_numNodes >> 1)); //chooseNode();
        if( s->getDescriptor() != 0)
        {
            isVisited[s] = true;
            reorderedNodes.push_back(s);
            S.push(s);
        }

        dfsOrdering:
        while( !S.empty())
        {
            NodeIterator u = S.top();
            const std::size_t ssize = S.size();

            for( EdgeIterator e = beginEdges( u), endEdge = endEdges( u); e != endEdge; ++e)
            {
                NodeIterator v = getAdjacentNodeIterator(e);
                if( v->getDescriptor() == 0)
                    continue;

                if( isVisited[v] == false)
                {
                    isVisited[v] = true;
                    S.push( v);
                    reorderedNodes.push_back(v);
                    //break;
                }
            }

            if( ssize == S.size())
            {
                S.pop();
            }
        }

        for(; x!= endNode; ++x)
            if( isVisited[x] == false)
            {
                if( x->getDescriptor() == 0)
                    continue;
                isVisited[x] = true;
                S.push(x);
                reorderedNodes.push_back(x);
                goto dfsOrdering;
            }

        BucketSet< StaticFSEdge<Vtype,Etype,InEtype> > outEdgeSet( m_numEdges);
        BucketSet< StaticFSInEdge<Vtype,Etype,InEtype> >  inEdgeSet( m_numEdges);
        std::vector< StaticFSNode<Vtype,Etype,InEtype> >  nodeSet;
        nodeSet.resize( m_numNodes);
        nodeSet.shrink_to_fit();
        m_nodeIters.resize( m_numNodes);
        m_nodeIters.shrink_to_fit();
        std::map<NodeIterator, NodeIterator> old2new;
        for( std::size_t i=0, j=0; i<reorderedNodes.size(); i++)
        {
            NodeIterator src = reorderedNodes[i];

            std::memcpy( (void*) &(nodeSet[j]), (void*) &(*src), sizeof( *src));
            NodeIterator dst = (nodeSet.begin() + j);
            m_nodeIters[j] = dst;
            old2new[src] = dst;

            NodeDescriptor dsc = &(m_nodeIters[j]);
            dst->setDescriptor( dsc);
            j++;

            std::size_t numOutEdges = outdeg(src);
            outEdgeSet.registerBucket(&(dst->m_edges), numOutEdges);
            (src->m_edges).copyTo(dst->m_edges);
            (dst->m_edges).m_bucketSet = &m_edgeSet;

            std::size_t numInEdges = indeg(src);
            inEdgeSet.registerBucket(&(dst->m_inEdges), numInEdges);
            (src->m_inEdges).copyTo(dst->m_inEdges);
            (dst->m_inEdges).m_bucketSet = &m_inEdgeSet;
        }

        m_edgeSet.swap( outEdgeSet);
        m_inEdgeSet.swap( inEdgeSet);
        m_nodes.swap( nodeSet);

        for( NodeIterator u=beginNodes(), endNode=endNodes(); u != endNode; u++)
        {
            for( EdgeIterator e = beginEdges(u), endEdge = endEdges(u); e != endEdge; ++e)
                e->m_adjacentNode = old2new[e->m_adjacentNode];

            for( InEdgeIterator k = beginInEdges(u), endInEdge = endInEdges(u); k != endInEdge; ++k)
                k->m_adjacentNode = old2new[k->m_adjacentNode];
        }
    }

    void dfsCompress()
    {
        NodeIterator s = (beginNodes() + (m_numNodes >> 1));
        dfsCompress( s);
    }
    
    void dfsCompress( const NodeIterator& s)
    {
        std::vector<NodeIterator> reorderedNodes;
        reorderedNodes.reserve( m_numNodes);

        //dfs node ordering by visit time depth and breadth
        std::map<NodeIterator, bool> isVisited;
        for( NodeIterator u=beginNodes(), endNode=endNodes(); u != endNode; ++u)
            isVisited[u]=false;

        NodeIterator x=beginNodes(), endNode=endNodes();

        std::stack<NodeIterator> S;
        if( s->getDescriptor() != 0)
        {
            isVisited[s] = true;
            reorderedNodes.push_back(s);
            S.push(s);
        }

        dfsOrdering:
        while( !S.empty())
        {
            NodeIterator u = S.top();
            const std::size_t ssize = S.size();

            for( EdgeIterator e = beginEdges( u), endEdge = endEdges( u); e != endEdge; ++e)
            {
                NodeIterator v = getAdjacentNodeIterator(e);
                if( v->getDescriptor() == 0)
                    continue;

                if( isVisited[v] == false)
                {
                    isVisited[v] = true;
                    S.push( v);
                    reorderedNodes.push_back(v);
                    //break;
                }
            }

            if( ssize == S.size())
            {
                S.pop();
            }
        }

        for(; x!= endNode; ++x)
        {
            if( isVisited[x] == false)
            {
                if( x->getDescriptor() == 0)
                    continue;

                isVisited[x] = true;
                S.push(x);
                reorderedNodes.push_back(x);
                goto dfsOrdering;
            }
        }

        isVisited.clear();
    
        compress( reorderedNodes);
    }

    void compress( std::vector<NodeIterator>& reorderedNodes)
    {
        //nodes
        std::vector< StaticFSNode<Vtype,Etype,InEtype> >  nodeSet;
        std::map<NodeIterator, NodeIterator> old2new;
        nodeSet.resize( m_numNodes);
        nodeSet.shrink_to_fit();
        m_nodeIters.resize( m_numNodes);
        m_nodeIters.shrink_to_fit();
        for( std::size_t i=0, j=0; i<reorderedNodes.size(); i++)
        {
            NodeIterator src = reorderedNodes[i];

            std::memcpy( (void*) &(nodeSet[j]), (void*) &(*src), sizeof( *src));
            NodeIterator dst = (nodeSet.begin() + j);
            m_nodeIters[j] = dst;
            old2new[src] = dst;

            NodeDescriptor dsc = &(m_nodeIters[j]);
            dst->setDescriptor( dsc);
            j++;
        }

        m_nodes.swap( nodeSet);
        nodeSet.clear();
        nodeSet.shrink_to_fit();
        reorderedNodes.clear();
        reorderedNodes.shrink_to_fit();

        {
            typedef BucketSet< StaticFSEdge<Vtype,Etype,InEtype> > EdgeBucketSet;
            EdgeBucketSet outEdgeSet( m_numEdges);

            for( std::size_t i=0; i<m_nodes.size(); i++)
            {
                NodeIterator dst = (m_nodes.begin()+i);
                typename EdgeBucketSet::Bucket edgeBucket = dst->m_edges;
                std::size_t numOutEdges = edgeBucket.size();
                outEdgeSet.registerBucket(&(dst->m_edges), numOutEdges);
                edgeBucket.copyTo(dst->m_edges);
                (dst->m_edges).m_bucketSet = &m_edgeSet;
            }

            m_edgeSet.swap( outEdgeSet);
        }

        {
            typedef BucketSet< StaticFSInEdge<Vtype,Etype,InEtype> > InEdgeBucketSet;
            InEdgeBucketSet inEdgeSet( m_numEdges);

            for( std::size_t i=0; i<m_nodes.size(); i++)
            {
                NodeIterator dst = (m_nodes.begin()+i);
                typename InEdgeBucketSet::Bucket inedgeBucket = dst->m_inEdges;
                std::size_t numInEdges = inedgeBucket.size();
                inEdgeSet.registerBucket(&(dst->m_inEdges), numInEdges);
                inedgeBucket.copyTo(dst->m_inEdges);
                (dst->m_inEdges).m_bucketSet = &m_inEdgeSet;
            }

            m_inEdgeSet.swap( inEdgeSet);
        }

        for( NodeIterator u=beginNodes(), endNode=endNodes(); u != endNode; u++)
        {
            for( EdgeIterator e = beginEdges(u), endEdge = endEdges(u); e != endEdge; ++e)
                e->m_adjacentNode = old2new[e->m_adjacentNode];

            for( InEdgeIterator k = beginInEdges(u), endInEdge = endInEdges(u); k != endInEdge; ++k)
                k->m_adjacentNode = old2new[k->m_adjacentNode];
        }
    }

    void eraseNode( NodeDescriptor& descriptor)
    {
        NodeIterator v = getNodeIterator( descriptor);
        //std::swap(*lastNode, *it);
        v->setDescriptor( 0);
        --m_numNodes;
    }

    EdgeIterator endEdges( const NodeIterator& u) const
    {
        return u->m_edges.end();
    }

    InEdgeIterator endInEdges( const NodeIterator& u) const
    {
        return u->m_inEdges.end();
    }

    NodeIterator endNodes()
    {
        return m_nodes.end();
    }

    void eraseEdge( NodeDescriptor& uD, NodeDescriptor& vD)
    {
        EdgeIterator e = getEdgeIterator( uD, vD);
        NodeIterator v = getNodeIterator( vD);
        InEdgeIterator k = getInEdgeIterator( e);
        NodeIterator u = getNodeIterator( uD);

        u->m_edges.erase(e);
        v->m_inEdges.erase(k);
        --m_numEdges;

        for( EdgeIterator endEdge = endEdges(u); e != endEdge; ++e)
        {
            InEdgeIterator y = beginInEdges(e->m_adjacentNode) + e->m_InEdge;
            y->m_edge--;
        }

        for( InEdgeIterator endInEdge = endInEdges(v); k != endInEdge; ++k)
        {
            EdgeIterator x = beginEdges(k->m_adjacentNode) + k->m_edge;
            x->m_InEdge--;
        }
    }

    void eraseEdge( const InEdgeIterator& k)
    {
        eraseEdge( getEdgeIterator(k));
    }

    void expand()
    {
    }

    NodeIterator getAdjacentNodeIterator( const EdgeIterator& e) const
    {
        return e->m_adjacentNode;
    }

    NodeIterator getAdjacentNodeIterator( const InEdgeIterator& k) const
    {
        return k->m_adjacentNode;
    }


    NodeDescriptor getDescriptor( const NodeIterator& u) const
    {
        return u->getDescriptor();
    }

    EdgeIterator getEdgeIterator( const NodeDescriptor& uD, const NodeDescriptor& vD)
    {
        NodeIterator u = getNodeIterator( uD);
        NodeIterator v = getNodeIterator( vD);
        return getEdgeIterator( u, v);
    }

    EdgeIterator getEdgeIterator( const NodeIterator& u, const NodeIterator& v)
    {
        EdgeIterator e, end;
        NodeIterator neigh;
        for( e = beginEdges(u), end = endEdges(u); e != end; ++e)
        {
            neigh = getAdjacentNodeIterator(e);
            if( neigh == v)
            {
                break;
            }
        }
        return e;
    }

    InEdgeIterator getInEdgeIterator( const NodeDescriptor& uD, const NodeDescriptor& vD) const
    {
        NodeIterator u = getNodeIterator( uD);
        NodeIterator v = getNodeIterator( vD);
        return getInEdgeIterator(u, v);
    }

    InEdgeIterator getInEdgeIterator( const NodeIterator& u, const NodeIterator& v) const
    {
        InEdgeIterator k, end;
        NodeIterator neigh;
        for( k = beginInEdges(v), end = endInEdges(v); k != end; ++k)
        {
            neigh = getAdjacentNodeIterator(k);
            if( neigh == u)
            {
                break;
            }
        }
        return k;
    }

    EdgeIterator getEdgeIterator( const InEdgeIterator& k) const
    {
        NodeIterator u = getAdjacentNodeIterator(k);
        return beginEdges(u) + k->m_edge;
    }

    SizeType getId( const NodeIterator& u) const
    {
        return distance( m_nodes.begin(), u);
    }

    InEdgeIterator getInEdgeIterator( const EdgeIterator& e) const
    {
        NodeIterator u = getAdjacentNodeIterator(e);
        return beginInEdges(u) + e->m_InEdge;
    }

    NodeIterator getNodeIterator( const NodeDescriptor& descriptor) const
    {
        return *descriptor;
    }

    bool hasEdge( const NodeDescriptor& uD, const NodeDescriptor& vD) const
    {
        if( hasNode(uD) && hasNode(vD))
        {
            NodeIterator u = getNodeIterator( uD);
            NodeIterator v = getNodeIterator( vD);
            EdgeIterator e, end;
            NodeIterator neigh;
            for( e = beginEdges(u), end = endEdges(u); e != end; ++e)
            {
                neigh = target(e);
                if( neigh == v)
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool hasEdges( const NodeIterator& u) const
    {
        return !(*u).m_edges.empty();
    }

    bool hasInEdges( const NodeIterator& u) const
    {
        return !(*u).m_inEdges.empty();
    }

    bool hasNode( const NodeDescriptor& descriptor) const
    {
        return descriptor != 0;
    }

    SizeType indeg( const NodeIterator& u) const
    {
        return distance( beginInEdges(u), endInEdges(u));
    }

    NodeDescriptor insertNode()
    {
        //create empty node
        StaticFSNode<Vtype,Etype,InEtype> newNode;
        //create descriptor
        m_nodeIters.push_back(m_nodes.end());
        NodeDescriptor m_auxNodeDescriptor = &(m_nodeIters.back());
        newNode.setDescriptor( m_auxNodeDescriptor);

        //insert node
        m_nodes.push_back( newNode);
        NodeIterator m_auxNodeIterator = m_nodes.end();
        --m_auxNodeIterator;

        m_edgeSet.registerBucket(&(m_auxNodeIterator->m_edges));
        m_inEdgeSet.registerBucket(&(m_auxNodeIterator->m_inEdges));

        //update descriptor
        *m_auxNodeDescriptor = m_auxNodeIterator;
        ++m_numNodes;
        return m_auxNodeDescriptor;
    }

    NodeDescriptor insertNodeBefore( const NodeDescriptor& uD)
    {
        //create empty node
        StaticFSNode<Vtype,Etype,InEtype> newNode;
        //create descriptor
        NodeDescriptor m_auxNodeDescriptor = new NodeIterator();
        newNode.setDescriptor( m_auxNodeDescriptor);

        NodeIterator u = getNodeIterator(uD);
        //insert node
        NodeIterator m_auxNodeIterator = m_nodes.insert( u, newNode);

        //update descriptor
        *m_auxNodeDescriptor = m_auxNodeIterator;
        ++m_numNodes;
        return m_auxNodeDescriptor;
    }

    void insertEdge( const NodeDescriptor& uD, const NodeDescriptor& vD)
    {
        NodeIterator u = getNodeIterator( uD);
        NodeIterator v = getNodeIterator( vD);

        //create empty edge
        StaticFSEdge<Vtype, Etype, InEtype> newEdge;
        //create empty back edge
        StaticFSInEdge<Vtype, Etype, InEtype> newInEdge;

        //insert forward edge
        (*u).m_edges.push_back( newEdge);

        //insert backward edge
        (*v).m_inEdges.push_back( newInEdge);

        //get iterators on edges
        EdgeIterator e = u->m_edges.end();
        --e;
        InEdgeIterator k = v->m_inEdges.end();
        --k;

        //insert edge info
        e->m_adjacentNode = v;
        e->m_InEdge = distance( v->m_inEdges.begin(), k);

        k->m_adjacentNode = u;
        k->m_edge = distance( u->m_edges.begin(), e);

        ++m_numEdges;
    }

    void changeHead( EdgeIterator& e, NodeIterator& v)
    {
        NodeIterator& x = e->m_adjacentNode;

        if( x == v)
            return;

        InEdgeIterator k = getInEdgeIterator( e);
        NodeIterator& u = k->m_adjacentNode;

        //erase inEdge
        (*x).m_inEdges.erase( k);

        //insert inEdge
        StaticFSInEdge<Vtype, Etype, InEtype> newInEdge;
        (*v).m_inEdges.push_back( newInEdge);
        k = v->m_inEdges.end();
        --k;

        //update inEdge's tail
        k->m_adjacentNode = u;
        k->m_edge = distance( u->m_edges.begin(), e);

        //update edge's head
        e->m_adjacentNode = v;
        e->m_InEdge = distance( v->m_inEdges.begin(), k);
    }

    void swapEdges( EdgeIterator& e1, EdgeIterator& e2)
    {
        InEdgeIterator k1 = getInEdgeIterator( e1);
        InEdgeIterator k2 = getInEdgeIterator( e2);

        std::swap(e1,e2);
        std::swap(k1,k2);

        /*NodeIterator v1 = getAdjacentNodeIterator( e1);
        NodeIterator v2 = getAdjacentNodeIterator( e2);

        e1->m_adjacentNode = v2;
        e2->m_adjacentNode = v1;

        e1->m_InEdge = distance( k2, (e2->m_adjacentNode)->m_inEdges.begin());
        e2->m_InEdge = distance( k1, (e1->m_adjacentNode)->m_inEdges.begin());

        k1->m_edge = distance( e2, (k2->m_adjacentNode)->m_edges.begin());
        k2->m_edge = distance( e1, (k1->m_adjacentNode)->m_edges.begin());

        EdgeIterator etemp = e1;
        e1 = e2;
        e2 = etemp;*/
    }

    SizeType memUsage()
    {
        std::cout << "Graph mem Usage\t\tNodes\tEdges\tInEdges\n";
        std::cout << "\tNumber:\t\t" << m_numNodes << "\t" << m_numEdges << "\t" << m_numEdges << std::endl;
        std::cout << "\tSize:\t\t" << StaticFSNode<Vtype,Etype,InEtype>::memUsage() << "\t" << StaticFSEdge<Vtype,Etype,InEtype>::memUsage() << "\t" << StaticFSInEdge<Vtype,Etype,InEtype>::memUsage() << std::endl;

        /*std::cout << "\tNodes:\t\t" << m_numNodes << "\n";
        std::cout << "\tEdges:\t\t" << m_numEdges << "\n";
        std::cout << "\tInEdg/es:\t" << m_numEdges << "\n";
        std::cout << "\tNode Size:\t" << StaticFSNode<Vtype,Etype,InEtype>::memUsage() << "bytes" << "\t";
        std::cout << "\tEdge Size:\t" << StaticFSEdge<Vtype,Etype,InEtype>::memUsage() << "bytes" << "\t";
        std::cout << "\tBWEdge Size:\t" << StaticFSInEdge<Vtype,Etype,InEtype>::memUsage() << "bytes" << std::endl;*/

        return m_numNodes * StaticFSNode<Vtype,Etype,InEtype>::memUsage() + m_numEdges * (StaticFSEdge<Vtype,Etype,InEtype>::memUsage() + StaticFSInEdge<Vtype,Etype,InEtype>::memUsage());
    }


    EdgeIterator nilEdge() const
    {
        return (*m_nodes.begin()).m_edges.end();
    }

    NodeIterator nilNode() const
    {
        return m_nodes.end();
    }

    SizeType outdeg( const NodeIterator& u) const
    {
        return distance( beginEdges(u), endEdges(u));
    }

    void pushEdge( const NodeDescriptor& uD, const NodeDescriptor& vD)
    {
        insertEdge(uD,vD);
    }

    void reserve( const SizeType& numNodes, const SizeType& numEdges)
    {
        m_nodes.reserve( numNodes);
        m_nodeIters.reserve(numNodes);
        return;
    }

    void sanitizeDescriptor( const NodeIterator& u) const
    {
        NodeDescriptor descriptor = getDescriptor(u);
        *descriptor = u;
    }


    void setDescriptor( NodeIterator& u, NodeDescriptor& uD) const
    {
        u->setDescriptor( uD);
        *uD = u;
    }

private:
    std::vector< StaticFSNode<Vtype,Etype,InEtype> >    m_nodes;
    std::vector<NodeIterator>    m_nodeIters;
    MersenneTwister                               m_random;
    SizeType                                      m_numNodes;
    SizeType                                      m_numEdges;
    BucketSet< StaticFSEdge<Vtype,Etype,InEtype> >      m_edgeSet;
    BucketSet< StaticFSInEdge<Vtype,Etype,InEtype> >    m_inEdgeSet;
};


template<typename Vtype, typename Etype, typename InEtype>
class StaticFSEdge : public Etype
{

public:
    typedef typename StaticForwardStarImpl<Vtype,Etype,InEtype>::InEdgeIterator InEdgeIterator;
    typedef typename StaticForwardStarImpl<Vtype,Etype,InEtype>::NodeIterator NodeIterator;
    typedef typename StaticForwardStarImpl<Vtype,Etype,InEtype>::NodeDescriptor NodeDescriptor;
    typedef typename StaticForwardStarImpl<Vtype,Etype,InEtype>::SizeType SizeType;

    StaticFSEdge( unsigned int init = 0): Etype()
    {
    }

    StaticFSEdge( NodeIterator adjacentNode , SizeType oppositeEdge , Etype data = Etype()):
            Etype(),
		    m_adjacentNode(adjacentNode),
			m_InEdge(oppositeEdge)
    {
    }

    static unsigned int memUsage()
    {
        //std::cout << GraphElement< Etype, EdgeDescriptor>::memUsage() + sizeof(NodeIterator) + sizeof(EdgeIterator) + sizeof(bool) << std::endl;
        return sizeof(StaticFSEdge);
    }

    NodeIterator    m_adjacentNode;
    SizeType        m_InEdge;
};


template<typename Vtype, typename Etype, typename InEtype>
class StaticFSInEdge : public InEtype
{

public:
    typedef typename StaticForwardStarImpl<Vtype,Etype,InEtype>::InEdgeIterator InEdgeIterator;
    typedef typename StaticForwardStarImpl<Vtype,Etype,InEtype>::NodeIterator NodeIterator;
    typedef typename StaticForwardStarImpl<Vtype,Etype,InEtype>::NodeDescriptor NodeDescriptor;
    typedef typename StaticForwardStarImpl<Vtype,Etype,InEtype>::SizeType SizeType;

    StaticFSInEdge( unsigned int init = 0): InEtype()
    {
    }

    StaticFSInEdge( NodeIterator adjacentNode , SizeType oppositeEdge ,  InEtype data = InEtype()):
            InEtype(),
		    m_adjacentNode(adjacentNode),
			m_edge(oppositeEdge)
    {
    }

    //StaticFSInEdge( const StaticFSEdge<Vtype,Etype,InEtype>& edge):InEtype(edge)
    //{
    //}

    static unsigned int memUsage()
    {
        //std::cout << GraphElement< InEtype, EdgeDescriptor>::memUsage() + sizeof(NodeIterator) + sizeof(EdgeIterator) + sizeof(bool) << std::endl;
        return sizeof(StaticFSInEdge);
    }

    NodeIterator    m_adjacentNode;
    SizeType        m_edge;
};

template<typename Vtype, typename Etype, typename InEtype>
class StaticFSNode : public GraphElement< Vtype, typename StaticForwardStarImpl<Vtype,Etype,InEtype>::NodeDescriptor>
{
public:
    typedef typename StaticForwardStarImpl<Vtype,Etype,InEtype>::NodeDescriptor      NodeDescriptor;
    typedef typename std::vector<StaticFSEdge<Vtype,Etype,InEtype> >::const_iterator   iterator;
    typedef typename std::vector<StaticFSInEdge<Vtype,Etype,InEtype> >::const_iterator backIterator;

    StaticFSNode():GraphElement< Vtype, NodeDescriptor>()
    {
    }

    StaticFSNode( NodeDescriptor descriptor):GraphElement< Vtype, NodeDescriptor>( descriptor)
    {
    }

    /*StaticFSNode( const StaticFSNode& other):GraphElement< Vtype, NodeDescriptor>(other)
    {
        for( iterator it = other.m_edges.begin(); it != m_edges.end(); ++it)
        {
            m_edges.push_back( *it);
        }

        for( backIterator it = other.m_inEdges.begin(); it != m_inEdges.end(); ++it)
        {
            m_inEdges.push_back( *it);
        }
    }*/

    static unsigned int memUsage()
    {
        return sizeof(StaticFSNode);
    }

    //std::vector<StaticFSNode<Vtype,Etype,InEtype> >::iterator node;
    typename BucketSet<StaticFSEdge<Vtype,Etype,InEtype> >::Bucket         m_edges;
    typename BucketSet<StaticFSInEdge<Vtype,Etype,InEtype> >::Bucket       m_inEdges;

};



#endif //FORWARDSTARIMPL_H
