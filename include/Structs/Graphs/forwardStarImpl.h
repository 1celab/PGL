#ifndef FORWARDSTARIMPL_H
#define FORWARDSTARIMPL_H

#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Sets/bucketSet.h>
#include <Utilities/mersenneTwister.h>
#include <list>

template<typename Vtype, typename Etype, typename InEtype>
class FSNode;
template<typename Vtype, typename Etype, typename InEtype>
class FSEdge;
template<typename Vtype, typename Etype, typename InEtype>
class FSInEdge;

template<typename Vtype, typename Etype, typename InEtype>
class ForwardStarImpl
{
public:

    typedef unsigned int                                                         SizeType;
    typedef typename std::list< FSNode< Vtype, Etype, InEtype> >::iterator       NodeIterator;
    typedef typename std::vector< FSEdge< Vtype, Etype, InEtype> >::iterator     EdgeIterator;
    typedef typename std::vector< FSInEdge< Vtype, Etype, InEtype> >::iterator   InEdgeIterator;
    typedef NodeIterator*                                                        NodeDescriptor;

    ForwardStarImpl():m_numNodes(0),m_numEdges(0)
    {
    }

    ~ForwardStarImpl()
    {
        NodeIterator end = m_nodes.end();
        EdgeIterator e, end_edges;

        for( NodeIterator u=m_nodes.begin(), endNode=m_nodes.end(); u!=endNode; ++u)
            delete getDescriptor(u);
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

        for( NodeIterator u=m_nodes.begin(), endNode=m_nodes.end(); u!=endNode; ++u)
            delete getDescriptor(u);

        m_nodes.clear();
        m_edgeSet.clear();
        m_inEdgeSet.clear();
    }

    void compress()
    {

        /*
    NodeDescriptor insertNode()
    {
        //create empty node
        FSNode<Vtype,Etype,InEtype> newNode;
        //create descriptor

        NodeDescriptor m_auxNodeDescriptor = new NodeIterator();
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

        struct NodeObject
        {
            FSNode<Vtype,Etype,InEtype> v;
            NodeIterator iter;
        };

        NodeObject* vec_nodes =  new NodeObject( G.getNumNodes());


        NodeIterator v = m_nodes.begin();
        for( std::size_t i=0; i<)
        {
            vec_nodes[i].v.setDescriptor( vec_nodes[i].iter);
            m_nodes
        }*/



        BucketSet< FSEdge<Vtype,Etype,InEtype> > outEdgeSet( m_numEdges);
        BucketSet< FSInEdge<Vtype,Etype,InEtype> >  inEdgeSet( m_numEdges);

        for( NodeIterator v=m_nodes.begin(), endNode=m_nodes.end(); v!=endNode; ++v)
        {
            std::size_t numOutEdges = outdeg(v);
            typename BucketSet< FSEdge<Vtype,Etype,InEtype> >::Bucket outEdgesBucket = v->m_edges;
            outEdgeSet.registerBucket(&(v->m_edges), numOutEdges);
            outEdgesBucket.copyTo(v->m_edges);
            (v->m_edges).m_bucketSet = &m_edgeSet;

            std::size_t numInEdges = indeg(v);
            typename BucketSet< FSInEdge<Vtype,Etype,InEtype> >::Bucket inEdgesBucket = v->m_inEdges;
            inEdgeSet.registerBucket(&(v->m_inEdges), numInEdges);
            inEdgesBucket.copyTo(v->m_inEdges);
            (v->m_inEdges).m_bucketSet = &m_inEdgeSet;
        }

        m_edgeSet.swap( outEdgeSet);
        m_inEdgeSet.swap( inEdgeSet);
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

    void eraseNode( NodeDescriptor& descriptor)
    {
        NodeIterator u = getNodeIterator(descriptor);
        delete descriptor;
        descriptor = 0;
        m_nodes.erase(u);
        --m_numNodes;
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
        FSNode<Vtype,Etype,InEtype> newNode;
        //create descriptor
        NodeDescriptor m_auxNodeDescriptor = new NodeIterator();
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
        FSNode<Vtype,Etype,InEtype> newNode;
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
        FSEdge<Vtype, Etype, InEtype> newEdge;
        //create empty back edge
        FSInEdge<Vtype, Etype, InEtype> newInEdge;

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
        FSInEdge<Vtype, Etype, InEtype> newInEdge;
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
        std::cout << "\tSize:\t\t" << FSNode<Vtype,Etype,InEtype>::memUsage() << "\t" << FSEdge<Vtype,Etype,InEtype>::memUsage() << "\t" << FSInEdge<Vtype,Etype,InEtype>::memUsage() << std::endl;

        /*std::cout << "\tNodes:\t\t" << m_numNodes << "\n";
        std::cout << "\tEdges:\t\t" << m_numEdges << "\n";
        std::cout << "\tInEdges:\t" << m_numEdges << "\n";
        std::cout << "\tNode Size:\t" << FSNode<Vtype,Etype,InEtype>::memUsage() << "bytes" << "\t";
        std::cout << "\tEdge Size:\t" << FSEdge<Vtype,Etype,InEtype>::memUsage() << "bytes" << "\t";
        std::cout << "\tBWEdge Size:\t" << FSInEdge<Vtype,Etype,InEtype>::memUsage() << "bytes" << std::endl;*/

        return m_numNodes * FSNode<Vtype,Etype,InEtype>::memUsage() + m_numEdges * (FSEdge<Vtype,Etype,InEtype>::memUsage() + FSInEdge<Vtype,Etype,InEtype>::memUsage());
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
    std::list< FSNode<Vtype,Etype,InEtype> >    m_nodes;
    MersenneTwister                             m_random;
    SizeType                                    m_numNodes;
    SizeType                                    m_numEdges;
    BucketSet< FSEdge<Vtype,Etype,InEtype> >    m_edgeSet;
    BucketSet< FSInEdge<Vtype,Etype,InEtype> >  m_inEdgeSet;
};


template<typename Vtype, typename Etype, typename InEtype>
class FSEdge : public Etype
{

public:
    typedef typename ForwardStarImpl<Vtype,Etype,InEtype>::InEdgeIterator InEdgeIterator;
    typedef typename ForwardStarImpl<Vtype,Etype,InEtype>::NodeIterator NodeIterator;
    typedef typename ForwardStarImpl<Vtype,Etype,InEtype>::NodeDescriptor NodeDescriptor;
    typedef typename ForwardStarImpl<Vtype,Etype,InEtype>::SizeType SizeType;

    FSEdge( unsigned int init = 0): Etype()
    {
    }

    FSEdge( NodeIterator adjacentNode , SizeType oppositeEdge , Etype data = Etype()):
            Etype(),
		    m_adjacentNode(adjacentNode),
			m_InEdge(oppositeEdge)
    {
    }

    static unsigned int memUsage()
    {
        //std::cout << GraphElement< Etype, EdgeDescriptor>::memUsage() + sizeof(NodeIterator) + sizeof(EdgeIterator) + sizeof(bool) << std::endl;
        return sizeof(FSEdge);
    }

    NodeIterator    m_adjacentNode;
    SizeType        m_InEdge;
};


template<typename Vtype, typename Etype, typename InEtype>
class FSInEdge : public InEtype
{

public:
    typedef typename ForwardStarImpl<Vtype,Etype,InEtype>::InEdgeIterator InEdgeIterator;
    typedef typename ForwardStarImpl<Vtype,Etype,InEtype>::NodeIterator NodeIterator;
    typedef typename ForwardStarImpl<Vtype,Etype,InEtype>::NodeDescriptor NodeDescriptor;
    typedef typename ForwardStarImpl<Vtype,Etype,InEtype>::SizeType SizeType;

    FSInEdge( unsigned int init = 0): InEtype()
    {
    }

    FSInEdge( NodeIterator adjacentNode , SizeType oppositeEdge ,  InEtype data = InEtype()):
            InEtype(),
		    m_adjacentNode(adjacentNode),
			m_edge(oppositeEdge)
    {
    }

    //FSInEdge( const FSEdge<Vtype,Etype,InEtype>& edge):InEtype(edge)
    //{
    //}

    static unsigned int memUsage()
    {
        //std::cout << GraphElement< InEtype, EdgeDescriptor>::memUsage() + sizeof(NodeIterator) + sizeof(EdgeIterator) + sizeof(bool) << std::endl;
        return sizeof(FSInEdge);
    }

    NodeIterator    m_adjacentNode;
    SizeType        m_edge;
};

template<typename Vtype, typename Etype, typename InEtype>
class FSNode : public GraphElement< Vtype, typename ForwardStarImpl<Vtype,Etype,InEtype>::NodeDescriptor>
{
public:
    typedef typename ForwardStarImpl<Vtype,Etype,InEtype>::NodeDescriptor      NodeDescriptor;
    typedef typename std::list<FSEdge<Vtype,Etype,InEtype> >::const_iterator   iterator;
    typedef typename std::list<FSInEdge<Vtype,Etype,InEtype> >::const_iterator backIterator;

    FSNode():GraphElement< Vtype, NodeDescriptor>()
    {
    }

    FSNode( NodeDescriptor descriptor):GraphElement< Vtype, NodeDescriptor>( descriptor)
    {
    }

    /*FSNode( const FSNode& other):GraphElement< Vtype, NodeDescriptor>(other)
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
        return sizeof(FSNode);
    }

    typename BucketSet<FSEdge<Vtype,Etype,InEtype> >::Bucket         m_edges;
    typename BucketSet<FSInEdge<Vtype,Etype,InEtype> >::Bucket       m_inEdges;

};



#endif //FORWARDSTARIMPL_H
