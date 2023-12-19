#ifndef ADJACENCYLISTIMPL_H
#define ADJACENCYLISTIMPL_H

#include <Structs/Graphs/dynamicGraph.h>
#include <Utilities/mersenneTwister.h>
#include <list>

template<typename Vtype, typename Etype, typename InEtype>
class ALNode;
template<typename Vtype, typename Etype, typename InEtype>
class ALEdge;
template<typename Vtype, typename Etype, typename InEtype>
class ALInEdge;


template<typename Vtype, typename Etype, typename InEtype>
class AdjacencyListImpl
{
public:

    typedef unsigned int                                                         SizeType;
    typedef typename std::list< ALNode< Vtype, Etype, InEtype> >::iterator       NodeIterator;
    typedef typename std::list< ALEdge< Vtype, Etype, InEtype> >::iterator       EdgeIterator;
    typedef typename std::list< ALInEdge< Vtype, Etype, InEtype> >::iterator     InEdgeIterator;
    typedef NodeIterator*                                                        NodeDescriptor;

    AdjacencyListImpl(): m_numNodes(0), m_numEdges(0)
    {
    }

    ~AdjacencyListImpl()
    {
        for( NodeIterator u=m_nodes.begin(), endNode=m_nodes.end(); u!=endNode; ++u)
            delete getDescriptor(u);
    }

    EdgeIterator beginEdges( const NodeIterator& u)
    {
        return u->m_edges.begin();
    }

    InEdgeIterator beginInEdges( const NodeIterator& u)
    {
        return u->m_InEdges.begin();
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
    }

    void compress()
    {
    }

    EdgeIterator endEdges( const NodeIterator& u)
    {
        return u->m_edges.end();
    }

    InEdgeIterator endInEdges( const NodeIterator& u)
    {
        return u->m_InEdges.end();
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

        (*v).m_InEdges.erase(k);
        (*u).m_edges.erase(e);

        --m_numEdges;
    }

    /*void eraseEdge( const InEdgeIterator& k)
    {
        eraseEdge( getEdgeIterator(k));
    }*/

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

    InEdgeIterator getInEdgeIterator( const NodeIterator& u, const NodeIterator& v)
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
        return k->m_edge;
    }

    SizeType getId( const NodeIterator& u)
    {
        return distance( m_nodes.begin(), u);
    }

    InEdgeIterator getInEdgeIterator( const EdgeIterator& e)
    {
        return e->m_InEdge;
    }

    NodeIterator getNodeIterator( const NodeDescriptor& descriptor) const
    {
        return *descriptor;
    }

    bool hasEdge( const NodeDescriptor& uD, const NodeDescriptor& vD)
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

    bool hasEdges( const NodeIterator& u)
    {
        return !(*u).m_edges.empty();
    }

    bool hasInEdges( const NodeIterator& u)
    {
        return !(*u).m_InEdges.empty();
    }

    bool hasNode( const NodeDescriptor& descriptor)
    {
        return descriptor != 0;
    }

    SizeType indeg( const NodeIterator& u)
    {
        return distance( beginInEdges(u), endInEdges(u));
    }

    NodeDescriptor insertNode()
    {
        //create empty node
        ALNode<Vtype,Etype,InEtype> newNode;
        //create descriptor
        NodeDescriptor m_auxNodeDescriptor = new NodeIterator();
        newNode.setDescriptor( m_auxNodeDescriptor);

        //insert node
        m_nodes.push_back( newNode);
        NodeIterator m_auxNodeIterator = m_nodes.end();
        --m_auxNodeIterator;

        //update descriptor
        *m_auxNodeDescriptor = m_auxNodeIterator;
        ++m_numNodes;
        return m_auxNodeDescriptor;
    }

    NodeDescriptor insertNodeBefore( const NodeDescriptor& uD)
    {
        //create empty node
        ALNode<Vtype,Etype,InEtype> newNode;
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
        ALEdge<Vtype,Etype,InEtype> newEdge;
        //create empty back edge
        ALInEdge<Vtype,Etype,InEtype> newInEdge;

        //insert forward edge
        (*u).m_edges.push_back( newEdge);

        //insert backward edge
        (*v).m_InEdges.push_back( newInEdge);

        //get iterators on edges
        EdgeIterator e = u->m_edges.end();
        --e;
        InEdgeIterator k = v->m_InEdges.end();
        --k;

        //insert edge info
        e->m_adjacentNode = v;
        e->m_InEdge = k;

        k->m_adjacentNode = u;
        k->m_edge = e;

        ++m_numEdges;
    }

    void changeHead( EdgeIterator& e, NodeIterator& v)
    {
        NodeIterator x = getAdjacentNodeIterator(e);

        if( x == v)
            return;

        InEdgeIterator k = getInEdgeIterator( e);
        NodeIterator u = getAdjacentNodeIterator(k);

        if( v == u) { std::cout << "\nself loop\n"; exit(0);}

        //erase inEdge
        (*x).m_InEdges.erase( k);

        //insert inEdge
        ALInEdge<Vtype,Etype,InEtype> newInEdge;
        (*v).m_InEdges.push_back( newInEdge);
        k = v->m_InEdges.end();
        --k;

        //update inEdge's tail
        k->m_adjacentNode = u;
        k->m_edge = e;

        //update edge's head
        e->m_adjacentNode = v;
        e->m_InEdge = k;
    }

    /*void swapEdges( EdgeIterator& e1, EdgeIterator& e2)
    {
        InEdgeIterator k1 = getInEdgeIterator( e1);
        InEdgeIterator k2 = getInEdgeIterator( e2);

        NodeIterator v1 = getAdjacentNodeIterator( e1);
        NodeIterator v2 = getAdjacentNodeIterator( e2);

        e1->m_adjacentNode = v2;
        e2->m_adjacentNode = v1;

        e1->m_InEdge = k2;
        e2->m_InEdge = k1;

        k1->m_edge = e2;
        k2->m_edge = e1;

        EdgeIterator etemp = e1;
        e1 = e2;
        e2 = etemp;
    }

    void swapEdges( InEdgeIterator& k1, InEdgeIterator& k2)
    {
        EdgeIterator e1 = getEdgeIterator( k1);
        EdgeIterator e2 = getEdgeIterator( k2);

        NodeIterator v1 = getAdjacentNodeIterator( k1);
        NodeIterator v2 = getAdjacentNodeIterator( k2);

        k1->m_adjacentNode = v2;
        k2->m_adjacentNode = v1;

        k1->m_edge = e2;
        k2->m_edge = e1;

        e1->m_InEdge = k2;
        e2->m_InEdge = k1;

        InEdgeIterator ktemp = k1;
        k1 = k2;
        k2 = ktemp;
    }*/

    SizeType memUsage()
    {
        std::cout << "Graph mem Usage\t\tNodes\tEdges\tInEdges\n";
        std::cout << "\tNumber:\t\t" << m_numNodes << "\t" << m_numEdges << "\t" << m_numEdges << std::endl;
        std::cout << "\tSize:\t\t" << ALNode<Vtype,Etype,InEtype>::memUsage() << "\t" << ALEdge<Vtype,Etype,InEtype>::memUsage() << "\t" << ALInEdge<Vtype,Etype,InEtype>::memUsage() << std::endl;

        /*std::cout << "\tNodes:\t\t" << m_numNodes << "\n";
        std::cout << "\tEdges:\t\t" << m_numEdges << "\n";
        std::cout << "\tInEdges:\t" << m_numEdges << "\n";
        std::cout << "\tNode Size:\t" << ALNode<Vtype,Etype,InEtype>::memUsage() << "bytes" << "\t";
        std::cout << "\tEdge Size:\t" << ALEdge<Vtype,Etype,InEtype>::memUsage() << "bytes" << "\t";
        std::cout << "\tBWEdge Size:\t" << ALInEdge<Vtype,Etype,InEtype>::memUsage() << "bytes" << std::endl;*/

        return m_numNodes * ALNode<Vtype,Etype,InEtype>::memUsage() + m_numEdges * (ALEdge<Vtype,Etype,InEtype>::memUsage() + ALInEdge<Vtype,Etype,InEtype>::memUsage());
    }


    EdgeIterator nilEdge() const
    {
        return (*m_nodes.begin()).m_edges.end();
    }

    NodeIterator nilNode() const
    {
        return m_nodes.end();
    }

    SizeType outdeg( const NodeIterator& u)
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

    void sanitizeDescriptor( const NodeIterator& u)
    {
        NodeDescriptor descriptor = getDescriptor(u);
        *descriptor = u;
    }


    void setDescriptor( NodeIterator& u, NodeDescriptor& uD)
    {
        u->setDescriptor( uD);
        *uD = u;
    }


private:
    std::list< ALNode<Vtype,Etype,InEtype> >  m_nodes;
    MersenneTwister                     m_random;
    SizeType                            m_numNodes;
    SizeType                            m_numEdges;
};


template<typename Vtype, typename Etype, typename InEtype>
class ALEdge : public Etype
{

public:
    typedef typename AdjacencyListImpl<Vtype,Etype,InEtype>::InEdgeIterator InEdgeIterator;
    typedef typename AdjacencyListImpl<Vtype,Etype,InEtype>::NodeIterator   NodeIterator;
    typedef typename AdjacencyListImpl<Vtype,Etype,InEtype>::NodeDescriptor NodeDescriptor;

    ALEdge( unsigned int init = 0): Etype()
    {
    }

    ALEdge( NodeIterator adjacentNode , InEdgeIterator oppositeEdge , Etype data = Etype()):
            Etype(),
		    m_adjacentNode(adjacentNode),
			m_InEdge(oppositeEdge)
    {
    }

    static unsigned int memUsage()
    {
        //std::cout << GraphElement< Etype, EdgeDescriptor>::memUsage() + sizeof(NodeIterator) + sizeof(EdgeIterator) + sizeof(bool) << std::endl;
        return sizeof(ALEdge);
    }

    NodeIterator                m_adjacentNode;
    InEdgeIterator              m_InEdge;
};


template<typename Vtype, typename Etype, typename InEtype>
class ALInEdge : public InEtype
{

public:
    typedef typename AdjacencyListImpl<Vtype,Etype,InEtype>::EdgeIterator EdgeIterator;
    typedef typename AdjacencyListImpl<Vtype,Etype,InEtype>::NodeIterator NodeIterator;
    typedef typename AdjacencyListImpl<Vtype,Etype,InEtype>::NodeDescriptor NodeDescriptor;

    ALInEdge( unsigned int init = 0): InEtype()
    {
    }

    ALInEdge( NodeIterator adjacentNode , EdgeIterator oppositeEdge ,  InEtype data = InEtype()):
            InEtype(),
		    m_adjacentNode(adjacentNode),
			m_edge(oppositeEdge)
    {
    }

    ALInEdge( const ALEdge<Vtype,Etype,InEtype>& edge):InEtype(edge)
    {
    }

    static unsigned int memUsage()
    {
        //std::cout << GraphElement< InEtype, EdgeDescriptor>::memUsage() + sizeof(NodeIterator) + sizeof(EdgeIterator) + sizeof(bool) << std::endl;
        return sizeof(ALInEdge);
    }

    NodeIterator              m_adjacentNode;
    EdgeIterator              m_edge;
};

template<typename Vtype, typename Etype, typename InEtype>
class ALNode : public GraphElement< Vtype, typename AdjacencyListImpl<Vtype,Etype,InEtype>::NodeDescriptor>
{
public:
    typedef typename AdjacencyListImpl<Vtype,Etype,InEtype>::NodeDescriptor NodeDescriptor;
    typedef typename std::list<ALEdge<Vtype,Etype,InEtype> >::const_iterator iterator;
    typedef typename std::list<ALInEdge<Vtype,Etype,InEtype> >::const_iterator backIterator;

    ALNode():GraphElement< Vtype, NodeDescriptor>()
    {
    }

    ALNode( NodeDescriptor descriptor):GraphElement< Vtype, NodeDescriptor>( descriptor)
    {
    }

    /*ALNode( const ALNode& other):GraphElement< Vtype, NodeDescriptor>(other)
    {
        for( iterator it = other.m_edges.begin(); it != m_edges.end(); ++it)
        {
            m_edges.push_back( *it);
        }

        for( backIterator it = other.m_InEdges.begin(); it != m_InEdges.end(); ++it)
        {
            m_InEdges.push_back( *it);
        }
    }*/

    static unsigned int memUsage()
    {
        return sizeof(ALNode);
    }

    std::list< ALEdge<Vtype,Etype,InEtype> >          m_edges;
    std::list< ALInEdge<Vtype,Etype,InEtype> >        m_InEdges;

};



#endif //ADJACENCYLISTIMPL_H
