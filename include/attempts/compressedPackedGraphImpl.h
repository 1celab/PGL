#ifndef CompressedPackedGraphImpl_H
#define CompressedPackedGraphImpl_H

#include <Utilities/mersenneTwister.h>
#include <vector>

template<typename Vtype, typename Etype>
class CPGNode;
template<typename Vtype, typename Etype>
class CPGEdge;

template<typename Vtype, typename Etype>
class CPGNodeObserver;
template<typename Vtype, typename Etype>
class CPGEdgeObserver;
template<typename Vtype, typename Etype>
class SingleEdgeObserver;


template<typename Vtype, typename Etype>
class CompressedPackedGraphImpl
{

    friend class CPGNodeObserver< Vtype, Etype>;
    friend class CPGEdgeObserver< Vtype, Etype>;
    friend class SingleEdgeObserver< Vtype, Etype>;

public:
    
    typedef unsigned int                                                        SizeType;
    typedef CPGNode<Vtype,Etype>**                                              NodeDescriptor;
    typedef typename PackedMemoryArray< CPGNode< Vtype, Etype> >::Iterator      NodeIterator;
    typedef typename PackedMemoryArray< CPGEdge< Vtype, Etype> >::Iterator      EdgeIterator;
    typedef typename PackedMemoryArray< CPGEdge< Vtype, Etype> >::Iterator      InEdgeIterator;
    

    CompressedPackedGraphImpl()
    {
        m_nodeObserver = new CPGNodeObserver< Vtype, Etype>(this);
        m_nodes.registerObserver( m_nodeObserver);
        
        m_edgeObserver = new CPGEdgeObserver< Vtype, Etype>(this);
        m_edges.registerObserver( m_edgeObserver);   

        m_singleEdgeObserver = new SingleEdgeObserver< Vtype, Etype>(this);
        m_edges.registerObserver( m_singleEdgeObserver);    
    }

    ~CompressedPackedGraphImpl() 
    {
        delete m_nodeObserver;
        delete m_edgeObserver;

        NodeIterator end = m_nodes.end();
        EdgeIterator end_edges = m_edges.end();

        for( NodeIterator u = m_nodes.begin(); u != end; ++u)
        {
            delete u->getDescriptor();
        }
    }
    
    EdgeIterator beginEdges()
    {
        return m_edges.begin();
    }
    
    EdgeIterator beginEdges( const NodeIterator& u)  
    { 
        return getEdgeIteratorAtAddress(u->m_firstEdge);
    }
    
    InEdgeIterator beginInEdges( const NodeIterator& u)  
    { 
        return getInEdgeIteratorAtAddress( u->m_firstEdge);
    }
    
    NodeIterator beginNodes()
    { 
        return  m_nodes.begin();
    }
    
    NodeIterator chooseNode()  
	{ 
	    double random = m_random.getRandomNormalizedDouble();
        SizeType pos = m_nodes.size() * random;
        
        NodeIterator m_auxNodeIterator = m_nodes.begin() + pos;
        return m_auxNodeIterator;
	}
	
	void clear()
    {
        m_nodes.clear();
        m_edges.clear();
    }
    
    void compress()
    {
        std::cout << "Compressing graph...\n";
        m_nodes.compress();
        m_edges.compress();
    }
    
    EdgeIterator endEdges()
    {
        return m_edges.end();
    }
    
    EdgeIterator endEdges( const NodeIterator& u)  
    { 
        return getEdgeIteratorAtAddress( u->m_lastEdge);
    }
       
    InEdgeIterator endInEdges( const NodeIterator& u)  
    { 
        return getInEdgeIteratorAtAddress( u->m_lastEdge);
    }
    
    NodeIterator endNodes()
    { 
        return m_nodes.end();
    }
    
    void eraseEdge( NodeDescriptor uD, NodeDescriptor vD) 
    { 
        EdgeIterator e = getEdgeIterator( uD, vD);
        InEdgeIterator k = getInEdgeIterator( e);
        NodeIterator u = getNodeIterator(uD);
        NodeIterator v = getNodeIterator(vD);
        
        //std::cout << "Deleting\t" << e->getDescriptor() << std::endl;
        //delete (e->getDescriptor());
        //descriptor = 0;
        
        // if we erase a node's first edge
        if( u->m_firstEdge == e.getAddress())
        {
            NodeIterator w,z;
            EdgeIterator f = e;
            ++f;
            
            // if it has no more edges
            if( f.getAddress() == u->m_lastEdge)
            {
                setFirstEdge(u,0);
            }
            else
            {
                setFirstEdge(u, f.getAddress());
            }
        }

        if( v->m_firstEdge == k.getAddress() )
		{
		    NodeIterator w,z;
			InEdgeIterator f = k;
		    ++f;
			if( f.getAddress() == v->m_lastEdge)
            {
                setFirstEdge( v, 0);
            }
            else
            {
                setFirstEdge( v, f.getAddress());
            }
		}

        e->m_oppositeEdge = 0;
        m_edges.erase( e);
        k->m_oppositeEdge = 0;
        m_edges.erase( k);   
    }

    void eraseNode( NodeDescriptor uD) 
    { 
        NodeIterator u = getNodeIterator( uD);
        m_nodes.erase(u);
        delete uD;
    }
    
    void expand()
    {
        std::cout << "Expanding graph...\n";
        m_nodes.expand();
        m_edges.expand();
    }
    
    NodeIterator findNextNodeWithEdges( NodeIterator u)
    {
        ++u;
        NodeIterator end = m_nodes.end();
        for( ; u != end; ++u)
        {
            if( u->hasEdges())
            {
                //std::cout << "Found next edge at distance: " << m_auxNodeIterator - u << "***" << std::endl;
                return u;
            }
        }
        return end;
    }

    NodeIterator findPreviousNodeWithEdges( const NodeIterator& u)
    {
        NodeIterator begin = m_nodes.begin();
        NodeIterator m_auxNodeIterator = u;
        if( u == begin) return m_nodes.end();
        --m_auxNodeIterator;
        while( m_auxNodeIterator != begin)
        {
            if( m_auxNodeIterator->hasEdges())
            {
                return m_auxNodeIterator;
            }   
            --m_auxNodeIterator;
        }
        return m_auxNodeIterator->hasEdges()? m_auxNodeIterator:m_nodes.end();
    }
    

    bool hasEdges( const NodeIterator& u)
    {        
        return u->hasEdges();
    }
  
    bool hasNode( const NodeDescriptor& descriptor)
    {
        return descriptor != 0;
    }
    
    NodeIterator getAdjacentNodeIterator( const EdgeIterator& e)
    { 
        return m_nodes.atAddress( e->m_adjacentNode);
    }
    
    
    NodeDescriptor getDescriptor( const NodeIterator& u) const
    {
        return u->getDescriptor();
    }

    
    EdgeIterator getEdgeIterator( const NodeDescriptor& uD, const NodeDescriptor& vD)
    {
        NodeIterator u = getNodeIterator( uD);
        NodeIterator v = getNodeIterator( vD);
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
    

    EdgeIterator getEdgeIteratorAtAddress( CPGEdge<Vtype,Etype>* addr)
    {
        return m_edges.atAddress(addr);
    } 

    EdgeIterator getInEdgeIteratorAtAddress( CPGEdge<Vtype,Etype>* addr)
    {
        return m_edges.atAddress(addr);
    } 
    
    EdgeIterator getEdgeIteratorAtIndex( const SizeType& position)
    {
        return m_edges.atIndex(position);
    } 
    
    SizeType getId( const NodeIterator& u)
    {
        return m_nodes.getElementIndexOf(u);
    }
    
    InEdgeIterator getInEdgeIterator( const EdgeIterator& e)
    {
        return getOppositeEdgeIterator(e);
    }

    InEdgeIterator getOppositeEdgeIterator( const EdgeIterator& e)
    {
        return m_edges.atAddress( e->m_oppositeEdge);
    }

    NodeIterator getNodeIterator( const NodeDescriptor& descriptor) 
    { 
        return m_nodes.atAddress(*descriptor);
    }

    NodeIterator getNodeIterator( const SizeType& position) 
    { 
        return m_nodes.atIndex(position);
    }

    NodeIterator getNodeIteratorAtAddress( CPGNode<Vtype,Etype>* addr) 
    { 
        return m_nodes.atAddress(addr);
    }


    void insertEdge( NodeDescriptor uD, NodeDescriptor vD) 
    { 
        bool monitoringFirstEdge = false;
        NodeIterator u = getNodeIterator( uD);
        NodeIterator v = getNodeIterator( vD);
        
        assert( u != v);
        assert( (!u->hasEdges()) || (u->m_firstEdge != u->m_lastEdge));
        assert( (!v->hasEdges()) || (v->m_firstEdge != v->m_lastEdge));
        
        NodeIterator w;
        EdgeIterator e;
        InEdgeIterator k;
        
        w = findNextNodeWithEdges(u);
        if( w != m_nodes.end()) e = getEdgeIteratorAtAddress(w->m_firstEdge);
        else                    e = m_edges.end();

        CPGEdge<Vtype, Etype> newEdge( v.getAddress());
        e = m_edges.insert( e, newEdge);

        if( !u->hasEdges())
        {
            setFirstEdge( u, e.getAddress());
            monitoringFirstEdge = true;
            w = findNextNodeWithEdges(u);
            if( w != m_nodes.end())
            {
                u->m_lastEdge = w->m_firstEdge;
            }
        } 

        m_singleEdgeObserver->observe( e.getAddress());


        w = findNextNodeWithEdges(v);
        if( w != m_nodes.end()) k = getEdgeIteratorAtAddress(w->m_firstEdge);
        else                    k = m_edges.end();
            
        CPGEdge<Vtype, Etype> newInEdge( u.getAddress());
        k = m_edges.insert( k, newInEdge);        
        e = getEdgeIteratorAtAddress(m_singleEdgeObserver->getCurrentAddress());

        if( monitoringFirstEdge) u->m_firstEdge = m_singleEdgeObserver->getCurrentAddress();

        if( !v->hasEdges())
        {
            setFirstEdge( v, k.getAddress());
            w = findNextNodeWithEdges(v);
            if( w != m_nodes.end())
            {
                v->m_lastEdge = w->m_firstEdge;
            }
        } 

        
        e->m_oppositeEdge = k.getAddress();
        k->m_oppositeEdge = e.getAddress();  

        assert( k->m_adjacentNode != 0);

        assert( (u->m_lastEdge > u->m_firstEdge) || (!(u->m_lastEdge)));
        std::cout << v->m_lastEdge << " " << v->m_firstEdge << "\n";
        assert( (v->m_lastEdge > v->m_firstEdge) || (!(v->m_lastEdge)));
        assert( u->hasEdges() && v->hasEdges());

    }


    void setFirstEdge( NodeIterator u, CPGEdge<Vtype,Etype>* address)
    {
        u->m_firstEdge = address;
        if( address)
        {
            u = findPreviousNodeWithEdges(u);
            if( u != m_nodes.end())
            {
                u->m_lastEdge = address;
            }
        }
        else
        {
            u->m_lastEdge = 0;
            u = findPreviousNodeWithEdges(u);
            if( u != m_nodes.end())
            {
                NodeIterator v = findNextNodeWithEdges(u);
                if( v != m_nodes.end())
                {
                    u->m_lastEdge = v->m_firstEdge;
                }
                else
                {
                    u->m_lastEdge = 0;
                }
            }
        }
    }

    SizeType indeg( const NodeIterator& u)
    {
        SizeType sum = 0;
        for( EdgeIterator e = beginEdges(u), end = endEdges(u); e != end; ++e)
        {
            if (e->isInEdge()) ++sum;
        } 
        return sum;
    }

    NodeDescriptor insertNode() 
    { 
        NodeDescriptor m_auxNodeDescriptor = new CPGNode<Vtype,Etype>*();
        *m_auxNodeDescriptor = 0;
        //std::cout << "\nMalloced for node" << m_auxNodeDescriptor << std::endl;
        CPGNode<Vtype,Etype> newNode;
        newNode.setDescriptor( m_auxNodeDescriptor);
        NodeIterator m_auxNodeIterator = m_nodes.optimalInsert( newNode);
        //NodeIterator m_auxNodeIterator = m_nodes.insert( m_nodes.end(),newNode);
        //NodeIterator m_auxNodeIterator = m_nodes.insert( m_nodes.begin(),newNode);
        *m_auxNodeDescriptor = m_auxNodeIterator.getAddress();
        m_lastPushedNode = m_nodes.end();
        m_currentPushedNode = m_nodes.end();
        return m_auxNodeDescriptor;
    }

    NodeDescriptor insertNodeBefore( NodeDescriptor descriptor) 
    { 
        NodeDescriptor m_auxNodeDescriptor = new CPGNode<Vtype,Etype>*();
        
        *m_auxNodeDescriptor = 0;
        CPGNode<Vtype,Etype> newNode;
        newNode.setDescriptor( m_auxNodeDescriptor);


        NodeIterator m_auxNodeIterator = m_nodes.insert( getNodeIterator(descriptor), newNode);

        *m_auxNodeDescriptor = m_auxNodeIterator.getAddress();

        m_lastPushedNode = m_nodes.end();
        m_currentPushedNode = m_nodes.end();
        return m_auxNodeDescriptor;
    }
    
    bool isValid()
    {
        CPGEdge<Vtype,Etype>* lastEdge = 0;

        bool valid = true;
        for( NodeIterator u = beginNodes(), end = endNodes(); u != end; ++u)
        {
            if( u->hasEdges())
            {
                if( lastEdge > u->m_firstEdge)
                {
                    valid = false;
                }
                lastEdge = u->m_firstEdge;
            }
        }        
        return valid;
    }
    
    SizeType memUsage()   
    { 
        std::cout << "Graph mem Usage\t\tNodes\tEdges\tInEdges\n";
        std::cout << "\tNumber:\t\t" << m_nodes.size() << "\t" << m_edges.size() << std::endl;
        std::cout << "\tReserved:\t" << m_nodes.capacity() << "\t" << m_edges.capacity() << std::endl;
        std::cout << "\tSize:\t\t" << CPGNode<Vtype,Etype>::memUsage() << "\t" << CPGEdge<Vtype,Etype>::memUsage() << std::endl;
        
        m_nodes.getMemoryUsage();

        /*std::cout << "\tNodes:\t\t" << m_nodes.size() << "\tReserved:" << m_nodes.capacity() << "\n"; 
        std::cout << "\tEdges:\t\t" << m_edges.size() << "\tReserved:" << m_edges.capacity() << "\n"; 
        std::cout << "\tInEdges:\t" << m_inEdges.size() << "\tReserved:" << m_inEdges.capacity() << "\n"; 
        std::cout << "\tNode Size:\t" << CPGNode<Vtype,Etype>::memUsage() << "bytes" << "\t";
        std::cout << "\tEdge Size:\t" << CPGEdge<Vtype,Etype>::memUsage() << "bytes" << "\t";
        std::cout << "\tBWEdge Size:\t" << PMAInEdge<Vtype,Etype>::memUsage() << "bytes" << std::endl;*/

        return  CPGNode<Vtype,Etype>::memUsage() * m_nodes.capacity() + 
                CPGEdge<Vtype,Etype>::memUsage() * m_edges.capacity();
    }
    
    EdgeIterator nilEdge() 
    { 
        return m_edges.end();
    }

    NodeIterator nilNode()
    {
        return m_nodes.end();
    }  
    
    SizeType outdeg( const NodeIterator& u)
    {
        SizeType sum = 0;
        for( EdgeIterator e = beginEdges(u), end = endEdges(u); e != end; ++e)
        {
            if (e->isOutEdge()) ++sum;
        } 
        return sum;
    }

    void printDot(std::ostream& out)
    {
        out << "digraph BST {\n\tnode [fontname=\"Arial\"]\n";
        m_nodes.printDot( out, "n", "e");
        m_edges.printDot( out, "e");
        out << "}";
    }
    
    void printDotRange( const char* filename, const NodeIterator& firstNode, const NodeIterator& lastNode)
    {
        std::ofstream out( filename);
        out << "digraph BST {\n\tnode [fontname=\"Arial\"]\n";
        m_nodes.printDotRange( out, firstNode, lastNode, "n", "e");
        m_edges.printDotRange( out, beginEdges(firstNode), endEdges(lastNode), "e");
        out << "}";
        out.close();
    }

    void pushEdge( NodeDescriptor uD, NodeDescriptor vD) 
    { 
        //assert( isValid());
        NodeIterator u = getNodeIterator( uD);
        NodeIterator v = getNodeIterator( vD);
        
        if( u != m_currentPushedNode)
        {
            m_lastPushedNode = m_currentPushedNode;
            m_currentPushedNode = u;
        }
        
        assert( u != v);
        assert( (!u->hasEdges()) || (u->m_firstEdge != u->m_lastEdge));
        assert( (!v->hasEdges()) || (v->m_firstEdge != v->m_lastEdge));
        
        NodeIterator w;
        EdgeIterator e;
        InEdgeIterator k;

        w = v;
        ++w;
            w = findNextNodeWithEdges(w);
            if( w != m_nodes.end()) k = getEdgeIteratorAtAddress(w->m_firstEdge);
            else                    k = m_edges.end();
            

        CPGEdge<Vtype, Etype> newEdge( v.getAddress());
        CPGEdge<Vtype, Etype> newInEdge( u.getAddress());


        m_edges.push_back( newEdge);
        e = m_edges.end();
        --e;
        
        k = m_edges.insert( k, newInEdge);        

        //e->m_adjacentNode = v.getPoolIndex();
        e->m_oppositeEdge = k.getAddress();

        //k->m_adjacentNode = u.getPoolIndex();
        k->m_oppositeEdge = e.getAddress();  

        assert( k->m_adjacentNode != 0);

        bool uHadEdges = true;
        bool vHadEdges = true;

        if( !u->hasEdges())
        {
            uHadEdges = false;
            u->m_firstEdge = e.getAddress();
        }
        
        if( !v->hasEdges())
        {
            vHadEdges = false;
            v->m_firstEdge = k.getAddress();
        }

        if( !uHadEdges)
        {
            //w = findPreviousNodeWithEdges(u);
            w = m_lastPushedNode;
            assert( w != u);
            if( w != m_nodes.end())
            {
                w->m_lastEdge = e.getAddress();
                assert( w->m_lastEdge != w->m_firstEdge);
                assert( (w->m_lastEdge > w->m_firstEdge) || (!w->m_lastEdge));
            }
        } 
        
        if( !vHadEdges)
        {
            w = findPreviousNodeWithEdges(v);
            assert( w != v);
            if( w != m_nodes.end())
            {
                w->m_lastEdge = k.getAddress();
                assert( w->m_lastEdge != w->m_firstEdge);
                assert( (w->m_lastEdge > w->m_firstEdge) || (!w->m_lastEdge));
            }
            w = v;
            ++w;    
            w = findNextNodeWithEdges(w);
            if( w != m_nodes.end())
            {
                v->m_lastEdge = w->m_firstEdge;
            }
        } 

        assert( (u->m_lastEdge > u->m_firstEdge) || (!(u->m_lastEdge)));
        assert( (v->m_lastEdge > v->m_firstEdge) || (!(v->m_lastEdge)));

        assert( u->hasEdges() && v->hasEdges());


        /*std::stringstream sf;
        sf << "/home/michai/Projects/Graphs/GraphViz/" << m_edges.size() << "f.dot";
        s = sf.str();
        out.open(s.c_str());
        printDot(out);
        out.close();*/

    }

    void reserve( const SizeType& numNodes, const SizeType& numEdges)
    {
        std::cout << "\tReserving space for nodes\t";
        m_nodes.reserve( numNodes);
        std::cout << "\tReserving space for edges\t";
        m_edges.reserve( numEdges);
    }

    void sanitizeDescriptor( const NodeIterator& u)
    {
        NodeDescriptor descriptor = getDescriptor(u);
        *descriptor = u.getAddress();
    }


    void setDescriptor( NodeIterator& u, NodeDescriptor& uD)
    {
        u->setDescriptor( uD);
        *uD = u.getAddress();
    }


private:
    PackedMemoryArray< CPGNode< Vtype, Etype> >     m_nodes;
    PackedMemoryArray< CPGEdge< Vtype, Etype> >     m_edges; 

    CPGNodeObserver< Vtype, Etype>*                 m_nodeObserver;
    CPGEdgeObserver< Vtype, Etype>*                 m_edgeObserver;
    SingleEdgeObserver< Vtype, Etype>*              m_singleEdgeObserver;


    NodeIterator m_lastPushedNode, m_currentPushedNode;

    MersenneTwister m_random;
};

template<typename Vtype, typename Etype>
class CPGEdge : public Etype
{

public:
    //typedef typename CompressedPackedGraphImpl<Vtype,Etype>::NodeDescriptor   NodeDescriptor;
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::SizeType         SizeType;
    
    enum Direction
    {
        OUT,
        IN,
        BI
    };


    CPGEdge( unsigned int init = 0): Etype(),
                        m_adjacentNode(0), 
                        m_oppositeEdge(0)
    {
    }
   
    CPGEdge( CPGNode<Vtype,Etype>* adjacentNode):
            Etype(),
			m_adjacentNode(adjacentNode),
			m_oppositeEdge(0)
    {
    }

    bool isOutEdge()
    {
        return (m_direction == OUT) || (m_direction == BI);
    }

    bool isInEdge()
    {
        return (m_direction == IN) || (m_direction == BI);
    }

    static unsigned int memUsage() 
    {
        return sizeof(CPGEdge);
    }

    
    bool operator ==( const CPGEdge& other) const
	{
        return ( m_oppositeEdge == other.m_oppositeEdge) && (m_adjacentNode == other.m_adjacentNode);
	}
	
	bool operator !=( const CPGEdge& other) const
	{
		return (m_oppositeEdge != other.m_oppositeEdge) || (m_adjacentNode != other.m_adjacentNode);
	}


    friend std::ostream& operator << ( std::ostream& out, CPGEdge other)
	{
        if( other.m_adjacentNode != 0)
        {
            out << "{" << other.m_adjacentNode << "|";
        }
        else
        {
            out << "{-|";
        }
        /*if( other.m_InEdge != 0)
        {   
            out << other.m_InEdge << "|";
        }
        else
        {
            out << "-|";
        }*/
        //out << "}|" << other.m_descriptor << "}";
        out << "}";
		return out;
	}


    CPGNode<Vtype,Etype>*       m_adjacentNode;
    CPGEdge<Vtype,Etype>*       m_oppositeEdge;
    CPGEdge::Direction          m_direction;   
};


template<typename Vtype, typename Etype>
class CPGNode : public GraphElement< Vtype, typename CompressedPackedGraphImpl<Vtype,Etype>::NodeDescriptor>
{
public:
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::NodeDescriptor NodeDescriptor;
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::SizeType       SizeType;

    CPGNode( unsigned int init = 0):GraphElement< Vtype, NodeDescriptor>(), 
                                    m_firstEdge(0), 
                                    m_lastEdge(0)
    {
    }

    CPGNode( NodeDescriptor descriptor):GraphElement< Vtype, NodeDescriptor>(descriptor), 
                                    m_firstEdge(0), 
                                    m_lastEdge(0)
    {
    }
    
    bool hasEdges() const
    {
        return m_firstEdge != 0;
    }


    static unsigned int memUsage() 
    {
        return sizeof(CPGNode);
    }

    friend std::ostream& operator << ( std::ostream& out, CPGNode other)
	{
        if( other.hasEdges())
        {
            out << "{" << other.m_firstEdge << "|" << other.m_lastEdge << "}";
        }
        else
        {
            out << "{-|-}";
        }
        /*if( other.hasInEdges())
        {   
            out << "|{" << other.m_firstInEdge << "|" << other.m_lastInEdge << "}";
        }
        else
        {
            out << "|{-|-}";
        }*/
        //out << "}";
        //out << "}|" << other.m_descriptor << "}";
		return out;
	}


    CPGEdge<Vtype,Etype>*     m_firstEdge;
    CPGEdge<Vtype,Etype>*     m_lastEdge;
};


template<typename Vtype, typename Etype>
class CPGNodeObserver : public PackedMemoryArray< CPGNode<Vtype,Etype> >::Observer
{
public:

    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::EdgeIterator   EdgeIterator;
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::InEdgeIterator   InEdgeIterator;
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::NodeIterator   NodeIterator;
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::SizeType       SizeType;

    CPGNodeObserver( CompressedPackedGraphImpl<Vtype,Etype>* G): m_G(G)
    {
    }

    void move( CPGNode<Vtype,Etype>* source, CPGNode<Vtype,Etype>* sourcePool, CPGNode<Vtype,Etype>* destination, CPGNode<Vtype,Etype>* destinationPool, const CPGNode<Vtype,Etype>& node)
    {
        assert( node != m_G->m_nodes.getEmptyElement());
       
        if( node.hasEdges())
        {
            EdgeIterator e = m_G->getEdgeIteratorAtAddress( node.m_firstEdge);
            EdgeIterator endEdges = m_G->getEdgeIteratorAtAddress( node.m_lastEdge);
            while ( e != endEdges)
            {
                e->m_oppositeEdge->m_adjacentNode = destination;
                ++e;
            }
        }

        //std::cout << "Moving " << source << " to " << destination << std::endl;
		*(node.getDescriptor()) = destination;        
    }
private:
    CompressedPackedGraphImpl<Vtype,Etype>* m_G;
    EdgeIterator                        e, end;
};




template<typename Vtype, typename Etype>
class SingleEdgeObserver : public PackedMemoryArray< CPGEdge<Vtype,Etype> >::Observer
{
public:
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::SizeType       SizeType;


    SingleEdgeObserver( CompressedPackedGraphImpl<Vtype,Etype>* G, CPGEdge<Vtype,Etype>* address = 0): m_G(G), m_address(address)
    {
    }

    void move( CPGEdge<Vtype,Etype>* source, CPGEdge<Vtype,Etype>* sourcePool, CPGEdge<Vtype,Etype>* destination, CPGEdge<Vtype,Etype>* destinationPool, const CPGEdge<Vtype,Etype>& edge)
    {
        if( source == destination) return;
        assert( edge != m_G->m_edges.getEmptyElement());
        if( source == m_address)
            m_address = destination;        
    }

    void observe(CPGEdge<Vtype,Etype>* address)
    {
        m_address = address;
    }

    CPGEdge<Vtype,Etype>* getCurrentAddress()
    {
        return m_address;
    }

private:
    CompressedPackedGraphImpl<Vtype,Etype>* m_G;
    CPGEdge<Vtype,Etype>*   m_address;
};




template<typename Vtype, typename Etype>
class CPGEdgeObserver : public PackedMemoryArray< CPGEdge<Vtype,Etype> >::Observer
{
public:
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::SizeType       SizeType;
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::NodeIterator   NodeIterator;
    typedef typename CompressedPackedGraphImpl<Vtype,Etype>::InEdgeIterator   InEdgeIterator;

    CPGEdgeObserver( CompressedPackedGraphImpl<Vtype,Etype>* G): m_G(G),lastChangedNode(0),lastChangedTailNode(0)
    {
    }

    void move( CPGEdge<Vtype,Etype>* source, CPGEdge<Vtype,Etype>* sourcePool, CPGEdge<Vtype,Etype>* destination, CPGEdge<Vtype,Etype>* destinationPool, const CPGEdge<Vtype,Etype>& edge)
    {
        if( source == destination) return;
        assert( edge != m_G->m_edges.getEmptyElement());

        if( !(edge.m_oppositeEdge)) return;      
        
        edge.m_oppositeEdge->m_oppositeEdge = destination;


        if( (edge.m_oppositeEdge->m_adjacentNode->m_firstEdge == source) 
            && (edge.m_oppositeEdge->m_adjacentNode != lastChangedNode))
        {
            NodeIterator u = m_G->getNodeIteratorAtAddress(edge.m_oppositeEdge->m_adjacentNode);
            m_G->setFirstEdge( u, destination);   
            lastChangedNode = edge.m_oppositeEdge->m_adjacentNode;
        }
        
    }

    void reset()
    {
        lastChangedNode = 0;
        lastChangedTailNode = 0;
    }
private:
    CompressedPackedGraphImpl<Vtype,Etype>* m_G;
    CPGNode<Vtype,Etype>*  lastChangedNode;
    CPGNode<Vtype,Etype>*  lastChangedTailNode;
};


#endif //CompressedPackedGraphImpl_H
