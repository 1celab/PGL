#ifndef PackedGraphImpl_H
#define PackedGraphImpl_H

#include <Utilities/mersenneTwister.h>
#include <vector>

template<typename Vtype, typename Etype>
class PGNode;
template<typename Vtype, typename Etype>
class PGEdge;
template<typename Vtype, typename Etype>
class PGInEdge;
template<typename Vtype, typename Etype>
class PGNodeObserver;
template<typename Vtype, typename Etype>
class PGEdgeObserver;
template<typename Vtype, typename Etype>
class PGInEdgeObserver;

template<typename Vtype, typename Etype>
class PackedGraphImpl
{

    friend class PGNodeObserver< Vtype, Etype>;
    friend class PGEdgeObserver< Vtype, Etype>;
    friend class PGInEdgeObserver< Vtype, Etype>;

public:
    
    typedef unsigned int                                                        SizeType;
    typedef unsigned int*                                              NodeDescriptor;
    typedef unsigned int*                                              EdgeDescriptor;
    typedef typename PackedMemoryArray< PGNode< Vtype, Etype> >::Iterator      NodeIterator;
    typedef typename PackedMemoryArray< PGEdge< Vtype, Etype> >::Iterator      EdgeIterator;
    typedef typename PackedMemoryArray< PGInEdge< Vtype, Etype> >::Iterator    InEdgeIterator;
    

    PackedGraphImpl()
    {
        m_nodeObserver = new PGNodeObserver< Vtype, Etype>(this);
        m_nodes.registerObserver( m_nodeObserver);
        
        m_edgeObserver = new PGEdgeObserver< Vtype, Etype>(this);
        m_edges.registerObserver( m_edgeObserver);       
   
        m_InEdgeObserver = new PGInEdgeObserver< Vtype, Etype>(this);
        m_inEdges.registerObserver( m_InEdgeObserver);
    }

    ~PackedGraphImpl() 
    {
        delete m_nodeObserver;
        delete m_edgeObserver;
        delete m_InEdgeObserver;

        NodeIterator end = m_nodes.end();
        EdgeIterator end_edges = m_edges.end();

        for( NodeIterator u = m_nodes.begin(); u != end; ++u)
        {
            delete u->getDescriptor();
        }

        for( EdgeIterator e = m_edges.begin(); e != end_edges; ++e)
        {
            //std::cout << "Erasing " << e->getDescriptor() << std::endl;
            delete e->getDescriptor();
        }
    }
    
    EdgeIterator beginEdges()
    {
        return m_edges.begin();
    }
    
    EdgeIterator beginEdges( const NodeIterator& u)  
    { 
        return getEdgeIteratorAtIndex(u->m_firstEdge);
    }
    
    InEdgeIterator beginInEdges( const NodeIterator& u)  
    { 
        return getInEdgeIteratorAtIndex( u->m_firstInEdge);
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
        m_inEdges.clear();
    }
    
    void compress()
    {
        std::cout << "Compressing graph...\n";
        m_nodes.compress();
        m_edges.compress();
        m_inEdges.compress();
    }
    
    EdgeIterator endEdges()
    {
        return m_edges.end();
    }
    
    EdgeIterator endEdges( const NodeIterator& u)  
    { 
        return getEdgeIteratorAtIndex( u->m_lastEdge);
    }
       
    InEdgeIterator endInEdges( const NodeIterator& u)  
    { 
        return getInEdgeIteratorAtIndex( u->m_lastInEdge);
    }
    
    NodeIterator endNodes()
    { 
        return m_nodes.end();
    }
    
    void eraseEdge( EdgeDescriptor descriptor) 
    { 
        EdgeIterator e = getEdgeIterator(descriptor);
        InEdgeIterator k = getInEdgeIterator( e);
        NodeIterator u = getAdjacentNodeIterator(k);
        NodeIterator v = getAdjacentNodeIterator(e);
        
        //std::cout << "Deleting\t" << e->getDescriptor() << std::endl;
        //delete (e->getDescriptor());
        //descriptor = 0;
        
        // if we erase a node's first edge
        if( u->m_firstEdge == m_edges.getPoolIndex(e))
        {
            NodeIterator w,z;
            EdgeIterator f = e;
            ++f;
            
            // if it has no more edges
            if( m_edges.getPoolIndex(f) == u->m_lastEdge)
            {
                setFirstEdge(u,std::numeric_limits<SizeType>::max());
            }
            else
            {
                setFirstEdge(u, m_edges.getPoolIndex(f));
            }
        }

        if( v->m_firstInEdge == m_inEdges.getPoolIndex(k) )
		{
		    NodeIterator w,z;
			InEdgeIterator f = k;
		    ++f;
			if( m_inEdges.getPoolIndex(f) == v->m_lastInEdge)
            {
                setFirstInEdge( v, std::numeric_limits<SizeType>::max());
            }
            else
            {
                setFirstInEdge( v, m_inEdges.getPoolIndex(f));
            }
		}

        e->m_InEdge = std::numeric_limits<SizeType>::max();
        m_edges.erase( e);
        k->m_edge = std::numeric_limits<SizeType>::max();
        m_inEdges.erase( k);  
        delete descriptor; 
    }

    void eraseNode( NodeDescriptor uD) 
    { 
        NodeIterator u = getNodeIterator( uD);
        std::vector< EdgeDescriptor> edges;
        //uD = 0;
        //u->setDescriptor(0);
        m_nodes.erase(u);
        delete uD;
    }
    
    void expand()
    {
        std::cout << "Expanding graph...\n";
        m_nodes.expand();
        m_edges.expand();
        m_inEdges.expand();
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

    NodeIterator findNextNodeWithInEdges( NodeIterator u)
    {
        ++u;
        NodeIterator end = m_nodes.end();
        for( ; u != end; ++u)
        {
            if( u->hasInEdges())
            {
                //std::cout << "Found next back edge at distance: " << m_auxNodeIterator - u  << std::endl;
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
    
    NodeIterator findPreviousNodeWithInEdges( const NodeIterator& u)
    {
        NodeIterator begin = m_nodes.begin();
        NodeIterator m_auxNodeIterator = u;
        if( u == begin) return m_nodes.end();
        --m_auxNodeIterator;
        while( m_auxNodeIterator != begin)
        {
            if( m_auxNodeIterator->hasInEdges())
            {
                return m_auxNodeIterator;
            }   
            --m_auxNodeIterator;
        }
        return m_auxNodeIterator->hasInEdges()? m_auxNodeIterator:m_nodes.end();
    }
    
    bool hasEdge( const EdgeDescriptor& descriptor)
    {
        return descriptor != 0;
    }

    bool hasEdges( const NodeIterator& u)
    {        
        return u->hasEdges();
    }

    bool hasInEdges( const NodeIterator& u)
    {
        return u->hasInEdges();
    }
  
    bool hasNode( const NodeDescriptor& descriptor)
    {
        return descriptor != 0;
    }
    
    NodeIterator getAdjacentNodeIterator( const EdgeIterator& e)
    { 
        return m_nodes.atIndex( e->m_adjacentNode);
    }
    
    NodeIterator getAdjacentNodeIterator( const InEdgeIterator& k)
    {
        return m_nodes.atIndex( k->m_adjacentNode);
    }
    
    EdgeDescriptor getDescriptor( const EdgeIterator& e) const
    {
        return e->getDescriptor();
    }
	
    EdgeDescriptor getDescriptor( const InEdgeIterator& k)
    {
        return getEdgeIterator(k)->getDescriptor();
    }
    
    NodeDescriptor getDescriptor( const NodeIterator& u) const
    {
        return u->getDescriptor();
    }

    EdgeIterator getEdgeIterator( const EdgeDescriptor& descriptor) 
    { 
        return m_edges.atIndex(*descriptor);
    }
    
    EdgeIterator getEdgeIterator( const InEdgeIterator& k) 
    {
        return m_edges.atIndex(k->m_edge);
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
        return m_inEdges.atIndex( e->m_InEdge);
    }
    
    InEdgeIterator getInEdgeIteratorAtIndex( PGInEdge<Vtype,Etype>* addr)
    {
        return m_inEdges.atIndex(addr);
    }
    
    InEdgeIterator getInEdgeIteratorAtIndex( const SizeType& position)
    {
        return m_inEdges.atIndex(position);
    } 

    NodeIterator getNodeIterator( const NodeDescriptor& descriptor) 
    { 
        return m_nodes.atIndex(*descriptor);
    }

    NodeIterator getNodeIteratorAtIndex( const SizeType& position) 
    { 
        return m_nodes.atIndex(position);
    }


    EdgeDescriptor insertEdge( NodeDescriptor uD, NodeDescriptor vD) 
    { 
        //assert( isValid());
        NodeIterator u = getNodeIterator( uD);
        NodeIterator v = getNodeIterator( vD);
        
        assert( u != v);
        assert( (!u->hasEdges()) || (u->m_firstEdge != u->m_lastEdge));
        assert( (!v->hasInEdges()) || (v->m_firstInEdge != v->m_lastInEdge));
        
        NodeIterator w;
        EdgeIterator e;
        InEdgeIterator k;
        
            w = findNextNodeWithEdges(u);
            if( w != m_nodes.end()) e = getEdgeIteratorAtIndex(w->m_firstEdge);
            else                    e = m_edges.end();

            w = findNextNodeWithInEdges(v);
            if( w != m_nodes.end()) k = getInEdgeIteratorAtIndex(w->m_firstInEdge);
            else                    k = m_inEdges.end();
            
        EdgeDescriptor m_auxEdgeDescriptor = new unsigned int();      
        //std::cout << "Malloced\t" << m_auxEdgeDescriptor << std::endl;  
        *m_auxEdgeDescriptor = std::numeric_limits<SizeType>::max();

        PGEdge<Vtype, Etype> newEdge( m_nodes.getPoolIndex(v), m_auxEdgeDescriptor);
        PGInEdge<Vtype, Etype> newInEdge( m_nodes.getPoolIndex(u));
        e = m_edges.insert( e, newEdge);
        k = m_inEdges.insert( k, newInEdge);        
        *m_auxEdgeDescriptor = m_edges.getPoolIndex(e);
        e->m_InEdge = m_inEdges.getPoolIndex(k);
        k->m_edge = m_edges.getPoolIndex(e);  

        assert( k->m_adjacentNode != std::numeric_limits<SizeType>::max());

        if( !u->hasEdges())
        {
            setFirstEdge( u, m_edges.getPoolIndex(e));
            w = findNextNodeWithEdges(u);
            if( w != m_nodes.end())
            {
                u->m_lastEdge = w->m_firstEdge;
            }
        } 
        
        if( !v->hasInEdges())
        {
            setFirstInEdge( v, m_inEdges.getPoolIndex(k));
            w = findNextNodeWithInEdges(v);
            if( w != m_nodes.end())
            {
                v->m_lastInEdge = w->m_firstInEdge;
            }
        } 

        assert( (u->m_lastEdge > u->m_firstEdge) || (!(u->m_lastEdge)));
        assert( (v->m_lastInEdge > v->m_firstInEdge) || (!(v->m_lastInEdge)));
        assert( u->hasEdges() && v->hasInEdges());

        return m_auxEdgeDescriptor;
    }

    void setFirstEdge( NodeIterator u, const SizeType& address)
    {
        u->m_firstEdge = address;
        if( address != std::numeric_limits<SizeType>::max())
        {
            u = findPreviousNodeWithEdges(u);
            if( u != m_nodes.end())
            {
                u->m_lastEdge = address;
            }
        }
        else
        {
            u->m_lastEdge = std::numeric_limits<SizeType>::max();
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
                    u->m_lastEdge = std::numeric_limits<SizeType>::max();
                }
            }
        }
    }
    
    void setFirstInEdge( NodeIterator u, const SizeType& address)
    {
        u->m_firstInEdge = address;
        if( address != std::numeric_limits<SizeType>::max())
        {
            u = findPreviousNodeWithInEdges(u);
            if( u != m_nodes.end())
            {
                u->m_lastInEdge = address;
            }
        }
        else
        {
            u->m_lastInEdge = std::numeric_limits<SizeType>::max();
            u = findPreviousNodeWithInEdges(u);
            if( u != m_nodes.end())
            {
                NodeIterator v = findNextNodeWithInEdges(u);
                if( v != m_nodes.end())
                {
                    u->m_lastInEdge = v->m_firstInEdge;
                }
                else
                {
                    u->m_lastInEdge = std::numeric_limits<SizeType>::max();
                }
            }
        }
    }

    NodeDescriptor insertNode() 
    { 
        NodeDescriptor m_auxNodeDescriptor = new unsigned int();
        *m_auxNodeDescriptor = std::numeric_limits<SizeType>::max();
        //std::cout << "\nMalloced for node" << m_auxNodeDescriptor << std::endl;
        PGNode<Vtype,Etype> newNode;
        newNode.setDescriptor( m_auxNodeDescriptor);
        NodeIterator m_auxNodeIterator = m_nodes.optimalInsert( newNode);
        //NodeIterator m_auxNodeIterator = m_nodes.insert( m_nodes.end(),newNode);
        //NodeIterator m_auxNodeIterator = m_nodes.insert( m_nodes.begin(),newNode);
        *m_auxNodeDescriptor = m_nodes.getPoolIndex(m_auxNodeIterator);
        m_lastPushedNode = m_nodes.end();
        m_currentPushedNode = m_nodes.end();
        return m_auxNodeDescriptor;
    }

    NodeDescriptor insertNodeBefore( NodeDescriptor descriptor) 
    { 
        NodeDescriptor m_auxNodeDescriptor = new PGNode<Vtype,Etype>*();
        
        *m_auxNodeDescriptor = std::numeric_limits<SizeType>::max();
        PGNode<Vtype,Etype> newNode;
        newNode.setDescriptor( m_auxNodeDescriptor);


        NodeIterator m_auxNodeIterator = m_nodes.insert( getNodeIterator(descriptor), newNode);

        *m_auxNodeDescriptor = m_nodes.getPoolIndex(m_auxNodeIterator);

        m_lastPushedNode = m_nodes.end();
        m_currentPushedNode = m_nodes.end();
        return m_auxNodeDescriptor;
    }
    
    bool isValid()
    {
        SizeType lastEdge = std::numeric_limits<SizeType>::max();
        SizeType lastInEdge = std::numeric_limits<SizeType>::max();

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
            if( u->hasInEdges())
            {
                if( lastInEdge > u->m_firstInEdge)
                {
                    valid = false;
                }
                lastInEdge = u->m_firstInEdge;
            }
        }        
        return valid;
    }
    
    SizeType memUsage()   
    { 
        std::cout << "Graph mem Usage\t\tNodes\tEdges\tInEdges\n";
        std::cout << "\tNumber:\t\t" << m_nodes.size() << "\t" << m_edges.size() << "\t" << m_inEdges.size() << std::endl;
        std::cout << "\tReserved:\t" << m_nodes.capacity() << "\t" << m_edges.capacity() << "\t" << m_inEdges.capacity() << std::endl;
        std::cout << "\tSize:\t\t" << PGNode<Vtype,Etype>::memUsage() << "\t" << PGEdge<Vtype,Etype>::memUsage() << "\t" << PGInEdge<Vtype,Etype>::memUsage() << std::endl;
        
        m_nodes.getMemoryUsage();

        /*std::cout << "\tNodes:\t\t" << m_nodes.size() << "\tReserved:" << m_nodes.capacity() << "\n"; 
        std::cout << "\tEdges:\t\t" << m_edges.size() << "\tReserved:" << m_edges.capacity() << "\n"; 
        std::cout << "\tInEdges:\t" << m_inEdges.size() << "\tReserved:" << m_inEdges.capacity() << "\n"; 
        std::cout << "\tNode Size:\t" << PGNode<Vtype,Etype>::memUsage() << "bytes" << "\t";
        std::cout << "\tEdge Size:\t" << PGEdge<Vtype,Etype>::memUsage() << "bytes" << "\t";
        std::cout << "\tBWEdge Size:\t" << PGInEdge<Vtype,Etype>::memUsage() << "bytes" << std::endl;*/

        return  PGNode<Vtype,Etype>::memUsage() * m_nodes.capacity() + 
                PGEdge<Vtype,Etype>::memUsage() * m_edges.capacity() + 
                PGInEdge<Vtype,Etype>::memUsage() * m_inEdges.capacity();
    }
    
    EdgeIterator nilEdge() 
    { 
        return m_edges.end();
    }

    NodeIterator nilNode()
    {
        return m_nodes.end();
    }  
    
    void printDot(std::ostream& out)
    {
        out << "digraph BST {\n\tnode [fontname=\"Arial\"]\n";
        m_nodes.printDot( out, "n", "e");
        m_edges.printDot( out, "e", "b");
        m_inEdges.printDot( out,"b");
        out << "}";
    }
    
    void printDotRange( const char* filename, const NodeIterator& firstNode, const NodeIterator& lastNode)
    {
        std::ofstream out( filename);
        out << "digraph BST {\n\tnode [fontname=\"Arial\"]\n";
        m_nodes.printDotRange( out, firstNode, lastNode, "n", "e");
        m_edges.printDotRange( out, beginEdges(firstNode), endEdges(lastNode), "e", "b");
        m_inEdges.printDotRange( out, beginInEdges(firstNode), endInEdges(lastNode), "b");
        out << "}";
        out.close();
    }

    EdgeDescriptor pushEdge( NodeDescriptor uD, NodeDescriptor vD) 
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
        assert( (!v->hasInEdges()) || (v->m_firstInEdge != v->m_lastInEdge));
        
        NodeIterator w;
        EdgeIterator e;
        InEdgeIterator k;

        w = v;
        ++w;
            w = findNextNodeWithInEdges(w);
            if( w != m_nodes.end()) k = getInEdgeIteratorAtIndex(w->m_firstInEdge);
            else                    k = m_inEdges.end();
            
        EdgeDescriptor m_auxEdgeDescriptor = new PGEdge<Vtype,Etype>*();        
        *m_auxEdgeDescriptor = std::numeric_limits<SizeType>::max();

        PGEdge<Vtype, Etype> newEdge( m_nodes.getPoolIndex(v), m_auxEdgeDescriptor);
        PGInEdge<Vtype, Etype> newInEdge( m_nodes.getPoolIndex(u));

        //std::cout << "\nMalloced for edge" << m_auxEdgeDescriptor << std::endl;
        //newEdge.setDescriptor( m_auxEdgeDescriptor);
        m_edges.push_back( newEdge);
        e = m_edges.end();
        --e;
        
        k = m_inEdges.insert( k, newInEdge);        
        *m_auxEdgeDescriptor = m_edges.getPoolIndex(e);

        //e->m_adjacentNode = v.getPoolIndex();
        e->m_InEdge = m_inEdges.getPoolIndex(k);
        //e->setDescriptor(m_auxEdgeDescriptor);

        //k->m_adjacentNode = u.getPoolIndex();
        k->m_edge = m_edges.getPoolIndex(e);  
        //k->setDescriptor(m_auxEdgeDescriptor);

        assert( k->m_adjacentNode != std::numeric_limits<SizeType>::max());

        bool uHadEdges = true;
        bool vHadInEdges = true;

        if( !u->hasEdges())
        {
            uHadEdges = false;
            u->m_firstEdge = m_edges.getPoolIndex(e);
        }
        
        if( !v->hasInEdges())
        {
            vHadInEdges = false;
            v->m_firstInEdge = m_inEdges.getPoolIndex(k);
        }

        if( !uHadEdges)
        {
            //w = findPreviousNodeWithEdges(u);
            w = m_lastPushedNode;
            assert( w != u);
            if( w != m_nodes.end())
            {
                w->m_lastEdge = m_edges.getPoolIndex(e);
                assert( w->m_lastEdge != w->m_firstEdge);
                assert( (w->m_lastEdge > w->m_firstEdge) || (!w->m_lastEdge));
            }
        } 
        
        if( !vHadInEdges)
        {
            w = findPreviousNodeWithInEdges(v);
            assert( w != v);
            if( w != m_nodes.end())
            {
                w->m_lastInEdge = m_inEdges.getPoolIndex(k);
                assert( w->m_lastInEdge != w->m_firstInEdge);
                assert( (w->m_lastInEdge > w->m_firstInEdge) || (!w->m_lastInEdge));
            }
            w = v;
            ++w;    
            w = findNextNodeWithInEdges(w);
            if( w != m_nodes.end())
            {
                v->m_lastInEdge = w->m_firstInEdge;
            }
        } 

        assert( (u->m_lastEdge > u->m_firstEdge) || (!(u->m_lastEdge)));
        assert( (v->m_lastInEdge > v->m_firstInEdge) || (!(v->m_lastInEdge)));

        assert( u->hasEdges() && v->hasInEdges());


        /*std::stringstream sf;
        sf << "/home/michai/Projects/Graphs/GraphViz/" << m_edges.size() << "f.dot";
        s = sf.str();
        out.open(s.c_str());
        printDot(out);
        out.close();*/

        return m_auxEdgeDescriptor;
    }

    void reserve( const SizeType& numNodes, const SizeType& numEdges)
    {
        std::cout << "\tReserving space for nodes\t";
        m_nodes.reserve( numNodes);
        std::cout << "\tReserving space for edges\t";
        m_edges.reserve( numEdges);
        std::cout << "\tReserving space for inedges\t";
        m_inEdges.reserve( numEdges);
    }

    void sanitizeDescriptor( const NodeIterator& u)
    {
        NodeDescriptor descriptor = getDescriptor(u);
        *descriptor = m_nodes.getPoolIndex(u);
    }

    void sanitizeDescriptor( const EdgeIterator& e)
    {
        EdgeDescriptor descriptor = getDescriptor(e);
        *descriptor = m_edges.getPoolIndex(e);
    }

    void setDescriptor( NodeIterator& u, NodeDescriptor& uD)
    {
        u->setDescriptor( uD);
        *uD = m_nodes.getPoolIndex(u);
    }

    void setDescriptor( EdgeIterator& e, EdgeDescriptor& eD)
    {
        e->setDescriptor( eD);
        *eD = m_edges.getPoolIndex(e);
    }

private:
    PackedMemoryArray< PGNode< Vtype, Etype> >     m_nodes;
    PackedMemoryArray< PGEdge< Vtype, Etype> >     m_edges; 
    PackedMemoryArray< PGInEdge< Vtype, Etype> >   m_inEdges;
    PGNodeObserver< Vtype, Etype>*                 m_nodeObserver;
    PGEdgeObserver< Vtype, Etype>*                 m_edgeObserver;
    PGInEdgeObserver< Vtype, Etype>*               m_InEdgeObserver;

    NodeIterator m_lastPushedNode, m_currentPushedNode;

    MersenneTwister m_random;
};

template<typename Vtype, typename Etype>
class PGEdge : public GraphElement< Etype, typename PackedGraphImpl<Vtype,Etype>::EdgeDescriptor>
{

public:
    typedef typename PackedGraphImpl<Vtype,Etype>::EdgeDescriptor   EdgeDescriptor;
    typedef typename PackedGraphImpl<Vtype,Etype>::SizeType         SizeType;
    
    PGEdge( unsigned int init = std::numeric_limits<SizeType>::max()): GraphElement< Etype, EdgeDescriptor>(),
                        m_adjacentNode(std::numeric_limits<SizeType>::max()), 
                        m_InEdge(std::numeric_limits<SizeType>::max())
    {
    }
   
    PGEdge( const unsigned int& adjacentNode , EdgeDescriptor descriptor):
            GraphElement< Etype, EdgeDescriptor>(descriptor),
			m_adjacentNode(adjacentNode),
			m_InEdge(std::numeric_limits<SizeType>::max())
    {
    }

    static unsigned int memUsage() 
    {
        return sizeof(PGEdge);
    }


    friend std::ostream& operator << ( std::ostream& out, PGEdge other)
	{
        if( other.m_adjacentNode != std::numeric_limits<SizeType>::max())
        {
            out << "{" << other.m_adjacentNode << "|";
        }
        else
        {
            out << "{-|";
        }
        /*if( other.m_InEdge != std::numeric_limits<SizeType>::max())
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

    SizeType    m_adjacentNode;
    SizeType    m_InEdge;   
};


template<typename Vtype, typename Etype>
class PGInEdge : public Etype
{

public:
    typedef typename PackedGraphImpl<Vtype,Etype>::SizeType         SizeType;
   
    PGInEdge( unsigned int init = std::numeric_limits<SizeType>::max()): Etype(),
                        m_adjacentNode(init), 
                        m_edge(std::numeric_limits<SizeType>::max())
    {
    }

    static unsigned int memUsage() 
    {
        return sizeof(PGInEdge);
    }

    bool operator ==( const PGInEdge& other) const
	{
        return ( m_edge == other.m_edge) && (m_adjacentNode == other.m_adjacentNode);
	}
	
	bool operator !=( const PGInEdge& other) const
	{
		return (m_edge != other.m_edge) || (m_adjacentNode != other.m_adjacentNode);
	}

    friend std::ostream& operator << ( std::ostream& out, PGInEdge other)
	{
        if( other.m_adjacentNode != std::numeric_limits<SizeType>::max())
        {
            out << "{" << other.m_adjacentNode << "|";
        }
        else
        {
            out << "{-|";
        }
        /*if( other.m_edge != std::numeric_limits<SizeType>::max())
        {   
            out << other.m_edge << "|";
        }
        else
        {
            out << "-|";
        }*/
        //out << "}|" << other.m_descriptor;
        out << "}";
		return out;
	}

    SizeType    m_adjacentNode;
    SizeType    m_edge;   
};


template<typename Vtype, typename Etype>
class PGNode : public GraphElement< Vtype, typename PackedGraphImpl<Vtype,Etype>::NodeDescriptor>
{
public:
    typedef typename PackedGraphImpl<Vtype,Etype>::NodeDescriptor NodeDescriptor;
    typedef typename PackedGraphImpl<Vtype,Etype>::SizeType       SizeType;

    PGNode( unsigned int init = 0):GraphElement< Vtype, NodeDescriptor>(), 
                                    m_firstEdge(std::numeric_limits<SizeType>::max()), 
                                    m_lastEdge(std::numeric_limits<SizeType>::max()),
                                    m_firstInEdge(std::numeric_limits<SizeType>::max()),
                                    m_lastInEdge(std::numeric_limits<SizeType>::max())
    {
    }

    PGNode( NodeDescriptor descriptor):GraphElement< Vtype, NodeDescriptor>(descriptor), 
                                    m_firstEdge(std::numeric_limits<SizeType>::max()), 
                                    m_lastEdge(std::numeric_limits<SizeType>::max()),
                                    m_firstInEdge(std::numeric_limits<SizeType>::max()),
                                    m_lastInEdge(std::numeric_limits<SizeType>::max())
    {
    }
    
    bool hasEdges() const
    {
        return m_firstEdge != std::numeric_limits<SizeType>::max();
    }

    bool hasInEdges() const
    {
        return m_firstInEdge != std::numeric_limits<SizeType>::max();
    }

    static unsigned int memUsage() 
    {
        return sizeof(PGNode);
    }

    friend std::ostream& operator << ( std::ostream& out, PGNode other)
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


    SizeType    m_firstEdge;
    SizeType    m_lastEdge;
    SizeType    m_firstInEdge;
    SizeType    m_lastInEdge;
};


template<typename Vtype, typename Etype>
class PGNodeObserver : public PackedMemoryArray< PGNode<Vtype,Etype> >::Observer
{
public:

    typedef typename PackedGraphImpl<Vtype,Etype>::EdgeIterator   EdgeIterator;
    typedef typename PackedGraphImpl<Vtype,Etype>::InEdgeIterator   InEdgeIterator;
    typedef typename PackedGraphImpl<Vtype,Etype>::NodeIterator   NodeIterator;
    typedef typename PackedGraphImpl<Vtype,Etype>::SizeType       SizeType;

    PGNodeObserver( PackedGraphImpl<Vtype,Etype>* G): m_G(G)
    {
    }

    void move( PGNode<Vtype,Etype>* source, PGNode<Vtype,Etype>* sourcePool, PGNode<Vtype,Etype>* destination, PGNode<Vtype,Etype>* destinationPool, const PGNode<Vtype,Etype>& node)
    {
        if( source - sourcePool == destination - destinationPool) return;
        assert( node != m_G->m_nodes.getEmptyElement());
       
        if( node.hasEdges())
        {
            EdgeIterator e = m_G->getEdgeIteratorAtIndex( node.m_firstEdge);
            EdgeIterator endEdges = m_G->getEdgeIteratorAtIndex( node.m_lastEdge);
            while ( e != endEdges)
            {
                m_G->getInEdgeIteratorAtIndex(e->m_InEdge)->m_adjacentNode = destination - destinationPool;
                ++e;
            }
        }

        if( node.hasInEdges())
        {
            InEdgeIterator k = m_G->getInEdgeIteratorAtIndex( node.m_firstInEdge);
            InEdgeIterator endInEdges = m_G->getInEdgeIteratorAtIndex( node.m_lastInEdge);
            while ( k != endInEdges)
            {
                m_G->getEdgeIteratorAtIndex(k->m_edge)->m_adjacentNode = destination - destinationPool;
                ++k;
            }
        }

        //std::cout << "Moving " << source << " to " << destination << std::endl;
		*(node.getDescriptor()) = destination - destinationPool;        
    }
private:
    PackedGraphImpl<Vtype,Etype>* m_G;
    EdgeIterator                        e, end;
    InEdgeIterator                      back_e, back_end;
};


template<typename Vtype, typename Etype>
class PGEdgeObserver : public PackedMemoryArray< PGEdge<Vtype,Etype> >::Observer
{
public:
    typedef typename PackedGraphImpl<Vtype,Etype>::SizeType       SizeType;
    typedef typename PackedGraphImpl<Vtype,Etype>::NodeIterator   NodeIterator;
    typedef typename PackedGraphImpl<Vtype,Etype>::InEdgeIterator   InEdgeIterator;

    PGEdgeObserver( PackedGraphImpl<Vtype,Etype>* G): m_G(G),lastChangedNode(std::numeric_limits<SizeType>::max())
    {
    }

    void move( PGEdge<Vtype,Etype>* source, PGEdge<Vtype,Etype>* sourcePool, PGEdge<Vtype,Etype>* destination, PGEdge<Vtype,Etype>* destinationPool, const PGEdge<Vtype,Etype>& edge)
    {
        if( source - sourcePool == destination - destinationPool) return;
        assert( edge != m_G->m_edges.getEmptyElement());

        if( edge.m_InEdge == std::numeric_limits<SizeType>::max()) return;
        *(edge.getDescriptor()) = destination - destinationPool;        
        
        m_G->getInEdgeIteratorAtIndex(edge.m_InEdge)->m_edge = destination - destinationPool;

        SizeType adjacentNode = m_G->getInEdgeIteratorAtIndex(edge.m_InEdge)->m_adjacentNode;
        SizeType firstEdge = m_G->getNodeIteratorAtIndex(adjacentNode)->m_firstEdge;

        if( ( firstEdge == (unsigned int)(source - sourcePool)) && ( adjacentNode != lastChangedNode))
        {
            NodeIterator u = m_G->getNodeIteratorAtIndex(adjacentNode);
            m_G->setFirstEdge( u, destination - destinationPool);   
            lastChangedNode = adjacentNode;
        }
        
    }

    void reset()
    {
        lastChangedNode = std::numeric_limits<SizeType>::max();
    }
private:
    PackedGraphImpl<Vtype,Etype>* m_G;
    SizeType  lastChangedNode;
};


template<typename Vtype, typename Etype>
class PGInEdgeObserver : public PackedMemoryArray< PGInEdge<Vtype,Etype> >::Observer
{
public:
    typedef typename PackedGraphImpl<Vtype,Etype>::SizeType       SizeType;
    typedef typename PackedGraphImpl<Vtype,Etype>::NodeIterator   NodeIterator;
    typedef typename PackedGraphImpl<Vtype,Etype>::EdgeIterator   EdgeIterator;

    PGInEdgeObserver( PackedGraphImpl<Vtype,Etype>* G): m_G(G), lastChangedNode(std::numeric_limits<SizeType>::max())
    {
    }

    void move( PGInEdge<Vtype,Etype>* source, PGInEdge<Vtype,Etype>* sourcePool, PGInEdge<Vtype,Etype>* destination, PGInEdge<Vtype,Etype>* destinationPool, const PGInEdge<Vtype,Etype>& InEdge)
    {
        if( source - sourcePool == destination - destinationPool ) return;

        assert( InEdge != m_G->m_inEdges.getEmptyElement());

        if( InEdge.m_edge == std::numeric_limits<SizeType>::max()) return;

        m_G->getEdgeIteratorAtIndex(InEdge.m_edge)->m_InEdge = destination - destinationPool;
        SizeType adjacentNode = m_G->getEdgeIteratorAtIndex(InEdge.m_edge)->m_adjacentNode;
        SizeType firstInEdge = m_G->getNodeIteratorAtIndex(adjacentNode)->m_firstInEdge;

        if( ( firstInEdge == (unsigned int)(source - sourcePool)) && ( adjacentNode != lastChangedNode))
        {
            NodeIterator u = m_G->getNodeIteratorAtIndex(adjacentNode);
            m_G->setFirstInEdge( u, destination - destinationPool);
            lastChangedNode = adjacentNode;
        }
    }

    void reset()
    {
        lastChangedNode = std::numeric_limits<SizeType>::max();
    }
private:
    PackedGraphImpl<Vtype,Etype>* m_G;
    SizeType  lastChangedNode;
};


#endif //PackedGraphImpl_H
