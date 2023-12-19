#ifndef FORWARDSTARIMPL_H
#define FORWARDSTARIMPL_H

#include <Utilities/mersenneTwister.h>
#include <vector>

template<typename Vtype, typename Etype>
class FSNode;
template<typename Vtype, typename Etype>
class FSEdge;
template<typename Vtype, typename Etype>
class FSInEdge;
template<typename Vtype, typename Etype>
class FSNodeObserver;
template<typename Vtype, typename Etype>
class FSEdgeObserver;
template<typename Vtype, typename Etype>
class FSInEdgeObserver;

template<typename Vtype, typename Etype>
class ForwardStarImpl
{

    friend class FSNodeObserver< Vtype, Etype>;
    friend class FSEdgeObserver< Vtype, Etype>;
    friend class FSInEdgeObserver< Vtype, Etype>;

public:
    
    typedef unsigned int                                                        SizeType;
    typedef FSNode<Vtype,Etype>**                                              NodeDescriptor;
    typedef FSEdge<Vtype,Etype>**                                              EdgeDescriptor;
    typedef typename PackedMemoryArray< FSNode< Vtype, Etype> >::Iterator      NodeIterator;
    typedef typename PackedMemoryArray< FSEdge< Vtype, Etype> >::Iterator      EdgeIterator;
    typedef typename PackedMemoryArray< FSInEdge< Vtype, Etype> >::Iterator    InEdgeIterator;
    

    ForwardStarImpl()
    {
        m_nodeObserver = new FSNodeObserver< Vtype, Etype>(this);
        m_nodes.registerObserver( m_nodeObserver);
        
        m_edgeObserver = new FSEdgeObserver< Vtype, Etype>(this);
        m_edges.registerObserver( m_edgeObserver);       
   
        m_InEdgeObserver = new FSInEdgeObserver< Vtype, Etype>(this);
        m_inEdges.registerObserver( m_InEdgeObserver);
    }

    ~ForwardStarImpl() 
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
        return getEdgeIteratorAtAddress(u->m_firstEdge);
    }
    
    InEdgeIterator beginInEdges( const NodeIterator& u)  
    { 
        return getInEdgeIteratorAtAddress( u->m_firstInEdge);
    }
    
    NodeIterator beginNodes()
    { 
        return m_nodes.begin();
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
        return getEdgeIteratorAtAddress( u->m_lastEdge);
    }
       
    InEdgeIterator endInEdges( const NodeIterator& u)  
    { 
        return getInEdgeIteratorAtAddress( u->m_lastInEdge);
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

        if( v->m_firstInEdge == k.getAddress() )
		{
		    NodeIterator w,z;
			InEdgeIterator f = k;
		    ++f;
			if( f.getAddress() == v->m_lastInEdge)
            {
                setFirstInEdge( v, 0);
            }
            else
            {
                setFirstInEdge( v, f.getAddress());
            }
		}

        e->m_InEdge = 0;
        m_edges.shiftErase( e);
        k->m_edge = 0;
        m_inEdges.erase( k);  
        delete descriptor; 
    }

    void eraseNode( NodeDescriptor uD) 
    { 
        NodeIterator u = getNodeIterator( uD);
        std::vector< EdgeDescriptor> edges;
        //uD = 0;
        //u->setDescriptor(0);
        m_nodes.shiftErase(u);
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
        return m_nodes.atAddress( e->m_adjacentNode);
    }
    
    NodeIterator getAdjacentNodeIterator( const InEdgeIterator& k)
    {
        return m_nodes.atAddress( k->m_adjacentNode);
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
        return m_edges.atAddress(*descriptor);
    }
    
    EdgeIterator getEdgeIterator( const InEdgeIterator& k) 
    {
        return m_edges.atAddress(k->m_edge);
    }
    
    EdgeIterator getEdgeIteratorAtAddress( FSEdge<Vtype,Etype>* addr)
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
        return m_inEdges.atAddress( e->m_InEdge);
    }
    
    InEdgeIterator getInEdgeIteratorAtAddress( FSInEdge<Vtype,Etype>* addr)
    {
        return m_inEdges.atAddress(addr);
    }
    
    InEdgeIterator getInEdgeIteratorAtIndex( const SizeType& position)
    {
        return m_inEdges.atIndex(position);
    } 

    NodeIterator getNodeIterator( const NodeDescriptor& descriptor) 
    { 
        return m_nodes.atAddress(*descriptor);
    }

    NodeIterator getNodeIterator( const SizeType& position) 
    { 
        return m_nodes.atIndex(position);
    }

    NodeIterator getNodeIteratorAtAddress( FSNode<Vtype,Etype>* addr) 
    { 
        return m_nodes.atAddress(addr);
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
        if( w != m_nodes.end()) e = getEdgeIteratorAtAddress(w->m_firstEdge);
        else                    e = m_edges.end();

        w = findNextNodeWithInEdges(v);
        if( w != m_nodes.end()) k = getInEdgeIteratorAtAddress(w->m_firstInEdge);
        else                    k = m_inEdges.end();
            
        EdgeDescriptor m_auxEdgeDescriptor = new FSEdge<Vtype,Etype>*();      
        //std::cout << "Malloced\t" << m_auxEdgeDescriptor << std::endl;  
        *m_auxEdgeDescriptor = 0;

        FSEdge<Vtype, Etype> newEdge( v.getAddress(), m_auxEdgeDescriptor);
        FSInEdge<Vtype, Etype> newInEdge( u.getAddress());
        e = m_edges.shiftInsert( e, newEdge);
        k = m_inEdges.shiftInsert( k, newInEdge);        
        *m_auxEdgeDescriptor = e.getAddress();
        e->m_InEdge = k.getAddress();
        k->m_edge = e.getAddress();  

        assert( k->m_adjacentNode != 0);

        if( !u->hasEdges())
        {
            setFirstEdge( u, e.getAddress());
            w = findNextNodeWithEdges(u);
            if( w != m_nodes.end())
            {
                u->m_lastEdge = w->m_firstEdge;
            }
        } 
        
        if( !v->hasInEdges())
        {
            setFirstInEdge( v, k.getAddress());
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

    void setFirstEdge( NodeIterator u, FSEdge<Vtype,Etype>* address)
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
    
    void setFirstInEdge( NodeIterator u, FSInEdge<Vtype,Etype>* address)
    {
        u->m_firstInEdge = address;
        if( address)
        {
            u = findPreviousNodeWithInEdges(u);
            if( u != m_nodes.end())
            {
                u->m_lastInEdge = address;
            }
        }
        else
        {
            u->m_lastInEdge = 0;
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
                    u->m_lastInEdge = 0;
                }
            }
        }
    }

    NodeDescriptor insertNode() 
    { 
        NodeDescriptor m_auxNodeDescriptor = new FSNode<Vtype,Etype>*();
        *m_auxNodeDescriptor = 0;
        //std::cout << "\nMalloced for node" << m_auxNodeDescriptor << std::endl;
        FSNode<Vtype,Etype> newNode;
        newNode.setDescriptor( m_auxNodeDescriptor);
        NodeIterator m_auxNodeIterator = m_nodes.shiftInsert( newNode);
        //NodeIterator m_auxNodeIterator = m_nodes.insert( m_nodes.end(),newNode);
        //NodeIterator m_auxNodeIterator = m_nodes.insert( m_nodes.begin(),newNode);
        *m_auxNodeDescriptor = m_auxNodeIterator.getAddress();
        m_lastPushedNode = m_nodes.end();
        m_currentPushedNode = m_nodes.end();
        return m_auxNodeDescriptor;
    }

    NodeDescriptor insertNodeBefore( NodeDescriptor descriptor) 
    { 
        NodeDescriptor m_auxNodeDescriptor = new FSNode<Vtype,Etype>*();
        
        *m_auxNodeDescriptor = 0;
        FSNode<Vtype,Etype> newNode;
        newNode.setDescriptor( m_auxNodeDescriptor);


        NodeIterator m_auxNodeIterator = m_nodes.shiftInsert( getNodeIterator(descriptor), newNode);

        *m_auxNodeDescriptor = m_auxNodeIterator.getAddress();

        m_lastPushedNode = m_nodes.end();
        m_currentPushedNode = m_nodes.end();
        return m_auxNodeDescriptor;
    }
    
    bool isValid()
    {
        FSEdge<Vtype,Etype>* lastEdge = 0;
        FSInEdge<Vtype,Etype>* lastInEdge = 0;

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
        std::cout << "\tSize:\t\t" << FSNode<Vtype,Etype>::memUsage() << "\t" << FSEdge<Vtype,Etype>::memUsage() << "\t" << FSInEdge<Vtype,Etype>::memUsage() << std::endl;
        
        /*std::cout << "\tNodes:\t\t" << m_nodes.size() << "\tReserved:" << m_nodes.capacity() << "\n"; 
        std::cout << "\tEdges:\t\t" << m_edges.size() << "\tReserved:" << m_edges.capacity() << "\n"; 
        std::cout << "\tInEdges:\t" << m_inEdges.size() << "\tReserved:" << m_inEdges.capacity() << "\n"; 
        std::cout << "\tNode Size:\t" << FSNode<Vtype,Etype>::memUsage() << "bytes" << "\t";
        std::cout << "\tEdge Size:\t" << FSEdge<Vtype,Etype>::memUsage() << "bytes" << "\t";
        std::cout << "\tBWEdge Size:\t" << FSInEdge<Vtype,Etype>::memUsage() << "bytes" << std::endl;*/

        return  FSNode<Vtype,Etype>::memUsage() * m_nodes.capacity() + 
                FSEdge<Vtype,Etype>::memUsage() * m_edges.capacity() + 
                FSInEdge<Vtype,Etype>::memUsage() * m_inEdges.capacity();
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
            if( w != m_nodes.end()) k = getInEdgeIteratorAtAddress(w->m_firstInEdge);
            else                    k = m_inEdges.end();
            
        EdgeDescriptor m_auxEdgeDescriptor = new FSEdge<Vtype,Etype>*();        
        *m_auxEdgeDescriptor = 0;

        FSEdge<Vtype, Etype> newEdge( v.getAddress(), m_auxEdgeDescriptor);
        FSInEdge<Vtype, Etype> newInEdge( u.getAddress());

        //std::cout << "\nMalloced for edge" << m_auxEdgeDescriptor << std::endl;
        //newEdge.setDescriptor( m_auxEdgeDescriptor);
        m_edges.push_back( newEdge);
        e = m_edges.end();
        --e;
        
        k = m_inEdges.insert( k, newInEdge);        
        *m_auxEdgeDescriptor = e.getAddress();

        //e->m_adjacentNode = v.getPoolIndex();
        e->m_InEdge = k.getAddress();
        //e->setDescriptor(m_auxEdgeDescriptor);

        //k->m_adjacentNode = u.getPoolIndex();
        k->m_edge = e.getAddress();  
        //k->setDescriptor(m_auxEdgeDescriptor);

        assert( k->m_adjacentNode != 0);

        bool uHadEdges = true;
        bool vHadInEdges = true;

        if( !u->hasEdges())
        {
            uHadEdges = false;
            u->m_firstEdge = e.getAddress();
        }
        
        if( !v->hasInEdges())
        {
            vHadInEdges = false;
            v->m_firstInEdge = k.getAddress();
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
        
        if( !vHadInEdges)
        {
            w = findPreviousNodeWithInEdges(v);
            assert( w != v);
            if( w != m_nodes.end())
            {
                w->m_lastInEdge = k.getAddress();
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
        *descriptor = u.getAddress();
    }

    void sanitizeDescriptor( const EdgeIterator& e)
    {
        EdgeDescriptor descriptor = getDescriptor(e);
        *descriptor = e.getAddress();
    }

    void setDescriptor( NodeIterator& u, NodeDescriptor& uD)
    {
        u->setDescriptor( uD);
        *uD = u.getAddress();
    }

    void setDescriptor( EdgeIterator& e, EdgeDescriptor& eD)
    {
        e->setDescriptor( eD);
        *eD = e.getAddress();
    }

private:
    PackedMemoryArray< FSNode< Vtype, Etype> >     m_nodes;
    PackedMemoryArray< FSEdge< Vtype, Etype> >     m_edges; 
    PackedMemoryArray< FSInEdge< Vtype, Etype> >   m_inEdges;
    FSNodeObserver< Vtype, Etype>*                 m_nodeObserver;
    FSEdgeObserver< Vtype, Etype>*                 m_edgeObserver;
    FSInEdgeObserver< Vtype, Etype>*               m_InEdgeObserver;

    NodeIterator m_lastPushedNode, m_currentPushedNode;

    MersenneTwister m_random;
};

template<typename Vtype, typename Etype>
class FSEdge : public GraphElement< Etype, typename ForwardStarImpl<Vtype,Etype>::EdgeDescriptor>
{

public:
    typedef typename ForwardStarImpl<Vtype,Etype>::EdgeDescriptor   EdgeDescriptor;
    //typedef typename ForwardStarImpl<Vtype,Etype>::NodeDescriptor   NodeDescriptor;
    typedef typename ForwardStarImpl<Vtype,Etype>::SizeType         SizeType;
    
    FSEdge( unsigned int init = 0): GraphElement< Etype, EdgeDescriptor>(),
                        m_adjacentNode(0), 
                        m_InEdge(0)
    {
    }
   
    FSEdge( FSNode<Vtype,Etype>* adjacentNode , EdgeDescriptor descriptor):
            GraphElement< Etype, EdgeDescriptor>(descriptor),
			m_adjacentNode(adjacentNode),
			m_InEdge(0)
    {
    }

    static unsigned int memUsage() 
    {
        return sizeof(FSEdge);
    }


    friend std::ostream& operator << ( std::ostream& out, FSEdge other)
	{
        if( other.m_adjacentNode != 0)
        {
            out << "{" << other.m_adjacentNode << "|";
        }
        else
        {
            out << "{-|";
        }
        if( other.m_InEdge != 0)
        {   
            out << other.m_InEdge << "|";
        }
        else
        {
            out << "-|";
        }
        //out << "}|" << other.m_descriptor << "}";
        out << "}";
		return out;
	}

    FSNode<Vtype,Etype>*       m_adjacentNode;
    FSInEdge<Vtype,Etype>*     m_InEdge;   
};


template<typename Vtype, typename Etype>
class FSInEdge : public Etype
{

public:
    typedef typename ForwardStarImpl<Vtype,Etype>::EdgeDescriptor   EdgeDescriptor;
    typedef typename ForwardStarImpl<Vtype,Etype>::NodeDescriptor   NodeDescriptor;
    typedef typename ForwardStarImpl<Vtype,Etype>::SizeType         SizeType;
   
    FSInEdge( unsigned int init = 0): Etype(),
                        m_adjacentNode(0), 
                        m_edge(0)
    {
    }
   
    FSInEdge( FSNode<Vtype,Etype>* adjacentNode):
            Etype(),
			m_adjacentNode(adjacentNode),
			m_edge(0)
    {
    }

    static unsigned int memUsage() 
    {
        return sizeof(FSInEdge);
    }

    bool operator ==( const FSInEdge& other) const
	{
        return ( m_edge == other.m_edge) && (m_adjacentNode == other.m_adjacentNode);
	}
	
	bool operator !=( const FSInEdge& other) const
	{
		return (m_edge != other.m_edge) || (m_adjacentNode != other.m_adjacentNode);
	}

    friend std::ostream& operator << ( std::ostream& out, FSInEdge other)
	{
        if( other.m_adjacentNode != 0)
        {
            out << "{" << other.m_adjacentNode << "|";
        }
        else
        {
            out << "{-|";
        }
        if( other.m_edge != 0)
        {   
            out << other.m_edge << "|";
        }
        else
        {
            out << "-|";
        }
        //out << "}|" << other.m_descriptor;
        out << "}";
		return out;
	}

    FSNode<Vtype,Etype>*    m_adjacentNode;
    FSEdge<Vtype,Etype>*    m_edge;   
};


template<typename Vtype, typename Etype>
class FSNode : public GraphElement< Vtype, typename ForwardStarImpl<Vtype,Etype>::NodeDescriptor>
{
public:
    typedef typename ForwardStarImpl<Vtype,Etype>::NodeDescriptor NodeDescriptor;
    typedef typename ForwardStarImpl<Vtype,Etype>::SizeType       SizeType;

    FSNode( unsigned int init = 0):GraphElement< Vtype, NodeDescriptor>(), 
                                    m_firstEdge(0), 
                                    m_lastEdge(0),
                                    m_firstInEdge(0),
                                    m_lastInEdge(0)
    {
    }

    FSNode( NodeDescriptor descriptor):GraphElement< Vtype, NodeDescriptor>(descriptor), 
                                    m_firstEdge(0), 
                                    m_lastEdge(0),
                                    m_firstInEdge(0),
                                    m_lastInEdge(0)
    {
    }
    
    bool hasEdges() const
    {
        return m_firstEdge != 0;
    }

    bool hasInEdges() const
    {
        return m_firstInEdge != 0;
    }

    static unsigned int memUsage() 
    {
        return sizeof(FSNode);
    }

    friend std::ostream& operator << ( std::ostream& out, FSNode other)
	{
        if( other.hasEdges())
        {
            out << "{" << other.m_firstEdge << "|" << other.m_lastEdge << "}|";
        }
        else
        {
            out << "{-|-}|";
        }
        if( other.hasInEdges())
        {   
            out << "{" << other.m_firstInEdge << "|" << other.m_lastInEdge << "}";
        }
        else
        {
            out << "{-|-}";
        }
        //out << "}";
        //out << "}|" << other.m_descriptor << "}";
		return out;
	}


    FSEdge<Vtype,Etype>*     m_firstEdge;
    FSEdge<Vtype,Etype>*     m_lastEdge;
    FSInEdge<Vtype,Etype>*   m_firstInEdge;
    FSInEdge<Vtype,Etype>*   m_lastInEdge;
};


template<typename Vtype, typename Etype>
class FSNodeObserver : public PackedMemoryArray< FSNode<Vtype,Etype> >::Observer
{
public:

    typedef typename ForwardStarImpl<Vtype,Etype>::EdgeIterator   EdgeIterator;
    typedef typename ForwardStarImpl<Vtype,Etype>::InEdgeIterator   InEdgeIterator;
    typedef typename ForwardStarImpl<Vtype,Etype>::NodeIterator   NodeIterator;
    typedef typename ForwardStarImpl<Vtype,Etype>::SizeType       SizeType;

    FSNodeObserver( ForwardStarImpl<Vtype,Etype>* G): m_G(G)
    {
    }

    void move( FSNode<Vtype,Etype>* source, FSNode<Vtype,Etype>* sourcePool, FSNode<Vtype,Etype>* destination, FSNode<Vtype,Etype>* destinationPool, const FSNode<Vtype,Etype>& node)
    {
        assert( node != m_G->m_nodes.getEmptyElement());
       
        if( node.hasEdges())
        {
            EdgeIterator e = m_G->getEdgeIteratorAtAddress( node.m_firstEdge);
            EdgeIterator endEdges = m_G->getEdgeIteratorAtAddress( node.m_lastEdge);
            while ( e != endEdges)
            {
                e->m_InEdge->m_adjacentNode = destination;
                ++e;
            }
        }

        if( node.hasInEdges())
        {
            InEdgeIterator k = m_G->getInEdgeIteratorAtAddress( node.m_firstInEdge);
            InEdgeIterator endInEdges = m_G->getInEdgeIteratorAtAddress( node.m_lastInEdge);
            while ( k != endInEdges)
            {
                k->m_edge->m_adjacentNode = destination;
                ++k;
            }
        }

        //std::cout << "Moving " << source << " to " << destination << std::endl;
		*(node.getDescriptor()) = destination;        
    }
private:
    ForwardStarImpl<Vtype,Etype>* m_G;
    EdgeIterator                        e, end;
    InEdgeIterator                      back_e, back_end;
};


template<typename Vtype, typename Etype>
class FSEdgeObserver : public PackedMemoryArray< FSEdge<Vtype,Etype> >::Observer
{
public:
    typedef typename ForwardStarImpl<Vtype,Etype>::SizeType       SizeType;
    typedef typename ForwardStarImpl<Vtype,Etype>::NodeIterator   NodeIterator;
    typedef typename ForwardStarImpl<Vtype,Etype>::InEdgeIterator   InEdgeIterator;

    FSEdgeObserver( ForwardStarImpl<Vtype,Etype>* G): m_G(G),lastChangedNode(0),lastChangedTailNode(0)
    {
    }

    void move( FSEdge<Vtype,Etype>* source, FSEdge<Vtype,Etype>* sourcePool, FSEdge<Vtype,Etype>* destination, FSEdge<Vtype,Etype>* destinationPool, const FSEdge<Vtype,Etype>& edge)
    {
        if( source == destination) return;
        assert( edge != m_G->m_edges.getEmptyElement());

        if( !(edge.m_InEdge)) return;
        *(edge.getDescriptor()) = destination;        
        
        edge.m_InEdge->m_edge = destination;

        /*if( source == (FSEdge<Vtype,Etype>*)0x139c910)
        {
            std::cout << "Hi!\n";
        }*/

        if( (edge.m_InEdge->m_adjacentNode->m_firstEdge == source) 
            && (edge.m_InEdge->m_adjacentNode != lastChangedNode))
        {
            NodeIterator u = m_G->getNodeIteratorAtAddress(edge.m_InEdge->m_adjacentNode);
            m_G->setFirstEdge( u, destination);   
            lastChangedNode = edge.m_InEdge->m_adjacentNode;
        }
        
    }

    void reset()
    {
        lastChangedNode = 0;
        lastChangedTailNode = 0;
    }
private:
    ForwardStarImpl<Vtype,Etype>* m_G;
    FSNode<Vtype,Etype>*  lastChangedNode;
    FSNode<Vtype,Etype>*  lastChangedTailNode;
};


template<typename Vtype, typename Etype>
class FSInEdgeObserver : public PackedMemoryArray< FSInEdge<Vtype,Etype> >::Observer
{
public:
    typedef typename ForwardStarImpl<Vtype,Etype>::SizeType       SizeType;
    typedef typename ForwardStarImpl<Vtype,Etype>::NodeIterator   NodeIterator;
    typedef typename ForwardStarImpl<Vtype,Etype>::EdgeIterator   EdgeIterator;

    FSInEdgeObserver( ForwardStarImpl<Vtype,Etype>* G): m_G(G), lastChangedNode(0), lastChangedTailNode(0)
    {
    }

    void move( FSInEdge<Vtype,Etype>* source, FSInEdge<Vtype,Etype>* sourcePool, FSInEdge<Vtype,Etype>* destination, FSInEdge<Vtype,Etype>* destinationPool, const FSInEdge<Vtype,Etype>& InEdge)
    {
        if( source == destination) return;

        assert( InEdge != m_G->m_inEdges.getEmptyElement());

        /*if( source == (FSInEdge<Vtype,Etype>*)0x1735318)
        {
            std::cout << "Hi!\n";
        }
        
        if( source == (FSInEdge<Vtype,Etype>*)0x1735324)
        {
            std::cout << "Hi!\n";
        }*/

        if( !(InEdge.m_edge)) return;

        InEdge.m_edge->m_InEdge = destination;

        FSNode<Vtype,Etype>* adjacentNode = InEdge.m_edge->m_adjacentNode;
        FSInEdge<Vtype,Etype>* firstInEdge = adjacentNode->m_firstInEdge;

        if( ( firstInEdge == source) && ( adjacentNode != lastChangedNode))
        {
            NodeIterator u = m_G->getNodeIteratorAtAddress(adjacentNode);
            m_G->setFirstInEdge( u, destination);
            lastChangedNode = adjacentNode;
        }
    }

    void reset()
    {
        lastChangedNode = 0;
        lastChangedTailNode = 0;
    }
private:
    ForwardStarImpl<Vtype,Etype>* m_G;
    FSNode<Vtype,Etype>*  lastChangedNode;
    FSNode<Vtype,Etype>*  lastChangedTailNode;
};


#endif //FORWARDSTARIMPL_H
