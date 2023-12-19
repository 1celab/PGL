#ifndef PACKEDMEMORYARRAYIMPL_H
#define PACKEDMEMORYARRAYIMPL_H

#include <Utilities/mersenneTwister.h>
#include <vector>

template<typename Vtype, typename Etype, typename InEtype>
class PMANode;
template<typename Vtype, typename Etype, typename InEtype>
class PMAEdge;
template<typename Vtype, typename Etype, typename InEtype>
class PMAInEdge;
template<typename Vtype, typename Etype, typename InEtype>
class PMANodeObserver;
template<typename Vtype, typename Etype, typename InEtype>
class PMAEdgeObserver;
template<typename Vtype, typename Etype, typename InEtype>
class PMAInEdgeObserver;


template<typename Vtype, typename Etype, typename InEtype>
class EdgeBucket
{
    friend class PMANodeObserver<Vtype,Etype,InEtype>;
    friend class PMAEdgeObserver<Vtype,Etype,InEtype>;
    friend class PMAInEdgeObserver<Vtype,Etype,InEtype>;
};



template<typename Vtype, typename Etype, typename InEtype>
class PackedMemoryArrayImpl
{

    friend class PMANodeObserver<Vtype,Etype,InEtype>;
    friend class PMAEdgeObserver<Vtype,Etype,InEtype>;
    friend class PMAInEdgeObserver<Vtype,Etype,InEtype>;

public:

    typedef unsigned int                                                              SizeType;
    typedef PMANode<Vtype,Etype,InEtype>**                                            NodeDescriptor;
    typedef typename PackedMemoryArray< PMANode<Vtype,Etype,InEtype> >::Iterator      NodeIterator;
    typedef typename PackedMemoryArray< PMAEdge<Vtype,Etype,InEtype> >::Iterator      EdgeIterator;
    typedef typename PackedMemoryArray< PMAInEdge<Vtype,Etype,InEtype> >::Iterator    InEdgeIterator;


    PackedMemoryArrayImpl()
    {
        m_nodeObserver = new PMANodeObserver<Vtype,Etype,InEtype>(this);
        m_nodes.registerObserver( m_nodeObserver);

        m_edgeObserver = new PMAEdgeObserver<Vtype,Etype,InEtype>(this);
        m_edges.registerObserver( m_edgeObserver);

        m_InEdgeObserver = new PMAInEdgeObserver<Vtype,Etype,InEtype>(this);
        m_inEdges.registerObserver( m_InEdgeObserver);
    }

    ~PackedMemoryArrayImpl()
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
        m_edges.erase( e);
        k->m_edge = 0;
        m_inEdges.erase( k);
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

    EdgeIterator getEdgeIterator( const InEdgeIterator& k)
    {
        return m_edges.atAddress(k->m_edge);
    }

    EdgeIterator getEdgeIteratorAtAddress( PMAEdge<Vtype,Etype,InEtype>* addr)
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

    InEdgeIterator getInEdgeIteratorAtAddress( PMAInEdge<Vtype,Etype,InEtype>* addr)
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

    NodeIterator getNodeIteratorAtAddress( PMANode<Vtype,Etype,InEtype>* addr)
    {
        return m_nodes.atAddress(addr);
    }

    void insertEdge( NodeDescriptor uD, NodeDescriptor vD)
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

        PMAEdge<Vtype,Etype,InEtype> newEdge( v.getAddress());
        PMAInEdge<Vtype,Etype,InEtype> newInEdge( u.getAddress());
        e = m_edges.insert( e, newEdge);
        k = m_inEdges.insert( k, newInEdge);

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

    }

    void changeHead( EdgeIterator& e, NodeIterator& v)
    {
        /*NodeIterator x = getAdjacentNodeIterator( e);
        InEdgeIterator k = getInEdgeIterator( e);
        NodeIterator u = getAdjacentNodeIterator( k);

        NodeDescriptor uD = getDescriptor( u);
        NodeDescriptor vD = getDescriptor( v);
        NodeDescriptor xD = getDescriptor( x);

        eraseEdge( uD, xD);
        insertEdge( uD, vD);*/

        //e:(u->x)
        NodeIterator x = getAdjacentNodeIterator( e);

        if( x == v)
            return;

        InEdgeIterator k = getInEdgeIterator( e);
        NodeIterator u = getAdjacentNodeIterator( k);

        /*std::cout << "\nBEFORE";
        std::cout << "\nu:" << u->label << u->time << " o" << outdeg( u) << " i" << indeg(u );
        std::cout << "\ne:" << e->weight;
        std::cout << "\nx:" << x->label << x->time << " o" << outdeg( x) << " i" << indeg(x );
        std::cout << "\nv:" << v->label << v->time << " o" << outdeg( v) << " i" << indeg(v ) << std::endl;*/

        //erase inEdge (u->x)
        if( x->m_firstInEdge == k.getAddress())
		{
			InEdgeIterator f = k;
		    ++f;
			if( f.getAddress() == x->m_lastInEdge)
            {
                setFirstInEdge( x, 0);
                //std::cout << "x will have no inedges !" << std::endl;
            }

            else
            {
                setFirstInEdge( x, f.getAddress());
                //std::cout << "x reset first inedge" << std::endl;
            }
		}

        k->m_edge = 0;
        m_inEdges.erase( k);

        assert( u != v);
        assert( (!v->hasInEdges()) || (v->m_firstInEdge != v->m_lastInEdge));

        //insert inEdge
        NodeIterator w = findNextNodeWithInEdges( v);
        if( w != m_nodes.end()) k = getInEdgeIteratorAtAddress( w->m_firstInEdge);
        else                    k = m_inEdges.end();

        PMAInEdge<Vtype,Etype,InEtype> newInEdge( u.getAddress());
        k = m_inEdges.insert( k, newInEdge);

        //update Edge
        e->m_adjacentNode = v.getAddress();
        e->m_InEdge = k.getAddress();

        k->m_adjacentNode = u.getAddress();
        k->m_edge = e.getAddress();

        //std::cout << "PHASE2\n" << std::endl;

        if( !v->hasInEdges())
        {
            //std::cout << "v has not in-edges";
            setFirstInEdge( v, k.getAddress());
            w = findNextNodeWithInEdges(v);
            if( w != m_nodes.end())
            {
                v->m_lastInEdge = w->m_firstInEdge;
            }
        }
        //else
        //    std::cout << "v has in-edges";

        /*std::cout << "\nAFTER";
        std::cout << "\nu:" << u->label << u->time << " o" << outdeg( u) << " i" << indeg(u );
        std::cout << "\nx:" << x->label << x->time << " o" << outdeg( x) << " i" << indeg(x );
        std::cout << "\nv:" << v->label << v->time << " o" << outdeg( v) << " i" << indeg(v ) << std::endl;*/

        assert( (u->m_lastEdge > u->m_firstEdge) || (!(u->m_lastEdge)));
        assert( (v->m_lastInEdge > v->m_firstInEdge) || (!(v->m_lastInEdge)));
        assert( u->hasEdges() && v->hasInEdges());
    }

    void swapEdges( EdgeIterator& e1, EdgeIterator& e2)
    {
        InEdgeIterator k1 = getInEdgeIterator( e1);
        InEdgeIterator k2 = getInEdgeIterator( e2);

        NodeIterator v1 = getAdjacentNodeIterator( e1);
        NodeIterator v2 = getAdjacentNodeIterator( e2);

        NodeIterator u1 = getAdjacentNodeIterator( k1);
        NodeIterator u2 = getAdjacentNodeIterator( k2);
        assert( u1 == u2);

        //std::swap( e1->m_adjacentNode, e2->m_adjacentNode);
        //e1->m_adjacentNode = v2.getAddress();
        //e2->m_adjacentNode = v1.getAddress();

        //std::swap( e1->m_InEdge, e2->m_InEdge);
        //e1->m_InEdge = k2.getAddress();
        //e2->m_InEdge = k1.getAddress();

        //std::swap( k1->m_edge, k2->m_edge);
        //k1->m_edge = e2.getAddress();
        //k2->m_edge = e1.getAddress();

        /*EdgeIterator etemp = e1;
        e1 = e2;
        e2 = etemp;*/

        EdgeIterator pme1 = m_edges.atAddress( e1.getAddress());
        EdgeIterator pme2 = m_edges.atAddress( e2.getAddress());
        InEdgeIterator pmk1 = m_inEdges.atAddress( k1.getAddress());
        InEdgeIterator pmk2 = m_inEdges.atAddress( k2.getAddress());

        std::swap( pme1->m_adjacentNode, pme2->m_adjacentNode);
        //std::swap( pme1->m_InEdge, pme2->m_InEdge);
        //std::swap( pmk1->m_edge, pmk2->m_edge);
        //std::swap( pmk1->m_adjacentNode, pmk1->m_adjacentNode);

        std::swap( e1, e2);

        /*EdgeIterator etemp;
        *etemp = *e1;
        *e1 = *e2;
        *e2 = *etemp;*/
    }

	SizeType indeg( const NodeIterator& u)
    {
        //beg new indeg code: added by andreas 9/12/2014
        SizeType distance = 0;
        const InEdgeIterator end = endInEdges(u);
        InEdgeIterator beg = beginInEdges(u);

        while( beg != end)
        {
             beg++;
             distance++;
        }

        return distance;
        //end new indeg

        //old outdeg code: return endInEdges(u) - beginInEdges(u); original code by Panos
    }

	SizeType outdeg( const NodeIterator& u)
    {
        //beg new outdeg code: added by andreas 9/12/2014
        SizeType distance = 0;
        const EdgeIterator end = endEdges(u);
        EdgeIterator beg = beginEdges(u);

        while( beg != end)
        {
             beg++;
             distance++;
        }

        return distance;
        //end new outdeg

        //old outdeg code: return endEdges(u) - beginEdges(u); original code by Panos
    }

    void setFirstEdge( NodeIterator u, PMAEdge<Vtype,Etype,InEtype>* address)
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

    void setFirstInEdge( NodeIterator u, PMAInEdge<Vtype,Etype,InEtype>* address)
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
        NodeDescriptor m_auxNodeDescriptor = new PMANode<Vtype,Etype,InEtype>*();
        *m_auxNodeDescriptor = 0;
        //std::cout << "\nMalloced for node" << m_auxNodeDescriptor << std::endl;
        PMANode<Vtype,Etype,InEtype> newNode;
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
        NodeDescriptor m_auxNodeDescriptor = new PMANode<Vtype,Etype,InEtype>*();

        *m_auxNodeDescriptor = 0;
        PMANode<Vtype,Etype,InEtype> newNode;
        newNode.setDescriptor( m_auxNodeDescriptor);


        NodeIterator m_auxNodeIterator = m_nodes.insert( getNodeIterator(descriptor), newNode);

        *m_auxNodeDescriptor = m_auxNodeIterator.getAddress();

        m_lastPushedNode = m_nodes.end();
        m_currentPushedNode = m_nodes.end();
        return m_auxNodeDescriptor;
    }

    bool isValid()
    {
        PMAEdge<Vtype,Etype,InEtype>* lastEdge = 0;
        PMAInEdge<Vtype,Etype,InEtype>* lastInEdge = 0;

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
        std::cout << "\tSize:\t\t" << PMANode<Vtype,Etype,InEtype>::memUsage() << "\t" << PMAEdge<Vtype,Etype,InEtype>::memUsage() << "\t" << PMAInEdge<Vtype,Etype,InEtype>::memUsage() << std::endl;

        m_nodes.getMemoryUsage();

        /*std::cout << "\tNodes:\t\t" << m_nodes.size() << "\tReserved:" << m_nodes.capacity() << "\n";
        std::cout << "\tEdges:\t\t" << m_edges.size() << "\tReserved:" << m_edges.capacity() << "\n";
        std::cout << "\tInEdges:\t" << m_inEdges.size() << "\tReserved:" << m_inEdges.capacity() << "\n";
        std::cout << "\tNode Size:\t" << PMANode<Vtype,Etype,InEtype>::memUsage() << "bytes" << "\t";
        std::cout << "\tEdge Size:\t" << PMAEdge<Vtype,Etype,InEtype>::memUsage() << "bytes" << "\t";
        std::cout << "\tBWEdge Size:\t" << PMAInEdge<Vtype,Etype,InEtype>::memUsage() << "bytes" << std::endl;*/

        return  PMANode<Vtype,Etype,InEtype>::memUsage() * m_nodes.capacity() +
                PMAEdge<Vtype,Etype,InEtype>::memUsage() * m_edges.capacity() +
                PMAInEdge<Vtype,Etype,InEtype>::memUsage() * m_inEdges.capacity();
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
        assert( (!v->hasInEdges()) || (v->m_firstInEdge != v->m_lastInEdge));

        NodeIterator w;
        EdgeIterator e;
        InEdgeIterator k;

        w = v;
        ++w;
            w = findNextNodeWithInEdges(w);
            if( w != m_nodes.end()) k = getInEdgeIteratorAtAddress(w->m_firstInEdge);
            else                    k = m_inEdges.end();

        PMAEdge<Vtype,Etype,InEtype> newEdge( v.getAddress());
        PMAInEdge<Vtype,Etype,InEtype> newInEdge( u.getAddress());

        m_edges.push_back( newEdge);
        e = m_edges.end();
        --e;

        k = m_inEdges.insert( k, newInEdge);

        //e->m_adjacentNode = v.getPoolIndex();
        e->m_InEdge = k.getAddress();

        //k->m_adjacentNode = u.getPoolIndex();
        k->m_edge = e.getAddress();

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


    void setDescriptor( NodeIterator& u, NodeDescriptor& uD)
    {
        u->setDescriptor( uD);
        *uD = u.getAddress();
    }


private:
    PackedMemoryArray< PMANode< Vtype, Etype, InEtype> >     m_nodes;
    PackedMemoryArray< PMAEdge< Vtype, Etype, InEtype> >     m_edges;
    PackedMemoryArray< PMAInEdge< Vtype, Etype, InEtype> >   m_inEdges;
    PMANodeObserver< Vtype, Etype, InEtype>*                 m_nodeObserver;
    PMAEdgeObserver< Vtype, Etype, InEtype>*                 m_edgeObserver;
    PMAInEdgeObserver< Vtype, Etype, InEtype>*               m_InEdgeObserver;

    NodeIterator m_lastPushedNode, m_currentPushedNode;

    MersenneTwister m_random;
};

template<typename Vtype, typename Etype, typename InEtype>
class PMAEdge : public Etype
{

public:
    //typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::NodeDescriptor   NodeDescriptor;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::SizeType         SizeType;

    PMAEdge( unsigned int init = 0): Etype(),
                        m_adjacentNode(0),
                        m_InEdge(0)
    {
    }

    PMAEdge( PMANode<Vtype,Etype,InEtype>* adjacentNode):
            Etype(),
			m_adjacentNode(adjacentNode),
			m_InEdge(0)
    {
    }

    static unsigned int memUsage()
    {
        return sizeof(PMAEdge);
    }


    bool operator ==( const PMAEdge& other) const
	{
        return ( m_InEdge == other.m_InEdge) && (m_adjacentNode == other.m_adjacentNode);
	}

	bool operator !=( const PMAEdge& other) const
	{
		return (m_InEdge != other.m_InEdge) || (m_adjacentNode != other.m_adjacentNode);
	}


    friend std::ostream& operator << ( std::ostream& out, PMAEdge other)
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

    PMANode<Vtype,Etype,InEtype>*       m_adjacentNode;
    PMAInEdge<Vtype,Etype,InEtype>*     m_InEdge;
};


template<typename Vtype, typename Etype, typename InEtype>
class PMAInEdge : public InEtype
{

public:
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::NodeDescriptor   NodeDescriptor;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::SizeType         SizeType;

    PMAInEdge( unsigned int init = 0): InEtype(),
                        m_adjacentNode(0),
                        m_edge(0)
    {
    }

    PMAInEdge( PMANode<Vtype,Etype,InEtype>* adjacentNode):
            InEtype(),
			m_adjacentNode(adjacentNode),
			m_edge(0)
    {
    }

    static unsigned int memUsage()
    {
        return sizeof(PMAInEdge);
    }

    bool operator ==( const PMAInEdge& other) const
	{
        return ( m_edge == other.m_edge) && (m_adjacentNode == other.m_adjacentNode);
	}

	bool operator !=( const PMAInEdge& other) const
	{
		return (m_edge != other.m_edge) || (m_adjacentNode != other.m_adjacentNode);
	}

    friend std::ostream& operator << ( std::ostream& out, PMAInEdge other)
	{
        if( other.m_adjacentNode != 0)
        {
            out << "{" << other.m_adjacentNode << "|";
        }
        else
        {
            out << "{-|";
        }
        /*if( other.m_edge != 0)
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

    PMANode<Vtype,Etype,InEtype>*    m_adjacentNode;
    PMAEdge<Vtype,Etype,InEtype>*    m_edge;
};


template<typename Vtype, typename Etype, typename InEtype>
class PMANode : public GraphElement< Vtype, typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::NodeDescriptor>
{
public:
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::NodeDescriptor NodeDescriptor;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::SizeType       SizeType;

    PMANode( unsigned int init = 0):GraphElement< Vtype, NodeDescriptor>(),
                                    m_firstEdge(0),
                                    m_lastEdge(0),
                                    m_firstInEdge(0),
                                    m_lastInEdge(0)
    {
    }

    PMANode( NodeDescriptor descriptor):GraphElement< Vtype, NodeDescriptor>(descriptor),
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
        return sizeof(PMANode);
    }

    friend std::ostream& operator << ( std::ostream& out, PMANode other)
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


    PMAEdge<Vtype,Etype,InEtype>*     m_firstEdge;
    PMAEdge<Vtype,Etype,InEtype>*     m_lastEdge;
	PMAInEdge<Vtype,Etype,InEtype>*   m_firstInEdge;
    PMAInEdge<Vtype,Etype,InEtype>*   m_lastInEdge;
};


template<typename Vtype, typename Etype, typename InEtype>
class PMANodeObserver : public PackedMemoryArray< PMANode<Vtype,Etype,InEtype> >::Observer
{
public:

    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::EdgeIterator     EdgeIterator;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::InEdgeIterator   InEdgeIterator;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::NodeIterator     NodeIterator;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::SizeType         SizeType;

    PMANodeObserver( PackedMemoryArrayImpl<Vtype,Etype,InEtype>* G): m_G(G)
    {
    }

    void move( PMANode<Vtype,Etype,InEtype>* source, PMANode<Vtype,Etype,InEtype>* sourcePool, PMANode<Vtype,Etype,InEtype>* destination, PMANode<Vtype,Etype,InEtype>* destinationPool, const PMANode<Vtype,Etype,InEtype>& node)
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
    PackedMemoryArrayImpl<Vtype,Etype,InEtype>* m_G;
    EdgeIterator                                e, end;
    InEdgeIterator                              back_e, back_end;
};


template<typename Vtype, typename Etype, typename InEtype>
class PMAEdgeObserver : public PackedMemoryArray< PMAEdge<Vtype,Etype,InEtype> >::Observer
{
public:
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::SizeType         SizeType;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::NodeIterator     NodeIterator;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::InEdgeIterator   InEdgeIterator;

    PMAEdgeObserver( PackedMemoryArrayImpl<Vtype,Etype,InEtype>* G): m_G(G),lastChangedNode(0),lastChangedTailNode(0)
    {
    }

    void move( PMAEdge<Vtype,Etype,InEtype>* source, PMAEdge<Vtype,Etype,InEtype>* sourcePool, PMAEdge<Vtype,Etype,InEtype>* destination, PMAEdge<Vtype,Etype,InEtype>* destinationPool, const PMAEdge<Vtype,Etype,InEtype>& edge)
    {
        if( source == destination) return;
        assert( edge != m_G->m_edges.getEmptyElement());

        if( !(edge.m_InEdge)) return;

        edge.m_InEdge->m_edge = destination;

        /*if( source == (PMAEdge<Vtype,Etype>*)0x139c910)
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
    PackedMemoryArrayImpl<Vtype,Etype,InEtype>* m_G;
    PMANode<Vtype,Etype,InEtype>*  lastChangedNode;
    PMANode<Vtype,Etype,InEtype>*  lastChangedTailNode;
};


template<typename Vtype, typename Etype, typename InEtype>
class PMAInEdgeObserver : public PackedMemoryArray< PMAInEdge<Vtype,Etype,InEtype> >::Observer
{
public:
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::SizeType       SizeType;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::NodeIterator   NodeIterator;
    typedef typename PackedMemoryArrayImpl<Vtype,Etype,InEtype>::EdgeIterator   EdgeIterator;

    PMAInEdgeObserver( PackedMemoryArrayImpl<Vtype,Etype,InEtype>* G): m_G(G), lastChangedNode(0), lastChangedTailNode(0)
    {
    }

    void move( PMAInEdge<Vtype,Etype,InEtype>* source, PMAInEdge<Vtype,Etype,InEtype>* sourcePool, PMAInEdge<Vtype,Etype,InEtype>* destination, PMAInEdge<Vtype,Etype,InEtype>* destinationPool, const PMAInEdge<Vtype,Etype,InEtype>& InEdge)
    {
        if( source == destination) return;

        assert( InEdge != m_G->m_inEdges.getEmptyElement());

        /*if( source == (PMAInEdge<Vtype,Etype>*)0x1735318)
        {
            std::cout << "Hi!\n";
        }

        if( source == (PMAInEdge<Vtype,Etype>*)0x1735324)
        {
            std::cout << "Hi!\n";
        }*/

        if( !(InEdge.m_edge)) return;

        InEdge.m_edge->m_InEdge = destination;

        PMANode<Vtype,Etype,InEtype>* adjacentNode = InEdge.m_edge->m_adjacentNode;
        PMAInEdge<Vtype,Etype,InEtype>* firstInEdge = adjacentNode->m_firstInEdge;

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
    PackedMemoryArrayImpl<Vtype,Etype,InEtype>* m_G;
    PMANode<Vtype,Etype,InEtype>*  lastChangedNode;
    PMANode<Vtype,Etype,InEtype>*  lastChangedTailNode;
};


#endif //PACKEDMEMORYARRAYIMPL_H
