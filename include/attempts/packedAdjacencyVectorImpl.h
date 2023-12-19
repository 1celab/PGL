#ifndef PACKEDADJACENCYVECTORIMPL_H
#define PACKEDADJACENCYVECTORIMPL_H

#include <Utilities/mersenneTwister.h>
#include <vector>

template<typename Vtype, typename Etype>
class PAVNode;
template<typename Vtype, typename Etype>
class PAVEdge;

template<typename Vtype, typename Etype>
class PackedAdjacencyVectorImpl
{
public:
    
    typedef unsigned int                                                SizeType;
    typedef typename std::vector< PAVNode< Vtype, Etype> >::iterator    NodeIterator;
    typedef typename std::vector< PAVEdge< Vtype, Etype> >::iterator    EdgeIterator;
    typedef SizeType*                                                   NodeDescriptor;
    typedef SizeType*                                                   EdgeDescriptor;

    AdjacencyVectorImpl()
    {
        jsw_seed(1);
    }

    ~AdjacencyVectorImpl() 
    {
        NodeIterator end = m_nodes.end();
        EdgeIterator e, end_edges;

        for( NodeIterator u = m_nodes.begin(); u != end; ++u)
        {
            delete u->m_descriptor;
            end_edges = u->m_edges.end();
            for( e = u->m_edges.begin(); e != end_edges; ++e)
            {
                delete e->m_descriptor;
            }
        }
    }

    const NodeDescriptor& insertNode() 
    { 
        m_auxNodeDescriptor = new SizeType();
        *m_auxNodeDescriptor = m_nodes.size();        

        PAVNode<Vtype,Etype> newNode;
        m_nodes.push_back( newNode);
        NodeIterator u = m_nodes.end() - 1;
        u->setDescriptor(m_auxNodeDescriptor);
        return m_auxNodeDescriptor;
    }
    
    const EdgeDescriptor& insertEdge( const NodeDescriptor& uD, const NodeDescriptor& vD) 
    { 
        NodeIterator u = getNodeIterator( uD);
        NodeIterator v = getNodeIterator( vD);
        
        PAVEdge<Vtype, Etype> newEdge;
        
        newEdge.m_adjacentNode = v - m_nodes.begin();
        EdgeIterator e = endEdges(u);
        e = m_edges.insert( e, newEdge);
        
        newEdge.m_adjacentNode = u - m_nodes.begin();
        EdgeIterator k = endBackEdges(v);
        k = m_backEdges.insert( e, newEdge);
        
        EdgeIterator e = u->m_edges.end() - 1;
        EdgeIterator k = v->m_backEdges.end() - 1;
        
        e->m_oppositeEdge = k - endBackEdges(v);
        k->m_oppositeEdge = e - endEdges(u);

        e->m_isBackEdge = false;
        k->m_isBackEdge = true;

        m_auxEdgeDescriptor = new SizeType( e - m_edges.begin());
        e->setDescriptor(m_auxEdgeDescriptor);
        k->setDescriptor(m_auxEdgeDescriptor);
        
        return m_auxEdgeDescriptor;
    }

    void eraseNode( NodeDescriptor descriptor) 
    { 
        NodeIterator u = getNodeIterator( descriptor);
        EdgeIterator e, k, end;
        
        end = endEdges(u);
        for( EdgeIterator e = beginEdges(u); e!= end; ++e)
        {
            eraseEdge( (*e).id);
        }
       
        end = endBackEdges(u); 
        for( EdgeIterator e = beginBackEdges(u); e!= end; ++e)
        {
            k = getOppositeEdgeIterator( e);
            eraseEdge( (*k).id);
        }

        delete descriptor;

        NodeIterator v;
        NodeIterator end_nodes = m_nodes.end();
        for( v = m_nodes.erase(u); v != end_nodes; ++v)
        {
            sanitiseNode( v);
        }
    }
    
    void eraseEdge( EdgeDescriptor descriptor) 
    { 
        EdgeIterator e = getEdgeIterator(descriptor);
        NodeIterator v = m_nodes.begin()+ e->m_adjacentNode;
        EdgeIterator k = v->m_backEdges.begin() + e->m_oppositeEdge;
        NodeIterator u = m_nodes.begin()+ k->m_adjacentNode;
        
        EdgeIterator f,h,end;
        end = (*v).m_backEdges.end();
        for( f = (*v).m_backEdges.erase(k); f != end; ++f)
        {
            --(f->getDescriptor()->second);
            h = m_nodes[ f->m_adjacentNode].m_edges.begin() + f->m_oppositeEdge;
            --(h->m_adjacentNode);
        }

        end = (*u).m_edges.end();
        for( f = (*u).m_edges.erase(e); f != end; ++f)
        {
            --(f->getDescriptor()->second);
            h = m_nodes[ f->m_adjacentNode].m_backEdges.begin() + f->m_oppositeEdge;
            --(h->m_adjacentNode);
        }
    
        delete descriptor;
    }

    void sanitiseNode( const NodeIterator& u)
    {
        SizeType position =*(u->getDescriptor());
        --position;
        *(u->getDescriptor()) = position;

        EdgeIterator end,k;
        NodeIterator adjacentNode;
        SizeType adjacentNodePosition;        

        end = u->m_edges.end();        
        for( EdgeIterator e = u->m_edges.begin(); e != end; ++e)
        {
            e->getDescriptor()->first = position;
            if( e->m_adjacentNode < position) continue;
            
            --(e->m_adjacentNode);
            k = m_nodes[ e->m_adjacentNode].m_backEdges.begin() + e->m_oppositeEdge;
            k.m_adjacentNode = position;
        }

        end = u->m_backEdges.end();        
        for( EdgeIterator e = u->m_backEdges.begin(); e != end; ++e)
        {
            e->getDescriptor()->first = position;
            if( e->m_adjacentNode < position) continue; 

            --(e->m_adjacentNode);
            k = m_nodes[ e->m_adjacentNode].m_edges.begin() + e->m_oppositeEdge;
            k.m_adjacentNode = position;
        }
    }    

    const NodeIterator& beginNodes()
    { 
        m_auxNodeIterator = m_nodes.begin();
        return m_auxNodeIterator;
    }
    
    const NodeIterator& endNodes()
    { 
        m_auxNodeIterator = m_nodes.end();
        return m_auxNodeIterator;
    }

    const EdgeIterator& beginEdges( NodeIterator u)  
    { 
        m_auxEdgeIterator = u->m_edges.begin();
        return m_auxEdgeIterator;
    }
    
    const EdgeIterator& endEdges( NodeIterator u)  
    { 
        m_auxEdgeIterator = u->m_edges.end();
        return m_auxEdgeIterator;
    }
    
    const EdgeIterator& beginBackEdges( NodeIterator u)  
    { 
        m_auxEdgeIterator = u->m_backEdges.begin();
        return m_auxEdgeIterator;
    }
        
    const EdgeIterator& endBackEdges( NodeIterator u)  
    { 
        m_auxEdgeIterator = u->m_backEdges.end();
        return m_auxEdgeIterator;
    }
    
	const NodeIterator& chooseNode()  
	{ 
	    double random = ((double)jsw_rand()/std::numeric_limits<SizeType>::max());
        SizeType pos = m_nodes.size() * random;
        
        m_auxNodeIterator = m_nodes.begin() + pos;
        return m_auxNodeIterator;
	}
	
    const EdgeIterator& getEdgeIterator( const EdgeDescriptor& descriptor) 
    { 
        m_auxEdgeIterator = (m_nodes.begin() + descriptor->first)->m_edges.begin() + descriptor->second;
        return m_auxEdgeIterator;
    }
    
    const NodeIterator& getNodeIterator( const NodeDescriptor& descriptor) 
    { 
        m_auxNodeIterator = m_nodes.begin()+(*descriptor);
        return m_auxNodeIterator;
    }

    const NodeIterator& getOppositeNodeIterator( const EdgeIterator& e)
    {
        m_auxNodeIterator = m_nodes.begin();
        return m_auxNodeIterator;
    }

    const NodeIterator& nilNode()
    {
        m_auxNodeIterator = m_nodes.end();
        return m_auxNodeIterator;
    }   
    
    const EdgeIterator& nilEdge() 
    { 
        m_auxEdgeIterator = m_edges.end();
        return m_auxEdgeIterator;
    }
    
private:
    std::vector< PAVNode< Vtype, Etype> >   m_nodes;
    std::vector< PAVEdge< Vtype, Etype> >   m_edges;
    std::vector< PAVEdge< Vtype, Etype> >   m_backEdges;

    NodeIterator    m_auxNodeIterator;
    EdgeIterator    m_auxEdgeIterator;
    NodeDescriptor  m_auxNodeDescriptor;
    EdgeDescriptor  m_auxEdgeDescriptor;
};

template<typename Vtype, typename Etype>
class PAVEdge : public GraphElement< Etype, typename PackedAdjacencyVectorImpl<Vtype,Etype>::EdgeDescriptor>
{

public:
    typedef typename AdjacencyVectorImpl<Vtype,Etype>::EdgeDescriptor               EdgeDescriptor;
    typedef typename AdjacencyVectorImpl<Vtype,Etype>::NodeDescriptor               NodeDescriptor;
    typedef typename AdjacencyVectorImpl<Vtype,Etype>::SizeType                     SizeType; 
    
    PAVEdge( unsigned int init = 0): GraphElement< Etype, EdgeDescriptor>()
    {
    }
   
    PAVEdge( NodeDescriptor adjacentNode , EdgeDescriptor oppositeEdge , NodeDescriptor descriptor = 0, Etype data = Etype()):
            GraphElement< Etype, EdgeDescriptor>(descriptor),
			m_adjacentNode(adjacentNode),
			m_oppositeEdge(oppositeEdge)
    {
    }
    
    SizeType                    m_adjacentNode;
    SizeType                    m_oppositeEdge;
    bool                        m_isBackEdge;    
};

template<typename Vtype, typename Etype>
class PAVNode : public GraphElement< Vtype, typename PackedAdjacencyVectorImpl<Vtype,Etype>::NodeDescriptor>
{
public:
    typedef typename AdjacencyVectorImpl<Vtype,Etype>::NodeDescriptor   NodeDescriptor;

    PAVNode():GraphElement< Vtype, NodeDescriptor>()
    {
    }

    PAVNode( NodeDescriptor descriptor):GraphElement< Vtype, NodeDescriptor>(descriptor)
    {
    }

    SizeType                    m_firstEdge;
    SizeType                    m_firstBackEdge;   
};



#endif //PACKEDADJACENCYVECTORIMPL_H
