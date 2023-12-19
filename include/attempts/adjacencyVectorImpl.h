#ifndef ADJACENCYVECTORIMPL_H
#define ADJACENCYVECTORIMPL_H

#include <Utilities/mersenneTwister.h>
#include <vector>

template<typename Vtype, typename Etype>
class AVNode;
template<typename Vtype, typename Etype>
class AVEdge;

template<typename Vtype, typename Etype>
class AdjacencyVectorImpl
{
public:
    
    typedef unsigned int                                            SizeType;
    typedef typename std::vector< AVNode< Vtype, Etype> >::iterator NodeIterator;
    typedef typename std::vector< AVEdge< Vtype, Etype> >::iterator EdgeIterator;
    typedef SizeType*                                               NodeDescriptor;
    typedef typename std::pair<SizeType, SizeType>*                 EdgeDescriptor;

    AdjacencyVectorImpl()
    {
    }

    ~AdjacencyVectorImpl() 
    {
        NodeIterator end = m_nodes.end();
        EdgeIterator e, end_edges;

        for( NodeIterator u = m_nodes.begin(); u != end; ++u)
        {
            delete u->getDescriptor();
            end_edges = u->m_edges.end();
            for( e = u->m_edges.begin(); e != end_edges; ++e)
            {
                delete e->getDescriptor();
            }
        }
    }

    const NodeDescriptor& insertNode() 
    { 
        m_auxNodeDescriptor = new SizeType( m_nodes.size());
        //*m_auxNodeDescriptor = m_nodes.size();
        AVNode<Vtype,Etype> newNode;
        m_nodes.push_back( newNode);
        m_auxNodeIterator = m_nodes.end() - 1;
        m_auxNodeIterator->setDescriptor(m_auxNodeDescriptor);
        return m_auxNodeDescriptor;
    }
    
    const EdgeDescriptor& insertEdge( const NodeDescriptor& uD, const NodeDescriptor& vD) 
    { 
        NodeIterator u = getNodeIterator( uD);
        NodeIterator v = getNodeIterator( vD);
        
        AVEdge<Vtype, Etype> newEdge;
        newEdge.m_adjacentNode = v - m_nodes.begin();
        (*u).m_edges.push_back( newEdge);
        newEdge.m_adjacentNode = u - m_nodes.begin();
        (*v).m_backEdges.push_back( newEdge);
        
        EdgeIterator e = u->m_edges.end() - 1;
        EdgeIterator k = v->m_backEdges.end() - 1;
        
        e->m_oppositeEdge = k - v->m_backEdges.begin();
        k->m_oppositeEdge = e - u->m_edges.begin();
        e->m_isBackEdge = false;
        k->m_isBackEdge = true;

        m_auxEdgeDescriptor = new std::pair<SizeType,SizeType>( u - m_nodes.begin(), e - u->m_edges.begin());
        e->setDescriptor(m_auxEdgeDescriptor);
        k->setDescriptor(m_auxEdgeDescriptor);
        
        return m_auxEdgeDescriptor;
    }

    void eraseNode( NodeDescriptor& descriptor) 
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
            k = getOppositeIterator( e);
            eraseEdge( (*k).id);
        }

        delete descriptor;
        descriptor = 0;

        NodeIterator v;
        NodeIterator end_nodes = m_nodes.end();
        for( v = m_nodes.erase(u); v != end_nodes; ++v)
        {
            sanitiseNode( v);
        }
    }
    
    void eraseEdge( EdgeDescriptor& descriptor) 
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
            h = getOppositeEdgeIterator( f);
            --(h->m_adjacentNode);
        }

        end = (*u).m_edges.end();
        for( f = (*u).m_edges.erase(e); f != end; ++f)
        {
            --(f->getDescriptor()->second);
            h = getOppositeEdgeIterator( f);
            --(h->m_adjacentNode);
        }
    
        delete descriptor;
        descriptor = 0;
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
            k = getOppositeEdgeIterator( e);
            k.m_adjacentNode = position;
        }

        end = u->m_backEdges.end();        
        for( EdgeIterator e = u->m_backEdges.begin(); e != end; ++e)
        {
            e->getDescriptor()->first = position;
            if( e->m_adjacentNode < position) continue; 

            --(e->m_adjacentNode);
            k = getOppositeEdgeIterator( e);
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

    const EdgeIterator& beginEdges( const NodeIterator& u)  
    { 
        m_auxEdgeIterator = u->m_edges.begin();
        return m_auxEdgeIterator;
    }
    
    const EdgeIterator& endEdges( const NodeIterator& u)  
    { 
        m_auxEdgeIterator = u->m_edges.end();
        return m_auxEdgeIterator;
    }
    
    const EdgeIterator& beginBackEdges( const NodeIterator& u)  
    { 
        m_auxEdgeIterator = u->m_backEdges.begin();
        return m_auxEdgeIterator;
    }
        
    const EdgeIterator& endBackEdges( const NodeIterator& u)  
    { 
        m_auxEdgeIterator = u->m_backEdges.end();
        return m_auxEdgeIterator;
    }
    
	const NodeIterator& chooseNode()  
	{ 
	    double random = m_random.getRandomNormalizedDouble();
        SizeType pos = m_nodes.size() * random;
        
        m_auxNodeIterator = m_nodes.begin() + pos;
        return m_auxNodeIterator;
	}
	
	/*const EdgeDescriptor& getDescriptor( const EdgeIterator& e) const
    {
        return e;
    }
    
    const NodeDescriptor& getDescriptor( const NodeIterator& u) const
    {
        return u;
    }*/
	
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

    const NodeIterator& getAdjacentNodeIterator( const EdgeIterator& e)
    {
        m_auxNodeIterator = m_nodes.begin() + e->m_adjacentNode;
        return m_auxNodeIterator;
    }

    const EdgeIterator& getOppositeEdgeIterator( const EdgeIterator& e)
    {
        if( !isBackEdge(e))
        {
            m_auxEdgeIterator = m_nodes[ e->m_adjacentNode].m_backEdges.begin() + e->m_oppositeEdge;
            return m_auxEdgeIterator; 
        }
        m_auxEdgeIterator = m_nodes[ e->m_adjacentNode].m_edges.begin() + e->m_oppositeEdge;
        return m_auxEdgeIterator;
    }

    const bool& isBackEdge( const EdgeIterator& e) const
    {
        return e->m_isBackEdge;
    }

    bool edgeExists( const EdgeDescriptor& descriptor)
    {
        return descriptor != 0;
    }
  
    bool nodeExists( const NodeDescriptor& descriptor)
    {
        return descriptor != 0;
    }

    const NodeIterator& nilNode()
    {
        m_auxNodeIterator = m_nodes.end();
        return m_auxNodeIterator;
    }   
    
    const EdgeIterator& nilEdge() 
    { 
        m_auxEdgeIterator = (*m_nodes.begin()).m_edges.end();
        return m_auxEdgeIterator;
    }
    
private:
    std::vector< AVNode< Vtype, Etype> > m_nodes;
    NodeIterator    m_auxNodeIterator;
    EdgeIterator    m_auxEdgeIterator;
    NodeDescriptor  m_auxNodeDescriptor;
    EdgeDescriptor  m_auxEdgeDescriptor;
    MersenneTwister m_random;
};

template<typename Vtype, typename Etype>
class AVEdge : public GraphElement< Etype, typename AdjacencyVectorImpl<Vtype,Etype>::EdgeDescriptor>
{

public:
    typedef typename AdjacencyVectorImpl<Vtype,Etype>::EdgeDescriptor   EdgeDescriptor;
    typedef typename AdjacencyVectorImpl<Vtype,Etype>::NodeDescriptor   NodeDescriptor;
    typedef typename AdjacencyVectorImpl<Vtype,Etype>::SizeType         SizeType;
    
    AVEdge( unsigned int init = 0): GraphElement< Etype, EdgeDescriptor>()
    {
    }
   
    AVEdge( NodeDescriptor adjacentNode , EdgeDescriptor oppositeEdge , NodeDescriptor descriptor = 0, Etype data = Etype()):
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
class AVNode : public GraphElement< Vtype, typename AdjacencyVectorImpl<Vtype,Etype>::NodeDescriptor>
{
public:
    typedef typename AdjacencyVectorImpl<Vtype,Etype>::NodeDescriptor   NodeDescriptor;

    AVNode():GraphElement< Vtype, NodeDescriptor>()
    {
    }

    AVNode( NodeDescriptor descriptor):GraphElement< Vtype, NodeDescriptor>(descriptor)
    {
    }
    
    std::vector< AVEdge< Vtype, Etype> >    m_edges;
    std::vector< AVEdge< Vtype, Etype> >    m_backEdges;
};



#endif //ADJACENCYVECTORIMPL_H
