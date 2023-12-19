/* Copyright (C) Andreas Paraskevopoulos 2019 - All Rights Reserved
 * Unauthorized modification or copying of this file, via any medium
 * is strictly prohibited. The licensing header must be part in all copies
 * or modifications of this file. Confidential and fair dealing.
 * Author and Developer: Andreas Paraskevopoulos
 * <email: paraskevop@ceid.upatras.gr, phone: 6947292750>.
 */

#ifndef STATIC_ARRAY_GRAPH_H
#define STATIC_ARRAY_GRAPH_H

#include <map>
#include <cstdint>
#include <iostream>

template<typename Vtype, typename Etype, typename InEtype>
class StaticArrayGraph
{

  public:

    typedef unsigned int     SizeType;
    typedef void*            NodeDescriptor;
    typedef void*            EdgeDescriptor;    

    class NodeIterator;
    class EdgeIterator;
    class InEdgeIterator;

    struct NodeData: Vtype
    {
        NodeData& operator=( const NodeData &other)
        {
            Vtype::operator=( other);
            numOutEdges = other.numOutEdges;
            numInEdges = other.numInEdges;
            return *this;
        }

        SizeType numOutEdges;
        SizeType numInEdges;
    };

    struct EdgeData: Etype
    {
        EdgeData& operator=( const EdgeData &other)
        {
            Etype::operator=( other);
            target = other.target;
            inEdge = other.inEdge;
            return *this;
        }
        
        NodeIterator target;
        InEdgeIterator inEdge;
    };
    
    struct InEdgeData: InEtype
    {
        InEdgeData& operator=( const InEdgeData &other)
        {
            InEtype::operator=( other);
            source = other.source;
            outEdge = other.outEdge;
            return *this;
        }
        
        NodeIterator source;
        EdgeIterator outEdge;
    };
    
    class NodeIterator
    {
      public:
    
        NodeIterator():
        data(NULL)
        {}

        NodeIterator( NodeData* ptr):
        data(ptr)
        {}

	NodeIterator( const NodeIterator& v):
        data(v.data)
	{}

        NodeData& operator*() const
        {
            return *data;
        }

        NodeData* operator->() const
        {
            return data;
        }

        NodeIterator& operator++()
        {
            char* ptr = (char *) data;
            ptr += ( sizeof(NodeData) + sizeof(EdgeData) * data->numOutEdges + sizeof(InEdgeData) * data->numInEdges);
            data = (NodeData*) ptr;
            return *this;
        }

        NodeIterator& operator=( const NodeIterator &other)
        {
            data = other.data;
            return *this;
        }
        
        bool operator ==( const NodeIterator& other) const
        {
            return ( data == other.data );
        }

        bool operator !=( const NodeIterator& other) const
        {
            return ( data != other.data);
        }
        
        EdgeIterator beginEdges() const
        {
            EdgeData* eData = reinterpret_cast<EdgeData*>( data+1);
            return EdgeIterator( eData);
        }

        EdgeIterator endEdges() const
        {
            EdgeData* eData = reinterpret_cast<EdgeData*>( data+1);
            eData += data->numOutEdges;
            return EdgeIterator( eData);
        }
        
        InEdgeIterator beginInEdges() const
        {
            EdgeData* eData = reinterpret_cast<EdgeData*>( data+1);
            eData += data->numOutEdges;
            InEdgeData* kData = reinterpret_cast<InEdgeData*>( eData);
            return InEdgeIterator( kData);
        }

        InEdgeIterator endInEdges() const
        {
            EdgeData* eData = reinterpret_cast<EdgeData*>( data+1);
            eData += data->numOutEdges;
            InEdgeData* kData = reinterpret_cast<InEdgeData*>( eData);
            kData += data->numInEdges;
            return InEdgeIterator( kData);
        }
        
      private:
      
        NodeData* data;
    };

    class EdgeIterator
    {
      public:
      
        typedef int difference_type;
    
        EdgeIterator():
        data(NULL)
        {}
    
        EdgeIterator( EdgeData* ptr):
        data(ptr)
        {}
        
        EdgeIterator( const EdgeIterator& e):
        data(e.data)
        {}
        
        EdgeData& operator*()
        {
            return *data;
        }
        
        EdgeData* operator->() const
        {
            return data;
        }

        int operator-( const EdgeIterator& e) const
        { 
           return abs(data - e.data);
        }
        
        EdgeIterator operator+( int i) const
        {
           return EdgeIterator( data+i);
        }

        EdgeIterator& operator+=( int i)
        {
           data +=i;
           return *this;
        }
        
        EdgeIterator& operator++()
        {
           data++;
           return *this;
        }

        EdgeIterator operator++(int)
        {
           EdgeIterator e = *this;
           data++;
           return e;
        }

        EdgeIterator& operator--()
        {
           data--;
           return *this;
        }

        EdgeIterator operator--(int)
        {
           EdgeIterator e = *this;
           data--;
           return e;
        }

        EdgeIterator& operator=( const EdgeIterator &other)
        {
            data = other.data;
            return *this;
        }
        
        int operator-( const EdgeIterator &other)
        {
            return ( data - other.data);
        }

        bool operator==( const EdgeIterator& other) const
        {
            return ( data == other.data );
        }

        bool operator!=( const EdgeIterator& other) const
        {
            return ( data != other.data);
        }
        
      private:
      
        EdgeData* data;
    };
        
    class InEdgeIterator
    {
      public:
    
        InEdgeIterator():
        data(NULL)
        {}
    
        InEdgeIterator( InEdgeData* ptr):
        data(ptr)
        {}
        
        InEdgeIterator( const InEdgeIterator& k):
        data(k.data)
        {}

        InEdgeData& operator*()
        {
            return *data;
        }
        
        InEdgeData* operator->() const 
        {
            return data;
        }

        InEdgeIterator operator+( int i)
        {
           InEdgeIterator k = *this;
           k.data +=i;
           return k;
        }

        InEdgeIterator& operator++()
        {
           data +=1;
           return *this;
        }

        InEdgeIterator operator++(int)
        {
           InEdgeIterator k = *this;
           data +=1;
           return k;
        }

        InEdgeIterator& operator--()
        {
           data -=1;
           return *this;
        }


        InEdgeIterator operator--(int)
        {
           InEdgeIterator k = *this;
           data -=1;
           return k;
        }

        InEdgeIterator& operator=( const InEdgeIterator &other)
        {
            data = other.data;
            return *this;
        }
        
        bool operator ==( const InEdgeIterator& other) const
        {
            return ( data == other.data );
        }

        bool operator !=( const InEdgeIterator& other) const
        {
            return ( data != other.data);
        }
        
      private:
      
        InEdgeData* data;
    };
    
    StaticArrayGraph(): 
    m_numNodes(0), m_numEdges(0), m_pool(NULL), m_capacity(0)
    {}

    StaticArrayGraph( const uint32_t numNodes, const uint32_t numEdges, 
                      const std::vector<std::vector<uint32_t>>& outdegs, 
                      const std::vector<std::vector<uint32_t>>& indegs):
    m_numNodes(0), m_numEdges(0), m_pool(NULL), m_capacity(0)
    {
        build( numNodes, numEdges, outdegs, indegs);
    }

    template <typename GraphType>
    StaticArrayGraph( const GraphType& G): 
    m_numNodes(0), m_numEdges(0), m_pool(NULL), m_capacity(0)
    {
        load( G);
    }

    void build( const uint32_t numNodes, const uint32_t numEdges, 
                const std::vector<std::vector<uint32_t>>& outdegs, 
                const std::vector<std::vector<uint32_t>>& indegs)
    {
        clear();
        assert( hasInvalidInput( numNodes, numEdges, outdegs, indegs));

        m_numNodes = numNodes; 
        m_numEdges = numEdges;
        m_capacity = uint64_t(m_numNodes) * sizeof(NodeData) + uint64_t(m_numEdges) * ( sizeof(EdgeData) + sizeof(InEdgeData));
        m_pool = new char[m_capacity];
        if( m_pool == NULL)
        {
            std::cout << "Aborting... Cannot allocate" << m_capacity << " bytes\n";
            return;
        }

        std::memset( m_pool, 0, m_capacity);
        char* data;

        struct Node2NodeID
        {
            Node2NodeID( NodeIterator x, NodeIterator y): 
            u(x), v(y)
            {}
                        
            bool operator<( const Node2NodeID& other) const
            {
                if( u.operator->() < other.u.operator->())
                    return true;
                else if( u.operator->() == other.u.operator->())
                {
                    if(  v.operator->() < other.v.operator->())
                        return true;
                }
                return false;
            }
            
            NodeIterator u;
            NodeIterator v;   
        };

        std::vector<NodeIterator> nodes( numNodes);
        std::map<Node2NodeID, EdgeData*> edgeMap;
        std::map<Node2NodeID, InEdgeData*> inedgeMap;

        data = m_pool;
        for( int i=0; i<numNodes; i++)
        {
           NodeData* vData = (NodeData*) data;
           nodes[i] = NodeIterator(vData);
           vData->numOutEdges = outdegs[i].size();
           vData->numInEdges = indegs[i].size();
           data += ( sizeof(NodeData) + vData->numOutEdges * sizeof(EdgeData) + vData->numInEdges * sizeof(InEdgeData));
        }

        data = m_pool;
        for( int i=0; i<numNodes; i++)
        {
            data += sizeof(NodeData);
            NodeIterator u = nodes[i];

            for( int j=0; j<outdegs[i].size(); j++)
            {
                NodeIterator v = nodes[outdegs[i][j]];
                EdgeData* eData = (EdgeData*) data;
                EdgeIterator eo( eData);
                eo->target = v;
                Node2NodeID eId( u, v);
                edgeMap[eId] = eData;
                data += sizeof(EdgeData);
            }

            //  e
            //u--->v
            //  k
            //u--->v

            for( int j=0; j<indegs[i].size(); j++)
            {
                NodeIterator v = nodes[indegs[i][j]];
                InEdgeData* kData = (InEdgeData*) data;
                InEdgeIterator ko( kData);
                ko->source = v;
                Node2NodeID kId( v, u);
                inedgeMap[kId] = kData;
                data += sizeof(InEdgeData);
            }
        }

        for( NodeIterator u = beginNodes(), endNode = endNodes(); u != endNode; ++u)
        {
            for( EdgeIterator e = beginEdges(u), endEdge = endEdges(u); e != endEdge; ++e)
            {
                NodeIterator v = target( e);
                Node2NodeID kId( u, v);

                e->inEdge = InEdgeIterator( inedgeMap[kId]);
            }

            for( InEdgeIterator k = beginInEdges(u), endInEdge = endInEdges(u); k != endInEdge; ++k)
            {
                NodeIterator v = source( k);
                Node2NodeID eId( v, u);

                k->outEdge = EdgeIterator( edgeMap[eId]);
            }
        }
    }

    template <typename GraphType>
    void load( const GraphType& G)
    {
        clear();

        typedef typename GraphType::NodeIterator   InputNodeIterator;
        typedef typename GraphType::EdgeIterator   InputEdgeIterator;
        typedef typename GraphType::InEdgeIterator InputInEdgeIterator;

        m_numNodes = G.getNumNodes();
        m_numEdges = G.getNumEdges();
        m_capacity = uint64_t(m_numNodes)*sizeof(NodeData) + uint64_t(m_numEdges) * ( sizeof(EdgeData) + sizeof(InEdgeData));
        m_pool = new char[m_capacity];
        if( m_pool == NULL)
        {
            std::cout << "Exiting... Cannot allocate" << m_capacity << " bytes\n";
            exit(1);
        }

        std::memset( m_pool, 0, m_capacity);

        char* data;

        struct Node2NodeID
        {
            Node2NodeID( NodeIterator x, NodeIterator y): 
            u(x), v(y)
            {}
                        
            bool operator<( const Node2NodeID& other) const
            {
                if( u.operator->() < other.u.operator->())
                    return true;
                else if( u.operator->() == other.u.operator->())
                {
                    if(  v.operator->() < other.v.operator->())
                        return true;
                }
                return false;
            }
            
            NodeIterator u;
            NodeIterator v;   
        };

        std::map<InputNodeIterator, NodeIterator> nodeMap;
        std::map<Node2NodeID, EdgeData*> edgeMap;
        std::map<Node2NodeID, InEdgeData*> inedgeMap;

        data = m_pool;
        for( InputNodeIterator v = G.beginNodes(), endNode = G.endNodes(); v != endNode; ++v)
        {
           NodeData* vData = (NodeData*) data;
           nodeMap[v] = NodeIterator(vData);
           vData->numOutEdges = G.outdeg(v);
           vData->numInEdges = G.indeg(v);
           data += ( sizeof(NodeData) + vData->numOutEdges * sizeof(EdgeData) + vData->numInEdges * sizeof(InEdgeData));
        }

        data = m_pool;
        for( InputNodeIterator u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        {
            data += sizeof(NodeData);

            for( InputEdgeIterator e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
            {
                InputNodeIterator v = G.target( e);
                EdgeData* eData = (EdgeData*) data;
                EdgeIterator eo( eData);
                eo->target = nodeMap[v];
                Node2NodeID eId( nodeMap[u], nodeMap[v]);
                edgeMap[eId] = eData;
                data += sizeof(EdgeData);
            }

            // e
            //u--->v

            //    k
            //u--->v

            for( InputInEdgeIterator k = G.beginInEdges(u), endInEdge = G.endInEdges(u); k != endInEdge; ++k)
            {
                InputNodeIterator v = G.source( k);
                InEdgeData* kData = (InEdgeData*) data;
                InEdgeIterator ko( kData);
                ko->source = nodeMap[v];
                Node2NodeID kId( nodeMap[v], nodeMap[u]);
                inedgeMap[kId] = kData;
                data += sizeof(InEdgeData);
            }
        }

        for( NodeIterator u = beginNodes(), endNode = endNodes(); u != endNode; ++u)
        {
            for( EdgeIterator e = beginEdges(u), endEdge = endEdges(u); e != endEdge; ++e)
            {
                NodeIterator v = target( e);
                Node2NodeID kId( u, v);

                e->inEdge = InEdgeIterator( inedgeMap[kId]);
            }

            for( InEdgeIterator k = beginInEdges(u), endInEdge = endInEdges(u); k != endInEdge; ++k)
            {
                NodeIterator v = source( k);
                Node2NodeID eId( v, u);

                k->outEdge = EdgeIterator( edgeMap[eId]);
            }
        }
    }

    ~StaticArrayGraph()
    {
         clear();
    }

    SizeType getNumNodes() const
    {
        return m_numNodes;
    }

    SizeType getNumEdges() const
    {
        return m_numEdges;
    }

    EdgeIterator beginEdges( const NodeIterator& u) const
    {
        return u.beginEdges();
    }

    EdgeIterator endEdges( const NodeIterator& u) const
    {
        return u.endEdges();
    }
    
    InEdgeIterator beginInEdges( const NodeIterator& u)  const
    {
        return u.beginInEdges();
    }

    InEdgeIterator endInEdges( const NodeIterator& u) const
    {
        return u.endInEdges();
    }
    
    NodeIterator beginNodes() const
    {
        NodeData* vData = reinterpret_cast<NodeData*>(m_pool); 
        return NodeIterator(vData);
    }

    NodeIterator endNodes() const
    {
        NodeData* ptr = (NodeData*) (m_pool + m_capacity);        
        return NodeIterator( ptr);
    }

    bool hasNode( NodeDescriptor vD) const
    {
	if( ( vD >= ( (NodeData*) m_pool) && vD < ( (NodeData*) (m_pool + m_capacity))))
	    return true;
	return false;
    }

    void clear()
    {
        m_numNodes = 0;
        m_numEdges = 0;
        m_capacity = 0;

        if( m_pool != NULL)
        {
            delete [] m_pool;
            m_pool = NULL;
        }
    }

    void expand()
    {
    }

    NodeIterator source( const EdgeIterator& e) const
    {   
        InEdgeIterator k = getInEdgeIterator( e);
        return getAdjacentNodeIterator(k);
    }
    
    NodeIterator target( const EdgeIterator& e) const
    {
        return getAdjacentNodeIterator(e);
    }
    
    NodeIterator source( const InEdgeIterator& k) const
    {
        return getAdjacentNodeIterator(k);
    }
    
    NodeIterator target( const InEdgeIterator& k) const
    {  
        EdgeIterator e = getEdgeIterator( k);
        return getAdjacentNodeIterator(e);    
    }
    
    NodeIterator getAdjacentNodeIterator( const EdgeIterator& e) const
    {
        return e->target;
    }

    NodeIterator getAdjacentNodeIterator( const InEdgeIterator& k) const
    {
        return k->source;
    }

    void* getDescriptor( const NodeIterator& u) const
    {
        return u->data;
    }

    void* getDescriptor( const EdgeIterator& e) const
    {
        return e->data;
    }
    
    void* getDescriptor( const InEdgeIterator& k) const
    {
        return k->data;
    }
   
    NodeIterator getNodeIterator( const NodeDescriptor& vD) const
    {
        return NodeIterator( (NodeData*) vD); 
    }

    NodeDescriptor getNodeDescriptor( const NodeIterator& v) const
    {
        return ( (NodeDescriptor) &(*v)); 
    }

    NodeDescriptor nilNodeDescriptor() const
    {
        return NULL;
    }

    EdgeIterator getEdgeIterator( const NodeDescriptor& uD, const NodeDescriptor& vD) const
    {
        NodeIterator u = NodeIterator( uD);
        NodeIterator v = NodeIterator( vD);
        return getEdgeIterator( u, v);
    }

    EdgeIterator getEdgeIterator( const NodeIterator& u, const NodeIterator& v) const
    {
        NodeIterator neigh;
        EdgeIterator e = beginEdges(u), endEdge = endEdges(u);
        for(; e != endEdge; ++e)
        {
            neigh = getAdjacentNodeIterator(e);
            if( neigh == v)
                break;
        }
        
        return e;
    }

    InEdgeIterator getInEdgeIterator( const NodeDescriptor& uD, const NodeDescriptor& vD) const
    {
        NodeIterator u = NodeIterator( uD);
        NodeIterator v = NodeIterator( vD);
        return getInEdgeIterator(u, v);
    }

    InEdgeIterator getInEdgeIterator( const NodeIterator& u, const NodeIterator& v) const
    {
        NodeIterator neigh;
        InEdgeIterator k = beginInEdges(v), endInEdge = endInEdges(v);
        for(; k != endInEdge; ++k)
        {
            neigh = getAdjacentNodeIterator(k);
            if( neigh == u)
                break;
        }
        
        return k;
    }

    EdgeIterator getEdgeIterator( const InEdgeIterator& k) const
    {
        return EdgeIterator( k->outEdge);
    }

    InEdgeIterator getInEdgeIterator( const EdgeIterator& e) const
    {
        return InEdgeIterator( e->inEdge);
    }

    bool hasEdge( const NodeIterator& u, const NodeIterator& v) const
    {
        NodeIterator neigh;
        for( EdgeIterator e = beginEdges(u), endEdge = endEdges(u); e != endEdge; ++e)
        {
            neigh = target(e);
            if( neigh == v)
                return true;
        }

        return false;
    }

    bool hasEdges( const NodeIterator& v) const
    {
        return (outdeg(v) > 0);
    }

    bool hasInEdges( const NodeIterator& v) const
    {
        return (indeg(v) > 0);
    }

    SizeType memUsage()
    {
        std::cout << "Graph mem Usage\t\tNodes\tEdges\tInEdges\n";
        std::cout << "\tNumber:\t\t" << m_numNodes << "\t" << m_numEdges << "\t" << m_numEdges << std::endl;
        std::cout << "\tSize:\t\t" << sizeof(NodeData) << "\t" << sizeof(EdgeData) << "\t" << sizeof(InEdgeData) << std::endl;

        return ( m_numNodes * sizeof(NodeData) + m_numEdges * ( sizeof(EdgeData) + sizeof(InEdgeData)));
    }

    SizeType outdeg( const NodeIterator& v) const
    {
        return v->numOutEdges;
    }

    SizeType indeg( const NodeIterator& v) const
    {
        return v->numInEdges;
    }

    void reserve( const SizeType& numNodes, const SizeType& numEdges)
    {
        return;
    }

    void swapHeads( EdgeIterator& e1, EdgeIterator& e2)
    {
        InEdgeIterator k1 = getInEdgeIterator( e1);
        InEdgeIterator k2 = getInEdgeIterator( e2);

        std::swap( (*e1).target, (*e2).target);
        std::swap( (*e1).inEdge, (*e2).inEdge);
        std::swap( (*k1).outEdge, (*k2).outEdge);
    }
    
    void swapEdges( EdgeIterator& e1, EdgeIterator& e2)
    {
        InEdgeIterator k1 = getInEdgeIterator( e1);
        InEdgeIterator k2 = getInEdgeIterator( e2);

        std::swap( *e1, *e2);
        std::swap( (*k1).outEdge, (*k2).outEdge);
    }
    
    void swapEdges( InEdgeIterator& k1, InEdgeIterator& k2)
    {
        EdgeIterator e1 = getEdgeIterator( k1);
        EdgeIterator e2 = getEdgeIterator( k2);

        std::swap( *k1, *k2);
        std::swap( (*e1).inEdge, (*e2).inEdge);
    }

    bool hasInvalidInput( const uint32_t numNodes, const uint32_t numEdges, 
                          const std::vector<std::vector<uint32_t>>& outdegs, 
                          const std::vector<std::vector<uint32_t>>& indegs) const
    {
        if( outdegs.size() != numNodes)
            return true;

        if( indegs.size() != numNodes)
            return true;     

        int inCounter = 0, outCounter = 0;
        for( int i=0; i<numNodes; i++)
        {
            outCounter += outdegs[i].size();
            inCounter += indegs[i].size();

            for( int j=0; outdegs[i].size(); j++)
                if( outdegs[i][j] < 0 || outdegs[i][j] >= numNodes)
                    return true;

            for( int j=0; indegs[i].size(); j++)
                if( indegs[i][j] < 0 || indegs[i][j] >= numNodes)
                    return true;                
        }

        if( outCounter != numEdges)
            return true;

        if( inCounter != numEdges)
            return true;

        return false;
    }
    
  private:

    char* m_pool;
    uint64_t m_capacity;
    uint32_t m_numNodes;
    uint32_t m_numEdges;
    MersenneTwister m_random;
};

#endif
