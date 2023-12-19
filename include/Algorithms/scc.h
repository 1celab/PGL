#ifndef GRAPH_STRONGLY_CONNECTED_COMPONENTS_H
#define GRAPH_STRONGLY_CONNECTED_COMPONENTS_H

//standard header files
#include <algorithm>
#include <vector>
#include <stack>
#include <map>

//PGL lib header files
#include <Algorithms/marker.h>

/**
 * @class SCC
 *
 * @brief This class provides a graph processor for searching, marking
 * and processing the strongly connected components of a graph.
 *
 * @tparam GraphType The data structure used for the graph implementation.
 * @tparam DataArraySize The number of the algorithm data slots.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename GraphType, int DataArraySize=1>
class SCC
{

 public:

    typedef typename GraphType::NodeIterator                      NodeIterator;
    typedef typename GraphType::EdgeIterator                      EdgeIterator;
    typedef typename GraphType::InEdgeIterator                    InEdgeIterator;
    typedef typename GraphType::NodeDescriptor                    NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor                    EdgeDescriptor;
    typedef typename GraphType::SizeType                          SizeType;

    struct Component;
    struct NodeComponentData;
    struct NodeCompare;
    typedef std::vector<Component>                                 ComponentContainer;
    typedef std::map<NodeIterator, NodeComponentData, NodeCompare> NodeComponentDataMap;

    /**
     * @brief Creates a SCC object.
     * @param graph The input graph.
     * @param dataIndex The data slot of the algorithm.
     */
    SCC( GraphType& graph, const std::size_t dataIndex=0): G(graph), iDIX(dataIndex)
    {}


    /**
     * @brief Searches the strongly connected components of the graph.
     **/
    static void computeComponents( const GraphType& G, NodeComponentDataMap& data, ComponentContainer& components)
    {
        NodeIterator u, v, s, endNode;
        InEdgeIterator k, endInEdge;
        SizeType compId = 0;
        SizeType compSize = 0;

        std::stack<NodeIterator> S;
        std::stack<NodeIterator> orderedNodes;

        data.clear();
        components.clear();

        //initialize Forward DFS
        unmarkNodes( G, data);

        //run Forward DFS
        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
            if( isVisited( u, data) == false)
                runIterativeDFS( G, u, data, orderedNodes);

        //initialize Backward DFS
        unmarkNodes( G, data);

        //run Backward DFS
        while( !orderedNodes.empty())
        {
            u = orderedNodes.top();
            orderedNodes.pop();

            if( isVisited( u, data) == true)
                continue;

            //next strongly connected component
            compSize = 1;

            S.push( u);
            visit( u, data);
            setCompId( u, data, compId);

            while( !S.empty())
            {
                s = S.top();
                unsigned int ssize = S.size();

                for( k = G.beginInEdges( s), endInEdge = G.endInEdges( s); k != endInEdge; ++k)
                {
                    v = G.source( k);

                    if( isVisited( v, data) == false)
                    {
                        S.push( v);
                        visit( v, data);
                        setCompId( v, data, compId);
                        compSize++;
                        break;
                    }
                }
                
                if( ssize == S.size())
                    S.pop();
            }

            components.push_back( Component( compId, compSize));
            compId++;
        }

        //sort components by increasing size
        std::sort( components.begin(), components.end(), SCC<GraphType>::sortByCompSize);
    }

    /**
     * @brief Searches the strongly connected components of the graph.
     **/
    void computeComponents()
    {
        NodeIterator u, v, s, endNode;
        InEdgeIterator k, endInEdge;
        SizeType compId = 0;
        SizeType compSize = 0;

        std::stack<NodeIterator> S;
        std::stack<NodeIterator> orderedNodes;

        clear();

        //initialize Forward DFS
        unmarkNodes();

        //run Forward DFS
        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
            if( isVisited( u) == false)
                runIterativeDFS( u, orderedNodes);

        //initialize Backward DFS
        unmarkNodes();

        //run Backward DFS
        while( !orderedNodes.empty())
        {
            u = orderedNodes.top();
            orderedNodes.pop();

            if( isVisited( u) == true)
                continue;

            //next strongly connected component
            compSize = 1;

            S.push( u);
            visit( u);
            setCompId( u, compId);

            while( !S.empty())
            {
                s = S.top();
                unsigned int ssize = S.size();

                for( k = G.beginInEdges( s), endInEdge = G.endInEdges( s); k != endInEdge; ++k)
                {
                    v = G.source( k);

                    if( isVisited( v) == false)
                    {
                        S.push( v);
                        visit( v);
                        setCompId( v, compId);
                        compSize++;
                        break;
                    }
                }
                
                if( ssize == S.size())
                    S.pop();
            }

            components.push_back( Component( compId, compSize));
            compId++;
        }

        //sort components by increasing size
        std::sort( components.begin(), components.end(), this->sortByCompSize);
    }

    /**
     * @brief Returns the id of the largest strongly connected component.
     **/
    std::size_t getSizeLargestComponentId() const
    {
        if( components.empty() == false)
            return components.back().compId;
        else
            return 0;
    }

    /**
     * @brief Removes all components, except the largest one in graph.
     **/
    bool removeSmallSizeComponents()
    {
        NodeIterator u, v, endNode;
        EdgeIterator e, endEdge;
        InEdgeIterator k, endInEdge;

        std::vector<NodeDescriptor> erasedNodes;
        std::vector<EdgeDescriptor> erasedEdges;

        computeComponents();
        
        std::cout << "#strongly connected components: " <<  components.size() << std::endl;
        
        if( components.size() < 2)
            return false;

        erasedNodes.reserve( G.getNumNodes());
        erasedEdges.reserve( G.getNumEdges());

        SizeType keptCompId = components.back().compId;

        //find nodes and edges which are not in the largest strongly connected components, O(m)
        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        {            
            if( getCompId( u) != keptCompId)
            {
                erasedNodes.push_back( G.getNodeDescriptor( u));

                for( e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                    erasedEdges.push_back( G.getEdgeDescriptor( e));
            }

            else
            {
                for( e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                {
                    v = G.target( e);
                    if( getCompId( v) != keptCompId)
                        erasedEdges.push_back( G.getEdgeDescriptor( e));
                }
            }
        }

        //erase selected edges
        while( !erasedEdges.empty())
        {
            G.eraseEdge( erasedEdges.back());
            erasedEdges.pop_back();
        }

        //erase selected nodes
        while( !erasedNodes.empty())
        {
            G.eraseNode( erasedNodes.back());
            erasedNodes.pop_back();
        }

        return true;
    }


    void keepBiggerComponents( int numComps)
    {
        computeComponents();

        int i=0;
        const int lastCmp = components.size() - numComps;
        std::map<int,bool> cmpMap;
        for( i=0; i<lastCmp; i++)
            cmpMap[components[i].compId] = true;
        for(; i<components.size(); i++)
            cmpMap[components[i].compId] = false;

        NodeIterator u, v, endNode;
        EdgeIterator e, endEdge;
        InEdgeIterator k, endInEdge;

        std::vector<NodeDescriptor> erasedNodes;
        std::vector<EdgeDescriptor> erasedEdges;

        erasedNodes.reserve( G.getNumNodes());
        erasedEdges.reserve( G.getNumEdges());

        //find nodes and edges which are not in the largest strongly connected components, O(m)
        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        { 
	    if( cmpMap[getCompId( u)] == true)
            {
                erasedNodes.push_back( G.getNodeDescriptor( u));

                for( e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                    erasedEdges.push_back( G.getEdgeDescriptor( e));
            }

            else
            {
                for( e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                {
                    v = G.target( e);
                    if( cmpMap[getCompId( v)] == true)
                        erasedEdges.push_back( G.getEdgeDescriptor( e));
                }
            }
        }

        if( !erasedNodes.empty() || !erasedEdges.empty())
        {
            //erase selected edges
            while( !erasedEdges.empty())
            {
                G.eraseEdge( erasedEdges.back());
                erasedEdges.pop_back();
            }

            //erase selected nodes
            while( !erasedNodes.empty())
            {
                G.eraseNode( erasedNodes.back());
                erasedNodes.pop_back();
            }
        }

    }

    template<typename OutputGraphType>
    void buildReducedGraph( OutputGraphType& Go)
    {
        typedef typename OutputGraphType::NodeDescriptor ONodeDescriptor;
        Go.clear();

        std::vector<ONodeDescriptor> compNodes;
        for( unsigned int i=0; i<getNumComponents(); i++)
            compNodes[i] = Go.insertNode();

        for( NodeIterator u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
            for( EdgeIterator e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                NodeIterator v = G.target( e);

                if( getCompId( u) != getCompId( v))
                {
                    ONodeDescriptor cuD, cvD;
                    cuD = compNodes[getCompId(u)];
                    cvD = compNodes[getCompId(v)];
                    if( Go.hasEdge( cuD, cvD) == false)
                        Go.insertEdge( cuD, cvD);
                }
            }
    }

    /**
     * @brief Returns the computed strongly connected components of the graph.
     **/
    const ComponentContainer& getComponents() const
    {
        return components;
    }

    /**
     * @brief Returns the number of computed strongly connected components of the graph.
     * @return Number of components.
     **/
    unsigned int getNumComponents() const
    {
        return components.size();
    }

    /**
     * @brief Clear the component container.
     **/
    void clear()
    {
        components.clear();
    }

    /**
     * @brief Returns the id of the component in which node v belongs
     * @return A component id.
     **/
    virtual unsigned int getCompId( const NodeIterator& v)
    {
        return v->getCompId( iDIX);
    }

    struct Component
    {
        Component(): compId(0), compSize(0)
        {}

        Component( SizeType id, SizeType size): compId(id), compSize(size)
        {}

        SizeType compId;
        SizeType compSize;
    };

    struct NodeComponentData
    {
        NodeComponentData()
        {}

        NodeComponentData( SizeType id, bool visitFlag):
        compId(id), isVisited(visitFlag)
        {}

        NodeComponentData( const NodeComponentData& data)
        {
            compId = data.compId;
            isVisited = data.isVisited;
        }

        SizeType compId;
        bool isVisited;
    };

    struct NodeCompare
    {
        bool operator()( const NodeIterator &u, const NodeIterator &v) const
        {
            return ( (void*) &(*u) > (void*) &(*v));
        }
    };


 private:

    GraphType& G;
    ComponentContainer components;

    std::size_t iDIX;

    /**
     * @brief Marks all nodes of the graph as unvisited.
     **/
    virtual void unmarkNodes()
    {
        GraphType::NodeData::unmarkVisitedNodesByScc( G.beginNodes(), G.endNodes(), iDIX);
    }

    /**
     * @brief Marks all nodes of the graph as unvisited.
     **/
    static void unmarkNodes( const GraphType& G, NodeComponentDataMap& data)
    {
        for( NodeIterator u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
            data[u].isVisited = false;
    }

    /**
     * @brief Checks if the node v has been visited by the algorithm.
     * @return True if the node v is visited; False, otherwise.
     */
    virtual bool isVisited( const NodeIterator& v) const
    {
        return  v->isVisitedByScc( iDIX);
    }

    /**
     * @brief Marks the node v as visited.
     * @param v A node.
     **/
    virtual void visit( const NodeIterator& v)
    {
        v->markVisitedByScc( iDIX);
    }

    /**
     * @brief Checks if the node v has been visited by the algorithm.
     * @return True if the node v is visited; False, otherwise.
     */
    static bool isVisited( const NodeIterator& v, NodeComponentDataMap& data)
    {
        return  data[v].isVisited;
    }

    /**
     * @brief Marks the node v as visited.
     * @param v A node.
     **/
    static void visit( const NodeIterator& v, NodeComponentDataMap& data)
    {
        data[v].isVisited = true;
    }

    /**
     * @brief Sets the id of the component in which node v belongs
     * @param v A node.
     * @param id A component id.
     **/
    virtual void setCompId( const NodeIterator& v, unsigned int id)
    {
        v->setCompId( id, iDIX);
    }

    /**
     * @brief Sets the id of the component in which node v belongs
     * @param v A node.
     * @param id A component id.
     **/
    static void setCompId( const NodeIterator& v, NodeComponentDataMap& data, unsigned int id)
    {
        data[v].compId = id;
    }

    /**
     * @brief Performs a DFS ordering of nodes by decreasing finishing time.
     * @param root The root node.
     * @param orderedNodes A stack containing the DFS-ordered nodes.
     **/
    void runIterativeDFS( const NodeIterator& root, std::stack<NodeIterator> &orderedNodes)
    {
        std::stack<NodeIterator> stack;
        stack.push( root);
        visit( root);

        while( stack.empty() == false)
        {
            NodeIterator u = stack.top();

            unsigned int ssize = stack.size();

            for( EdgeIterator e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                NodeIterator v = G.target( e);

                if( isVisited( v) == false)
                {
                    visit( v);
                    stack.push( v);
                    break;
                }
            }

            if( ssize == stack.size())
            {
                stack.pop();
                orderedNodes.push( u);
            }
        }
    }

    /**
     * @brief Performs a DFS ordering of nodes by decreasing finishing time.
     * @param root The root node.
     * @param orderedNodes A stack containing the DFS-ordered nodes.
     **/
    static void runIterativeDFS( const GraphType& G, const NodeIterator& root, NodeComponentDataMap& data, std::stack<NodeIterator> &orderedNodes)
    {
        std::stack<NodeIterator> stack;
        stack.push( root);
        visit( root, data);

        while( stack.empty() == false)
        {
            NodeIterator u = stack.top();

            unsigned int ssize = stack.size();

            for( EdgeIterator e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                NodeIterator v = G.target( e);

                if( isVisited( v, data) == false)
                {
                    visit( v, data);
                    stack.push( v);
                    break;
                }
            }

            if( ssize == stack.size())
            {
                stack.pop();
                orderedNodes.push( u);
            }
        }
    }

    static bool sortByCompSize( const Component& c1, const Component& c2)
    {
        return c1.compSize < c2.compSize;
    }
};


class DefaultGlobalTimestampSccMarker : public DefaultMarkerGlobalTimestamp<std::size_t> {};
template <int DataArraySize> class DefaultSccNodeMarker : public ItemCollectionMultiMarker<1, DataArraySize, unsigned int, DefaultGlobalTimestampSccMarker> {};

template <int DataArraySize=1, class SccNodeMarker=DefaultSccNodeMarker<DataArraySize>>
struct SccNodeData
{
    SccNodeData()
    {
        for( std::size_t i=0; i<DataArraySize; i++)
            compId[i] = 0;
    }

    std::size_t compId[DataArraySize];
    SccNodeMarker sccNodeMarker[DataArraySize];

    void markVisitedByScc( const std::size_t i=0)
    {
        sccNodeMarker[i].mark( i);
    }

    bool isVisitedByScc( const std::size_t i=0)
    {
        return sccNodeMarker[i].isMarked( i);
    }

    struct SccNodeMarkerIterator
    {
        template<typename ObjectIterator>
        static void unmark( const ObjectIterator& it, const std::size_t i=0)
        {
            it->sccNodeMarker[i].unmark();
        }
    };

    template<typename ObjectIterator>
    static void unmarkVisitedNodesByScc( const ObjectIterator& beg, const ObjectIterator& end, std::size_t i=0)
    {
        SccNodeMarker::template init<SccNodeMarkerIterator, ObjectIterator>( beg, end, i);
    }

    void resetCompId( const std::size_t i=0)
    {
        setCompId( 0, i);
    }

    void setCompId( std::size_t id, const std::size_t i=0)
    {
        compId[i] = id;
    }

    unsigned int getCompId( const std::size_t i=0)
    {
        return compId[i];
    }
};

#endif
