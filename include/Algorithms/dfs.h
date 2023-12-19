#ifndef DEPTH_FIRST_SEARCH_H
#define DEPTH_FIRST_SEARCH_H

//standard header files
#include <stack>

//PGL lib header files
#include <Algorithms/marker.h>

/**
 * @class DFS
 *
 * @brief This class provides the Depth-first search (DFS) algorithm
 * for traversing a graph by depth.
 *
 * @tparam GraphType The data structure used for the graph implementation.
 * @tparam DataArraySize The number of the algorithm data slots.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename GraphType, std::size_t DataArraySize=1>
class DFS
{

 public:

    typedef typename GraphType::NodeIterator                      NodeIterator;
    typedef typename GraphType::EdgeIterator                      EdgeIterator;
    typedef typename GraphType::InEdgeIterator                    InEdgeIterator;
    typedef typename GraphType::NodeDescriptor                    NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor                    EdgeDescriptor;
    typedef typename GraphType::SizeType                          SizeType;

        /**
     * @brief Creates a DFS object.
     * @param graph The input graph.
     * @param dataIndex The data slot of the algorithm.
     */
    DFS( GraphType& graph, std::size_t dataIndex=0): G(graph), iDIX(dataIndex)
    {}

    void runQuery( const NodeIterator& s)
    {
        NodeIterator u, v;
        EdgeIterator e, endEdge;

        init( s);

        while( !S.empty())
        {
            u = S.top();
            S.pop();

            for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                v = G.target( e);

                if( isVisited( v) == false)
                {
                    visit( v);
                    S.push( v);
                }
            }
        }
    }

    int runQuery( const NodeIterator& s, int sourceLevel)
    {
        NodeIterator u, v;
        EdgeIterator e, endEdge;

        init( s, sourceLevel);
        int nextLevel = sourceLevel + 1;

        while( !S.empty())
        {
            u = S.top();
            S.pop();

            for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                v = G.target( e);

                if( isVisited( v) == false)
                {
                    visit( v);
                    S.push( v);
                    setLevel( v, nextLevel);
                    nextLevel++;
                }
            }
        }

        return getLevel( u);
    }


    void runQuery( const NodeIterator& s, std::size_t visitTime=0, std::size_t finishTime=0)
    {
        NodeIterator u, v;
        EdgeIterator e, endEdge;

        init( s);
        setVisitTime( s, visitTime);
        visitTime++;

        while( !S.empty())
        {
            u = S.top();
            const std::size_t ssize = S.size();

            for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                v = G.target( e);

                if( isVisited( v) == false)
                {
                    visit( v);
                    S.push( v);
                    setVisitTime( v, visitTime);
                    visitTime++;
                    break;
                }
            }

            if( ssize == S.size())
            {
                S.pop();
                setFinishTime(u, finishTime);
                finishTime++;
            }
        }
    }

    template<typename EdgeVisitor>
    void runCustomQuery( const NodeIterator& s, std::size_t visitTime=0, std::size_t finishTime=0)
    {
        NodeIterator u, v;
        const EdgeVisitor eVisitor;

        init( s);
        setVisitTime( s, visitTime);
        visitTime++;

        while( !S.empty())
        {
            u = S.top();
            const std::size_t ssize = S.size();

            for( typename EdgeVisitor::EdgeIterator e = eVisitor.getFirstEdge( G, u), endEdge = eVisitor.getEndEdge( G, u); e != endEdge; ++e)
            {
                v = eVisitor.getNextNode( G, e);

                if( isVisited( v) == false)
                {
                    visit( v);
                    S.push( v);
                    setVisitTime( v, visitTime);
                    visitTime++;
                    break;
                }
            }

            if( ssize == S.size())
            {
                S.pop();
                setFinishTime(u, finishTime);
                finishTime++;
            }
        }
    }
    
    bool existsPath( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u, v;
        EdgeIterator e, endEdge;

        init( s);

        while( !S.empty())
        {
            u = S.top();
            S.pop();

            if( u == t)
                return true;

            for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                v = G.target( e);

                if( isVisited( v) == false)
                {
                    visit( v);
                    S.push( v);
                }
            }
        }

        return false;
    }

 private:

    /*
     * @brief Initializes the algorithm.
     * @param s The source node.
     **/
    void init( const NodeIterator& s)
    {
        //mark all nodes as unvisited
        unmarkNodes();

        //clear the queue
        std::stack<NodeIterator> empty;
        std::swap( S, empty);

        S.push( s);
        visit( s);
    }

    /*
     * @brief Initializes the algorithm.
     * @param s The source node.
     * @param sourceLevel The source level value.
     **/
    void init( const NodeIterator& s, const int sourceLevel)
    {
        init( s);
        setLevel( s, sourceLevel);
    }

    /**
     * @brief Marks all nodes of the graph as unvisited.
     **/
    void unmarkNodes() const
    {
        GraphType::NodeData::unmarkVisitedNodesByDfs( G.beginNodes(), G.endNodes(), iDIX);
    }


    /**
     * @brief Sets the visit level of the node v.
     * @param v A node.
     * @param level The visit level of v in DFS tree.
     **/
    void setVisitTime( const NodeIterator& v, int level) const
    {
        v->setDfsVisitTime( level, iDIX);
    }

    /**
     * @brief Sets the level of the node v.
     * @param v A node.
     **/
    int getVisitTime( const NodeIterator& v) const
    {
        return v->getDfsVisitLevel( iDIX);
    }


    /**
     * @brief Sets the level of the node v.
     * @param v A node.
     * @param level The level of v in DFS tree.
     **/
    void setFinishTime( const NodeIterator& v, int level) const
    {
        v->setDfsFinishTime( level, iDIX);
    }

    /**
     * @brief Sets the level of the node v.
     * @param v A node.
     **/
    int getFinishTime( const NodeIterator& v) const
    {
        return v->getDfsFinishTime( iDIX);
    }

    /**
     * @brief Sets the level of the node v.
     * @param v A node.
     * @param level The level of v in DFS tree.
     **/
    void setLevel( const NodeIterator& v, int level) const
    {
        v->setDfsVisitTime( level, iDIX);
    }

    /**
     * @brief Sets the level of the node v.
     * @param v A node.
     **/
    int getLevel( const NodeIterator& v) const
    {
        return v->getDfsVisitTime( iDIX);
    }

    /**
     * @brief Marks the node v as visited.
     * @param v A node.
     **/
    void visit( const NodeIterator& v) const
    {
        v->markVisitedByDfs( iDIX);
    }

    /**
     * @brief Checks if node v has been visited by the algorithm.
     * @param v A node.
     * @return True if the node v is visited; False, otherwise.
     */
    bool isVisited( const NodeIterator& v) const
    {
        return v->isVisitedByDfs( iDIX);
    }

    GraphType& G;
    std::stack<NodeIterator> S;
    const std::size_t iDIX;
};

class DefaultGlobalTimestampDfsNodeMarker : public DefaultMarkerGlobalTimestamp<unsigned int> {};
template <int DataArraySize> class DefaultDfsNodeMarker : public ItemCollectionMultiMarker<1, DataArraySize, unsigned int, DefaultGlobalTimestampDfsNodeMarker> {};

template <int DataArraySize=1, class DfsNodeMarker=DefaultDfsNodeMarker<DataArraySize>>
struct DfsNodeData
{
    DfsNodeData()
    {}

    DfsNodeMarker dfsNodeMarker[DataArraySize];
    std::size_t dfsVisitTime[DataArraySize];
    int dfsFinishTime[DataArraySize];

    void setDfsVisitTime( int visitTime, std::size_t i=0)
    {
        dfsVisitTime[i] = visitTime;
    }

    int getDfsVisitTime( std::size_t i=0) const
    {
        return dfsVisitTime[i];
    }

    void setDfsFinishTime( int finishTime, std::size_t i=0)
    {
        dfsFinishTime[i] = finishTime;
    }

    int getDfsFinishTime( std::size_t i=0) const
    {
        return dfsFinishTime[i];
    }

    void markVisitedByDfs( const std::size_t i=0)
    {
        dfsNodeMarker[i].mark( i);
    }

    bool isVisitedByDfs( const std::size_t i=0) const
    {
        return dfsNodeMarker[i].isMarked( i);
    }

    struct DfsNodeMarkerIterator
    {
        template<typename ObjectIterator>
        static void unmark( const ObjectIterator& it, const std::size_t i=0)
        {
            it->dfsNodeMarker[i].unmark();
        }
    };

    template<typename ObjectIterator>
    static void unmarkVisitedNodesByDfs( const ObjectIterator& beg, const ObjectIterator& end, const std::size_t i=0)
    {
        DfsNodeMarker::template init<DfsNodeMarkerIterator, ObjectIterator>( beg, end, i);
    }
};

class DefaultGlobalTimestampDfsEdgeMarker : public DefaultMarkerGlobalTimestamp<unsigned int> {};
template <int DataArraySize> class DefaultDfsEdgeMarker : public ItemCollectionMultiMarker<1, DataArraySize, unsigned int, DefaultGlobalTimestampDfsEdgeMarker> {};

template <int DataArraySize=1, class DfsEdgeMarker=DefaultDfsEdgeMarker<DataArraySize>>
struct DfsEdgeData
{
    DfsEdgeData()
    {}

    DfsEdgeMarker dfsEdgeMarker[DataArraySize];

    void markVisitedByDfs( const std::size_t i=0)
    {
        dfsEdgeMarker[i].mark( i);
    }

    bool isVisitedByDfs( const std::size_t i=0) const
    {
        return dfsEdgeMarker[i].isMarked( i);
    }

    struct DfsEdgeMarkerIterator
    {
        template<typename ObjectIterator>
        static void unmark( const ObjectIterator& it, const std::size_t i=0)
        {
            it->dfsEdgeMarker[i].unmark();
        }
    };

    template<typename GraphType>
    static void unmarkVisitedEdgesByDfs( const GraphType& G, const std::size_t i=0)
    {
        DfsEdgeMarker::template initGraphEdges<DfsEdgeMarkerIterator, GraphType>( G, i);
    }
};
#endif
