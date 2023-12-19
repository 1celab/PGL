#ifndef BREADTH_FIRST_SEARCH_H
#define BREADTH_FIRST_SEARCH_H

//standard header files
#include <queue>

//PGL lib header files
#include <Algorithms/marker.h>


template <typename GraphType>
struct DefaultBFSVisitor
{
        typedef typename GraphType::NodeIterator                      NodeIterator;
        typedef typename GraphType::EdgeIterator                      EdgeIterator;

        void init( const NodeIterator& v)
        {}

        void pop( const NodeIterator& v)
        {}
        void visitFirstTime( const NodeIterator& u, const EdgeIterator& e, const NodeIterator& v)
        {}

        void visitAgain( const NodeIterator& u, const EdgeIterator& e, const NodeIterator& v)
        {}
};

/**
 * @class BFS
 *
 * @brief This class provides the Breadth-first search (BFS) algorithm 
 * for traversing a graph by breadth.
 *
 * @tparam GraphType The data structure used for the graph implementation.
 * @tparam DataArraySize The number of the algorithm data slots.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename GraphType, template <typename graphType> class BFSVisitor=DefaultBFSVisitor, std::size_t DataArraySize=1>
class BFS
{

 public:

    typedef typename GraphType::NodeIterator                      NodeIterator;
    typedef typename GraphType::EdgeIterator                      EdgeIterator;
    typedef typename GraphType::InEdgeIterator                    InEdgeIterator;
    typedef typename GraphType::NodeDescriptor                    NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor                    EdgeDescriptor;
    typedef typename GraphType::SizeType                          SizeType;

        /**
     * @brief Creates a BFS object.
     * @param graph The input graph.
     * @param dataIndex The data slot of the algorithm.
     */
    BFS( GraphType& graph, std::size_t dataIndex=0): G(graph), iDIX(dataIndex)
    {}

    void runQuery( const NodeIterator& s)
    {
        NodeIterator u, v;
        EdgeIterator e, endEdge;

        init( s);
        extraWork.init(s);

        while( !Q.empty())
        {
            u = Q.front();
            Q.pop();
            extraWork.pop(u);

            for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                v = G.target( e);
		
                if( isVisited( v) == false)
                {
		    extraWork.visitFirstTime(u, e, v);
                    visit( v);
                    Q.push( v);
                }
                else
                    extraWork.visitAgain(u, e, v);
            }
        }
    }

    int runQuery( const NodeIterator& s, int sourceLevel)
    {
        NodeIterator u, v;
        EdgeIterator e, endEdge;

        init( s, sourceLevel);

        while( !Q.empty())
        {
            u = Q.front();
            Q.pop();

            const int nextLevel = getLevel( u) + 1;
            for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                v = G.target( e);

                if( isVisited( v) == false)
                {
                    visit( v);
                    Q.push( v);
                    setLevel( v, nextLevel);
                }
            }
        }

        return getLevel( u);
    }


    bool existsPath( const NodeIterator& s, const NodeIterator& t)
    {
        NodeIterator u, v;
        EdgeIterator e, endEdge;

        init( s);

        while( !Q.empty())
        {
            u = Q.front();
            Q.pop();

            if( u == t)
                return true;

            for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                v = G.target( e);

                if( isVisited( v) == false)
                {
                    visit( v);
                    Q.push( v);
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
        std::queue<NodeIterator> empty;
        std::swap( Q, empty);

        Q.push( s);
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
        GraphType::NodeData::unmarkVisitedNodesByBfs( G.beginNodes(), G.endNodes(), iDIX);
    }
    
    /**
     * @brief Sets the level of the node v.
     * @param v A node.
     * @param level The level of v in BFS tree.
     **/
    void setLevel( const NodeIterator& v, int level)
    {
        v->setBfsLevel( level, iDIX);
    }

    /**
     * @brief Sets the level of the node v.
     * @param v A node.
     **/
    int getLevel( const NodeIterator& v) const
    {
        return v->getBfsLevel( iDIX);
    }

    /**
     * @brief Marks the node v as visited.
     * @param v A node.
     **/
    void visit( const NodeIterator& v) const
    {
        v->markVisitedByBfs( iDIX);
    }

    /**
     * @brief Checks if node v has been visited by the algorithm.
     * @param v A node.
     * @return True if the node v is visited; False, otherwise.
     */
    bool isVisited( const NodeIterator& v) const
    {
        return  v->isVisitedByBfs( iDIX);
    }
    
    GraphType& G;
    std::queue<NodeIterator> Q;
    BFSVisitor<GraphType> extraWork;
    const std::size_t iDIX;
};

class DefaultGlobalTimestampBfsNodeMarker : public DefaultMarkerGlobalTimestamp<unsigned int> {};
template <int DataArraySize> class DefaultBfsNodeMarker : public ItemCollectionMultiMarker<1, DataArraySize, unsigned int, DefaultGlobalTimestampBfsNodeMarker> {};

template <int DataArraySize=1, class BfsNodeMarker=DefaultBfsNodeMarker<DataArraySize>>
struct BfsNodeData
{
    BfsNodeData()
    {}

    BfsNodeMarker bfsNodeMarker[DataArraySize];
    int bfsLevel[DataArraySize];

    /**
     * @brief Sets the level of the node v.
     * @param v A node.
     **/
    void setBfsLevel( int level, std::size_t i=0) const
    {
        bfsLevel[i] = level;
    }

    /**
     * @brief Sets the level of the node v.
     * @param v A node.
     **/
    int getBfsLevel( std::size_t i=0) const
    {
        return bfsLevel[i];
    }

    void markVisitedByBfs( const std::size_t i=0)
    {
        bfsNodeMarker[i].mark( i);
    }

    bool isVisitedByBfs( const std::size_t i=0) const
    {
        return bfsNodeMarker[i].isMarked( i);
    }

    struct BfsNodeMarkerIterator
    {
        template<typename ObjectIterator>
        static void unmark( const ObjectIterator& it, const std::size_t i=0)
        {
            it->bfsNodeMarker[i].unmark();
        }
    };

    template<typename ObjectIterator>
    static void unmarkVisitedNodesByBfs( const ObjectIterator& beg, const ObjectIterator& end, const std::size_t i=0)
    {
        BfsNodeMarker::template init<BfsNodeMarkerIterator, ObjectIterator>( beg, end, i);
    }
};

class DefaultGlobalTimestampBfsEdgeMarker : public DefaultMarkerGlobalTimestamp<unsigned int> {};
template <int DataArraySize> class DefaultBfsEdgeMarker : public ItemCollectionMultiMarker<1, DataArraySize, unsigned int, DefaultGlobalTimestampBfsEdgeMarker> {};

template <int DataArraySize=1, class BfsEdgeMarker=DefaultBfsEdgeMarker<DataArraySize>>
struct BfsEdgeData
{
    BfsEdgeData()
    {}

    BfsEdgeMarker bfsEdgeMarker[DataArraySize];

    void markVisitedByBfs( const std::size_t i=0)
    {
        bfsEdgeMarker[i].mark( i);
    }

    bool isVisitedByBfs( const std::size_t i=0) const
    {
        return bfsEdgeMarker[i].isMarked( i);
    }

    struct BfsEdgeMarkerIterator
    {
        template<typename ObjectIterator>
        static void unmark( const ObjectIterator& it, const std::size_t i=0)
        {
            it->bfsEdgeMarker[i].unmark();
        }
    };

    template<typename GraphType>
    static void unmarkVisitedEdgesByBfs( const GraphType& G, const std::size_t i=0)
    {
        BfsEdgeMarker::template initGraphEdges<BfsEdgeMarkerIterator, GraphType>( G, i);
    }
};
#endif
