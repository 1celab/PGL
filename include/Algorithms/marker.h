#ifndef TIMESTAMP_MARKER_H
#define TIMESTAMP_MARKER_H

/**
 * @class DefaultMarkerGlobalTimestamp
 *
 * @brief This class contains the default global timestamp definition of the ItemCollectionMarker class.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename TimestampType=unsigned int>
class DefaultMarkerGlobalTimestamp
{

 public:

    DefaultMarkerGlobalTimestamp()
    {
        resetGlobalTimestamp();
        //oDLOG("[+] Added Global timestamp.\n");
    }

    const TimestampType& getGlobalTimestamp() const
    {
        return globalTimestamp;
    }

    bool isOverflowReached() const
    {
        return (globalTimestamp == 0);
    }

    void updateGlobalTimestamp()
    {
        globalTimestamp++;
    }

    void setGlobalTimestamp( const TimestampType timestamp)
    {
        globalTimestamp = timestamp;
    }

    void resetGlobalTimestamp()
    {
        globalTimestamp = 1;
    }

 private:

    TimestampType globalTimestamp;
};

/**
 * @class ItemCollectionMarker
 *
 * @brief This class provides a timestamp-based method for performing K=2^{size of TimestampType in bits} iterations
 * of marking the items of a container (e.g. the nodes of a graph), with O(1) initialization.
 * Each time, when K iterations are performed, a O(N) initialization is followed in between,
 * where N is the number of the items of the used target container.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename TimestampType=unsigned int, typename MarkerGlobalTimestamp=DefaultMarkerGlobalTimestamp<TimestampType>>
class ItemCollectionMarker
{

 public:

    ItemCollectionMarker()
    {
        resetLocalTimestamp();
    }

    void mark()
    {
        timestamp = sync.getGlobalTimestamp();
    }

    bool isMarked() const
    {
        return timestamp == sync.getGlobalTimestamp();
    }

    void unmark()
    {
        resetLocalTimestamp();
    }

    template<typename MarkerIterator, typename ObjectIterator>
    static void init( const ObjectIterator& beg, const ObjectIterator& end)
    {
        sync.updateGlobalTimestamp();

        if( sync.isOverflowReached())
        {
            for( ObjectIterator it=beg; it!=end; ++it)
                MarkerIterator::unmark( it);

            sync.resetGlobalTimestamp();
        }
    }

    template<typename MarkerIterator, typename GraphType>
    static void initGraphEdges( const GraphType& G)
    {
        sync.updateGlobalTimestamp();

        if( sync.isOverflowReached())
        {
            for( typename GraphType::NodeIterator u=G.beginNodes(), endNode=G.endNodes(); u != endNode; ++u)
                for( typename GraphType::EdgeIterator e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                    MarkerIterator::unmark( e);

            sync.resetGlobalTimestamp();
        }
    }

 private:

    TimestampType timestamp;
    static MarkerGlobalTimestamp sync;

    void resetLocalTimestamp()
    {
        timestamp = 0;
    }
};

template <typename TimestampType, typename MarkerGlobalTimestamp>
MarkerGlobalTimestamp ItemCollectionMarker<TimestampType, MarkerGlobalTimestamp>::sync;

/**
 * @class ItemCollectionMultiMarker
 *
 * @brief This class provides a timestamp-based method for performing K=2^{size of TimestampType in bits} iterations
 * of marking the items of a container (e.g. the nodes of a graph), with O(1) initialization.
 * Each time, when K iterations are performed, a O(N) initialization is followed in between,
 * where N is the number of the items of the used target container.
 *
 * @author Andreas Paraskevopoulos
 **/
template <unsigned int NumMarkers=1, unsigned int NumGlobalTimestamps=1, typename TimestampType=unsigned int, typename MarkerGlobalTimestamp=DefaultMarkerGlobalTimestamp<TimestampType>>
class ItemCollectionMultiMarker
{

 public:

    ItemCollectionMultiMarker()
    {
        for( std::size_t i=0; i<NumMarkers; i++)
        {
            resetLocalTimestamp( i);
            //marker[i].bind( i);
        }
    }

    void mark( const std::size_t i=0)
    {
        timestamp[i] = sync[i].getGlobalTimestamp();
    }

    bool isMarked( const std::size_t i=0) const
    {
        return ( timestamp[i] == sync[i].getGlobalTimestamp());
    }

    void unmark( const std::size_t i=0)
    {
        resetLocalTimestamp( i);
    }

    template<typename MarkerIterator, typename ObjectIterator>
    static void init( const ObjectIterator& beg, const ObjectIterator& end, const std::size_t i=0)
    {
        sync[i].updateGlobalTimestamp();

        if( sync[i].isOverflowReached())
        {
            for( ObjectIterator it=beg; it!=end; ++it)
                MarkerIterator::unmark( it, i);

            sync[i].resetGlobalTimestamp();
        }
    }

    template<typename MarkerIterator, typename GraphType>
    static void initGraphEdges( const GraphType& G, const std::size_t i=0)
    {
        sync[i].updateGlobalTimestamp();

        if( sync[i].isOverflowReached())
        {
            for( typename GraphType::NodeIterator u=G.beginNodes(), endNode=G.endNodes(); u != endNode; ++u)
                for( typename GraphType::EdgeIterator e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                    MarkerIterator::unmark( e, i);

            sync[i].resetGlobalTimestamp();
        }
    }

 private:

    TimestampType timestamp[NumMarkers];
    static MarkerGlobalTimestamp sync[NumGlobalTimestamps];

    void resetLocalTimestamp( const std::size_t i=0)
    {
        timestamp[i] = 0;
    }

    /*struct Marker
    {
        Marker( const std::size_t i=0): iDIX(i)
        {}

        void bind( const std::size_t i=0)
        {
            iDIX = i;
        }

        void mark()
        {
            timestamp[iDIX] = sync[iDIX].getGlobalTimestamp();
        }

        bool isMarked() const
        {
            return timestamp[iDIX] == sync[iDIX].getGlobalTimestamp();
        }

        void unmark()
        {
            resetLocalTimestamp( iDIX);
        }

        template<typename MarkerIterator, typename ObjectIterator>
        void init( const ObjectIterator& beg, const ObjectIterator& end)
        {
            sync[iDIX].updateGlobalTimestamp();

            if( sync[iDIX].isOverflowReached())
            {
                for( ObjectIterator it=beg; it!=end; ++it)
                    MarkerIterator::unmark( it, iDIX);

                sync[iDIX].resetGlobalTimestamp();
            }
        }

        template<typename MarkerIterator, typename GraphType>
        void initGraphEdges( const GraphType& G)
        {
            sync[iDIX].updateGlobalTimestamp();

            if( sync[iDIX].isOverflowReached())
            {
                for( typename GraphType::NodeIterator u=G.beginNodes(), endNode=G.endNodes(); u != endNode; ++u)
                    for( typename GraphType::EdgeIterator e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                        MarkerIterator::unmark( e, iDIX);

                sync[iDIX].resetGlobalTimestamp();
            }
        }

        std::size_t iDIX;
    };

    Marker marker[NumMarkers];
    */
};

template <unsigned int NumMarkers, unsigned int NumGlobalTimestamps, typename TimestampType, typename MarkerGlobalTimestamp>
MarkerGlobalTimestamp ItemCollectionMultiMarker<NumMarkers, NumGlobalTimestamps, TimestampType, MarkerGlobalTimestamp>::sync[NumGlobalTimestamps];

template <unsigned int NumGlobalTimestamps, typename TimestampType, typename MarkerGlobalTimestamp>
class ItemCollectionMultiMarker<1, NumGlobalTimestamps, TimestampType, MarkerGlobalTimestamp>
{

 public:

    ItemCollectionMultiMarker()
    {
        resetLocalTimestamp();
    }

    void mark( const std::size_t i=0)
    {
        timestamp = sync[i].getGlobalTimestamp();
    }

    bool isMarked( const std::size_t i=0) const
    {
        return timestamp == sync[i].getGlobalTimestamp();
    }

    void unmark()
    {
        resetLocalTimestamp();
    }

    template<typename MarkerIterator, typename ObjectIterator>
    static void init( const ObjectIterator& beg, const ObjectIterator& end, const std::size_t i=0)
    {
        sync[i].updateGlobalTimestamp();

        if( sync[i].isOverflowReached())
        {
            for( ObjectIterator it=beg; it!=end; ++it)
                MarkerIterator::unmark( it, i);

            sync[i].resetGlobalTimestamp();
        }
    }


    template<typename MarkerIterator, typename GraphType>
    static void initGraphEdges( const GraphType& G, const std::size_t i=0)
    {
        sync[i].updateGlobalTimestamp();

        if( sync[i].isOverflowReached())
        {
            for( typename GraphType::NodeIterator u=G.beginNodes(), endNode=G.endNodes(); u != endNode; ++u)
                for( typename GraphType::EdgeIterator e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                    MarkerIterator::unmark( e, i);

            sync[i].resetGlobalTimestamp();
        }
    }

 private:

    TimestampType timestamp;
    static MarkerGlobalTimestamp sync[NumGlobalTimestamps];

    void resetLocalTimestamp()
    {
        timestamp = 0;
    }
};

template <unsigned int NumGlobalTimestamps, typename TimestampType, typename MarkerGlobalTimestamp>
MarkerGlobalTimestamp ItemCollectionMultiMarker<1, NumGlobalTimestamps, TimestampType, MarkerGlobalTimestamp>::sync[NumGlobalTimestamps];

/*template <typename TimestampType, typename MarkerGlobalTimestamp>
class ItemCollectionMultiMarker<1, 1, TimestampType, MarkerGlobalTimestamp> : public ItemCollectionMarker<TimestampType, MarkerGlobalTimestamp>
{};*/


class DefaultGlobalTimestampNodeMarker : public DefaultMarkerGlobalTimestamp<std::size_t> {};
template <std::size_t DataArraySize> class DefaultNodeMarker : public ItemCollectionMultiMarker<1, DataArraySize, std::size_t, DefaultGlobalTimestampNodeMarker> {};

template <std::size_t DataArraySize=1, class NodeMarker=DefaultNodeMarker<DataArraySize>>
struct MarkerNodeData
{
    MarkerNodeData()
    {}

    NodeMarker nodeMarker[DataArraySize];

    void markVisited( const std::size_t i=0)
    {
        nodeMarker[i].mark( i);
    }

    void markUnvisited( const std::size_t i=0)
    {
        nodeMarker[i].unmark();
    }

    bool isVisited( const std::size_t i=0) const
    {
        return nodeMarker[i].isMarked( i);
    }

    struct NodeMarkerIterator
    {
        template<typename ObjectIterator>
        static void unmark( const ObjectIterator& it, const std::size_t i=0)
        {
            it->nodeMarker[i].unmark();
        }
    };

    template<typename ObjectIterator>
    static void unmarkVisitedNodes( const ObjectIterator& beg, const ObjectIterator& end, const std::size_t i=0)
    {
        NodeMarker::template init<NodeMarkerIterator, ObjectIterator>( beg, end, i);
    }
};

class DefaultGlobalTimestampEdgeMarker : public DefaultMarkerGlobalTimestamp<unsigned int> {};
template <int DataArraySize> class DefaultEdgeMarker : public ItemCollectionMultiMarker<1, DataArraySize, unsigned int, DefaultGlobalTimestampEdgeMarker> {};

template <int DataArraySize=1, class EdgeMarker=DefaultEdgeMarker<DataArraySize>>
struct MarkerEdgeData
{
    MarkerEdgeData()
    {}

    EdgeMarker edgeMarker[DataArraySize];

    void markVisited( const std::size_t i=0)
    {
        edgeMarker[i].mark( i);
    }

    void markUnvisited( const std::size_t i=0)
    {
        edgeMarker[i].unmark();
    }

    bool isVisited( const std::size_t i=0) const
    {
        return edgeMarker[i].isMarked( i);
    }

    struct EdgeMarkerIterator
    {
        template<typename ObjectIterator>
        static void unmark( const ObjectIterator& it, const std::size_t i=0)
        {
            it->edgeMarker[i].unmark();
        }
    };

    template<typename GraphType>
    static void unmarkVisitedEdges( const GraphType& G, const std::size_t i=0)
    {
        EdgeMarker::template initGraphEdges<EdgeMarkerIterator, GraphType>( G, i);
    }
};

#endif
