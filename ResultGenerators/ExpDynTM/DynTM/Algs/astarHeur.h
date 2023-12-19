#ifndef TSP_A_STAR_PREPROCESSOR
#define TSP_A_STAR_PREPROCESSOR

#include <climits>

#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/forwardStarImpl.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>

#include "../Structs/definitions.h"
#include "../Algs/stDijkstra.h"

namespace DynTM
{

/**
 * @class AstarHeur
 *
 * @brief A* Preprocessing.
 *
 * This class supports computing and storing the lower bound distances from\to each station.
 *
 * @tparam TspNetwork The time expanded structure used for the representation of public transporation network.
 * @author Andreas Paraskevopoulos
 *
 */
template<typename TspNetwork>
class AstarHeur
{

 public:

    typedef typename TspNetwork::GraphImpl                              GraphType;
    typedef typename TspNetwork::SizeType                               SizeType;
    typedef typename TspNetwork::NodeDescriptor                         NodeDescriptor;
    typedef typename TspNetwork::NodeIterator                           NodeIterator;
    typedef typename TspNetwork::EdgeDescriptor                         EdgeDescriptor;
    typedef typename TspNetwork::EdgeIterator                           EdgeIterator;
    typedef typename TspNetwork::StationContainer                       StationContainer;
    typedef typename TspNetwork::Station                                Station;
    typedef StaticPriorityQueue<Distance, NodeIterator, HeapStorage>    PriorityQueueType;

    /**
     * @brief Constructor. Creates an instance of AstarHeur.
     * @param graph The graph to run the algorithm on.
     * @param stationContainer A container with the stations.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order 
     * to check whether a node is visited or not.
     */
    AstarHeur( GraphType& graph, StationContainer& netStations, TTL *timestamp) 
    : G(graph), stations(netStations), stAlgTimestamp(0)
    {}

    /**
     * @brief Creates the station graph of transporation network.
     */
    void createStationGraph()
    {
        //number of stations
        const unsigned int numStations = stations.size();

        ProgressStream stG_progress( 2*numStations);
        stG_progress.label() << "\tCreating station graph";

        //station graph
        stG.clear();

        //map: station id -> station node
        stMap.resize( numStations);
        for( StationID stId = 0; stId < numStations; stId++)
        {
            stMap[stId] = stG.insertNode();
            stG.getNodeIterator( stMap[stId])->stId = stId;
            ++stG_progress;
        }

        StNodeDescriptor tailSt, headSt;
        StEdgeIterator eConnx;

        NodeIterator depNode;
        EdgeIterator e, connEdge, endEdge;

        for( StationID stId = 0; stId < numStations; stId++)
        {
            const Station& station = stations[stId];
            const std::vector<StationID>& adjStations = station.adjStations;
            const std::vector<EdgeIterator>& stEdges = station.stEdges;

            tailSt = stMap[stId];

            //public transport travel
            for( unsigned int i=0, numAdjStations=adjStations.size(); i<numAdjStations; ++i)
            {
                headSt = stMap[adjStations[i]];
                Distance minTravelTime = std::numeric_limits<Distance>::max();

                //get the minimum travel time to nextStNode
                for( e = stEdges[i], endEdge = stEdges[i+1]; e != endEdge; ++e)
                {
                    depNode = G.target( e);
                    connEdge = G.beginEdges( depNode); 
                    assert( G.target( connEdge)->isSwitch() && G.target( connEdge)->stId ==adjStations[i]); 
 
                    if( connEdge->weight < minTravelTime)
                        minTravelTime = connEdge->weight;
                }

                eConnx = stG.getEdgeIterator( stG.insertEdge( tailSt, headSt));
                eConnx->weight = minTravelTime; 
            }

            //walking travel
            for( unsigned int i=0, numWalkPaths=station.walkPaths.size(); i<numWalkPaths; i++)
            {
                headSt = stMap[station.walkPaths[i].stId];
                const Distance& travelTime = station.walkPaths[i].travelTime;

                if( stG.hasEdge( tailSt, headSt))
                {
                    eConnx = stG.getEdgeIterator( tailSt, headSt);
                    if( eConnx->weight < travelTime)
                        eConnx->weight = travelTime;
                }

                else
                {
                    eConnx = stG.getEdgeIterator( stG.insertEdge( tailSt, headSt));
                    eConnx->weight = travelTime;
                }
            }

            //ev travel
            for( unsigned int i=0, numEVPaths=station.evPaths.size(); i<numEVPaths; i++)
            {
                headSt = stMap[station.evPaths[i].stId];
                const Distance& travelTime = station.evPaths[i].travelTime;

                if( stG.hasEdge( tailSt, headSt))
                {
                    eConnx = stG.getEdgeIterator( tailSt, headSt);
                    if( eConnx->weight < travelTime)
                        eConnx->weight = travelTime;
                }

                else
                {
                    eConnx = stG.getEdgeIterator( stG.insertEdge( tailSt, headSt));
                    eConnx->weight = travelTime;
                }
            }

            ++stG_progress;
        }
    }

    /**
     * @brief Computes A* heuristics.
     */
    void buildTrees()
    {
        StNodeIterator u, v, endNode;
        StEdgeIterator e;

        //number of stations
        const unsigned int numStations = stations.size();

        //create station graph
        createStationGraph();

        //create the container for A* heuristics 
        stLowBnds.resize( numStations);
        for( unsigned int i=0; i<numStations; i++)
            stLowBnds[i].resize( numStations);

        ProgressStream computing_progress( numStations);
        computing_progress.label() << "\tComputing distances for " << numStations << " stations";

        Dijkstra<StationGraphType, Distance> dij( stG, &stAlgTimestamp);

        //compute min distance from/to each station
        for( StationID depStId=0; depStId<numStations; ++depStId)
        {
            u = stG.getNodeIterator( stMap[depStId]);
            dij.buildTree( u);
         
            for( v = stG.beginNodes(), endNode = stG.endNodes(); v != endNode; ++v)
                stLowBnds[u->stId][v->stId] = v->dist;

            ++computing_progress;
        }
    }

    /**
     * @brief Writes A* heuristics.
     */
    void writeHeuristics( const std::string& lowBndFileName)
    {
        std::ofstream out( lowBndFileName.c_str(), std::ios::out | std::ios::binary);        
        out.exceptions( std::ofstream::failbit | std::ofstream::badbit );

        //number of stations
        const unsigned int numStations = stations.size();

        ProgressStream storing_progress( numStations);
        storing_progress.label() << "\tStoring distances for " << numStations << " stations";

        try
        {
            //number of stations
            out.write( (char*) ( &numStations), sizeof( numStations));

            //write min distance from/to each station
            for( StationID stId = 0; stId < numStations; stId++)
            {
                //write the data
                out.write( reinterpret_cast<char*>( &stLowBnds[stId].front()), numStations * sizeof(Distance));
                ++storing_progress;
            }

            out.close();
        }

        catch( std::ofstream::failure e) 
        {
            std::cerr << "\nError writing to file [" << lowBndFileName << "]. "
                      << "\nThe reason :" << e.what() << "\n";
            exit(1);
        }
    }

 private:

	//station graph node
	struct StNodeInfo : DefaultGraphItem
	{
        StNodeInfo() : dist(std::numeric_limits<Distance>::max()),
                       pqItem(std::numeric_limits<PQRange>::max()), pred(0), timestamp(0)
        {}

        StationID stId;
        Distance dist, distBack;
        PQRange pqItem, pqItemBack;
        void* pred, * succ;
        TTL timestamp;
	};

	//station graph edge
	struct StEdgeInfo : DefaultGraphItem
	{
        StEdgeInfo(): weight(std::numeric_limits<Distance>::max())
        {}

        union{ Distance travelTime, weight; };
    };

	//station graph inEdge
	struct StInEdgeInfo : DefaultGraphItem
	{};

	typedef DynamicGraph< AdjacencyListImpl, StNodeInfo, StEdgeInfo, StInEdgeInfo> StationGraphType;
    typedef typename StationGraphType::NodeIterator                     StNodeIterator;
    typedef typename StationGraphType::NodeDescriptor                   StNodeDescriptor;
    typedef typename StationGraphType::EdgeIterator                     StEdgeIterator;
    typedef typename StationGraphType::EdgeDescriptor                   StEdgeDescriptor;

    GraphType& G;
    StationContainer& stations;

    StationGraphType stG;
    std::vector<StNodeDescriptor> stMap;
    TTL stAlgTimestamp;

    std::vector< std::vector<Distance> > stLowBnds;
};

};

#endif
