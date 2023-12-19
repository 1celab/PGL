#ifndef ROAD_NETWORK_READER_H
#define ROAD_NETWORK_READER_H

#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/forwardStarImpl.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>

#include <Utilities/geographic.h>
#include "../Algs/stDijkstra.h"

template< class TspNetwork>
class TspPreprocessor
{

 public:

    typedef float WeightType;

    //graph Node
    struct RNodeInfo : DefaultGraphItem
    {
        RNodeInfo(): stId( std::numeric_limits<StationID>::max()), timestamp(0)
        {}

        double lat, lon, hgt;
        StationID stId;
        void *pred;
        WeightType dist;
        unsigned int timestamp;
        unsigned int pqItem;
    };

    //graph Edge
    struct REdgeInfo : DefaultGraphItem
    {
        std::string roadType;
        union{ WeightType weight, travelTime; };
        int length;
        int speed;
    };

    //graph InEdge
    struct RInEdgeInfo : DefaultGraphItem
    {};

    typedef DynamicGraph< PackedMemoryArrayImpl, RNodeInfo, REdgeInfo, RInEdgeInfo>   pmaGraph;
    typedef DynamicGraph< ForwardStarImpl, RNodeInfo, REdgeInfo, RInEdgeInfo>         fsGraph;
    typedef DynamicGraph< AdjacencyListImpl, RNodeInfo, REdgeInfo, RInEdgeInfo>       GraphType;

    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::InEdgeIterator  InEdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    typedef typename TspNetwork::StationContainer                 StationContainer;
    typedef typename TspNetwork::Station                          Station;


    TspPreprocessor( StationContainer& netstations) : stations( netstations), algTimestamp( 0)
    {}

    void fillNearestNodeVector( std::vector<NodeIterator>& nearestNode, const double maxLength)
    {
        const unsigned int numStations = stations.size();

        //geo bounds of station network
        double minSLon, maxSLon, minSLat, maxSLat;
        minSLon = std::numeric_limits<double>::max();
        minSLat = std::numeric_limits<double>::max();
        maxSLon = std::numeric_limits<double>::min();
        maxSLat = std::numeric_limits<double>::min();

        for( StationID stId=0; stId<numStations; stId++)
        {
            const Station& station = stations[stId];

            if( minSLon > station.lon)
                minSLon = station.lon;

            if( minSLat > station.lat)
                minSLat = station.lat;

            if( maxSLon < station.lon)
                maxSLon = station.lon;

            if( maxSLat < station.lat)
                maxSLat = station.lat;
        }

        //set block width
        const double width = 0.000009 * maxLength / 2;

        //grid dimensions
        const unsigned int dimX = ( maxSLon - minSLon) / width;
        const unsigned int dimY = ( maxSLat - minSLat) / width;

        ProgressStream grid_progress( G.getNumNodes()+1);
        grid_progress.label() << "\tConstruct grid (" << dimX << "x" << dimY << ")";
        ++grid_progress;

        //grid
        std::vector< std::vector< std::vector<NodeIterator> > > grid;
        grid.resize( dimX);
        for( unsigned int i=0; i<dimX; i++)
            grid[i].resize( dimY);

        //grid assignment
        for( NodeIterator v = G.beginNodes(), endNode = G.endNodes(); v != endNode; ++v)
        {
            if( ( v->lon < minSLon || v->lon > maxSLon) || ( v->lat < minSLat || v->lat > maxSLat))
                continue;

            const unsigned int ix = ( (unsigned int) ( (v->lon - minSLon) / width)) % dimX;
            const unsigned int iy = ( (unsigned int) ( (v->lat - minSLat) / width)) % dimY;

            grid[ix][iy].push_back( v);
        }

        ProgressStream nearest_progress( numStations);
        nearest_progress.label() << "\tFind station access points";

        unsigned int numAccessPoints = 0;

        for( StationID stId=0; stId<numStations; stId++)
        {
            const Station& station = stations[stId];

            double minDist = std::numeric_limits<double>::max();

            const unsigned int ix = ( (unsigned int) ( (station.lon - minSLon) / width)) % dimX;
            const unsigned int iy = ( (unsigned int) ( (station.lat - minSLat) / width)) % dimY;

            //std::cout << "block (" << ix << "," << iy << ")\n";

            getEarliestNode( nearestNode, minDist, grid[ix][iy], station, stId);
            if( ix != 0) getEarliestNode( nearestNode, minDist, grid[ix-1][iy], station, stId);
            if( iy != 0) getEarliestNode( nearestNode, minDist, grid[ix][iy-1], station, stId);
            if( (ix+1) != dimX) getEarliestNode( nearestNode, minDist, grid[ix+1][iy], station, stId);
            if( (iy+1) != dimY) getEarliestNode( nearestNode, minDist, grid[ix][iy+1], station, stId);
            if( ix != 0 && iy != 0) getEarliestNode( nearestNode, minDist, grid[ix-1][iy-1], station, stId);
            if( ix != 0 && (iy+1) != dimY) getEarliestNode( nearestNode, minDist, grid[ix-1][iy+1], station, stId);
            if( (ix+1) != dimX && iy != 0) getEarliestNode( nearestNode, minDist, grid[ix+1][iy-1], station, stId);
            if( (ix+1) != dimX && (iy+1) != dimY) getEarliestNode( nearestNode, minDist, grid[ix+1][iy+1], station, stId);

            if( minDist > maxLength)
                nearestNode[stId] = G.endNodes();
            else
            {
                numAccessPoints++;
                nearestNode[stId]->stId = stId;
            }

           /*
            minDist = std::numeric_limits<double>::max();
            NodeIterator vSt;
            for( NodeIterator u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
            {
                double dist = distance( u->lat, u->lon, station.lat, station.lon);
                if( dist < minDist)
                {
                    vSt = u;
                    minDist = dist;
                }
            }

            if( vSt != nearestNode[stId]){ std::cout << "\nMISS\n"; }
            else {std::cout << "\nHIT\n"; }*/

            ++nearest_progress;
        }

        std::cout << "\tNumber of stations with road access:" << numAccessPoints << "\n";
    }

    void getEarliestNode( std::vector<NodeIterator>& nearestNode, double& minDist, const std::vector<NodeIterator>& areaNodes, const Station& station, const StationID stId)
    {
        for( unsigned int j=0; j<areaNodes.size(); j++)
        {
            NodeIterator v = areaNodes[j];
            double dist = distance( v->lat, v->lon, station.lat, station.lon);
            if( dist < minDist)
            {
                nearestNode[stId] = v;
                minDist = dist;
            }
        }

    }

    void setWalkPaths( const double maxRadius)
    {
        for( NodeIterator u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
            for( EdgeIterator e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                e->travelTime = e->length; //speed: 1m/s

        const unsigned int numStations = stations.size();
        const double walkSpeed = 1; //speed in m/s
        const double maxLength = maxRadius * walkSpeed; //length in m
        const unsigned int maxNumTargets = 2;

        //the nearest road graph node to station
        std::vector<NodeIterator> nearestNode;
        nearestNode.resize( numStations, G.endNodes());

        fillNearestNodeVector( nearestNode, maxLength);

        ProgressStream walkingTimes_progress( numStations);
        walkingTimes_progress.label() << "\tCompute walking travel times";

        Dijkstra<GraphType, WeightType> dij( G, &algTimestamp);
        unsigned int numWalkPaths = 0;

        for( StationID stId=0; stId<numStations; stId++)
        {
            Station& station = stations[stId];
            NodeIterator s = nearestNode[stId];

            if( s != G.endNodes())
            {
                std::vector<NodeIterator> targets;
                dij.reachNearestStations( s, targets, maxNumTargets, maxRadius);

                numWalkPaths += targets.size();

                const double travelTimeWk1 = distance( s->lat, s->lon, station.lat, station.lon) / walkSpeed;

                //std::cout << "\nsettled nodes:" << dij.getNumSettledNodes();
                for( unsigned int i=0, numTargets=targets.size(); i<numTargets; i++)
                {
                    const double travelTimeWk2 = distance( targets[i]->lat, targets[i]->lon, stations[targets[i]->stId].lat, stations[targets[i]->stId].lon) / walkSpeed;

                    targets[i]->dist += ( travelTimeWk1 + travelTimeWk2);
                    station.addWalkPath( targets[i]->stId, round( targets[i]->dist / 60.0));
                    /*std::cout << "\nFROM: " << station.lat << " " << station.lon << " TO: " << stations[targets[i]->stId].lat << " " << stations[targets[i]->stId].lon
                              << " travel time" << station.walkPaths.back().travelTime << std::endl; {int a; std::cin >>a;}*/
                }
            }

            ++walkingTimes_progress;
        }

        std::cout << "\tNumber of walk paths:" << numWalkPaths << "\n";
    }

    void setEVPaths( const double maxRadius)
    {
        for( NodeIterator u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
            for( EdgeIterator e = G.beginEdges(u), endEdge = G.endEdges(u); e != endEdge; ++e)
                e->travelTime = ceil( ( e->length * 3600.0) / ( e->speed * 1000.0));

        const unsigned int numStations = stations.size();
        const double evSpeed = 50; //ev speed in m/s
        const double walkSpeed = 1; //walk speed in m/s
        const double maxLength = maxRadius * evSpeed; //length in m
        const unsigned int maxNumTargets = 6;

        selectRandomEVStation();

        //the nearest road graph node to station
        std::vector<NodeIterator> nearestNode;
        nearestNode.resize( numStations, G.endNodes());

        fillNearestNodeVector( nearestNode, maxLength);

        ProgressStream evTimes_progress( numStations);
        evTimes_progress.label() << "\tCompute ev travel times";

        Dijkstra<GraphType, WeightType> dij( G, &algTimestamp);
        unsigned int numEVPaths = 0;

        for( StationID stId=0; stId<numStations; stId++)
        {
            Station& station = stations[stId];
            if( station.isEVStation == true)
            {
                NodeIterator s = nearestNode[stId];

                if( s != G.endNodes())
                {
                    std::vector<NodeIterator> targets;
                    dij.reachNearestStations( s, targets, maxNumTargets, maxRadius);

                    numEVPaths += targets.size();

                    const double travelTimeWk1 = distance( s->lat, s->lon, station.lat, station.lon) / walkSpeed;

                    for( unsigned int i=0, numTargets=targets.size(); i<numTargets; i++)
                        if( stations[targets[i]->stId].isEVStation == true)
                        {
                            const double travelTimeWk2 = distance( targets[i]->lat, targets[i]->lon, stations[targets[i]->stId].lat, stations[targets[i]->stId].lon) / walkSpeed;
                            targets[i]->dist += ( travelTimeWk1 + travelTimeWk2);
                            station.addEVPath( targets[i]->stId, round( targets[i]->dist / 60.0), computeEVConsumption(targets[i]));
                        }
                }
            }

            ++evTimes_progress;
        }

        std::cout << "\tNumber of ev paths:" << numEVPaths << "\n";
    }

    double computeEVConsumption( NodeIterator t)
    {
        const double k = 0.02;
        const double l = 1;
        const double a = 0.25;

        double consumption = 0;
        while( t->pred != G.nilNodeDescriptor())
        {
            NodeIterator v = G.getNodeIterator( t->pred);
            EdgeIterator e = G.getEdgeIterator( v, t);
            if( t->hgt >= v->hgt)
                consumption += (k * e->length + l * (t->hgt - v->hgt));
            else
                consumption += (k * e->length + a * (t->hgt - v->hgt));

            t = v;
        }

        return consumption;
    }

    void selectRandomEVStation()
    {
        int numEVStations = 10;/*0.20 * stations.size();*/
        std::cout << "\tEV stations:" << numEVStations << std::endl;

        boost::mt19937 gen(2016);
        boost::uniform_real<> dist(0, 1);
        boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random(gen, dist);
        while( numEVStations > 0)
        {
            StationID stationId;
            do{ stationId = random() * stations.size();} while( stations[stationId].isEVStation == true);

            stations[stationId].isEVStation = true;

            --numEVStations;
        }
    }

    /**
     *@brief calculate the distance in m between two points
     *@param Latitude and Longitude of the points.
     */
    double distance( const double lon1, const double lat1, const double lon2, const double lat2)
    {
        return greatCircle( lat1, lon1, lat2, lon2) * 1.609344;
    }

    void readData( std::string graphFilename)
    {
        std::string line;
        std::stringstream data;
        SizeType numNodes, numEdges;

        SizeType uId, vId;
        double length, speed, travelTime;
        bool isForward, isBackward;
        std::string roadType;

        std::ifstream in;
        in.exceptions( std::ifstream::failbit | std::ifstream::badbit );
        in.open( graphFilename.c_str());

        try{

            //skip comment lines
            do{ getline( in, line); } while( line[0] == '#');

            //read number of nodes
            data.str( line);
            data >> numNodes;
            data.clear();
            data.str("");

            //read number of edges
            getline( in, line);
            data.str( line);
            data >> numEdges;
            data.clear();
            data.str("");

            std::cout << "\tCreate road network\n";

            //reserve graph space
            G.clear();
            m_ids.clear();
            G.reserve( numNodes, numEdges);
            m_ids.reserve( numNodes);

            //create nodes
            for( SizeType i = 0; i<numNodes; ++i)
                G.insertNode();

            ProgressStream node_progress( numNodes);
            node_progress.label() << "\tReading " << numNodes << " nodes";

            //read node data
            for( NodeIterator v = G.beginNodes(), endNode = G.endNodes(); v != endNode; ++v)
            {
                getline( in, line);
                data.str( line);
                data >> vId >> v->lat >> v->lon >> v->hgt;
                data.clear();
                data.str("");

                //std::cout << " " << vId << " " << v->lat << " " << v->lon << " " << v->hgt;
                //{int a; std::cin >> a;}

                m_ids.push_back( G.getNodeDescriptor( v));

                ++node_progress;
            }

            ProgressStream edge_progress( numEdges);
            edge_progress.label() << "\tReading " << numEdges << " edges";

            //read edge data
            while( numEdges>0)
            {
                getline( in, line);
                data.str( line);
                data >> uId >> vId >> length >> roadType >> speed >> isForward >> isBackward;
                data.clear();
                data.str("");

                //std::cout << " " << uId << " " << vId << " " << length << " " << roadType << " " << speed << " " << isForward << " " << isBackward;
                //{int a; std::cin >> a;}

                travelTime = ceil( ( length * 3600.0) / ( speed * 1000.0));

                if( isForward)
                {
                    EdgeDescriptor eD = G.insertEdge( id2Desc( uId), id2Desc( vId));
                    EdgeIterator e = G.getEdgeIterator( eD);
                    e->weight = travelTime;
                    e->length = length;
                    e->speed = speed;
                    e->roadType = roadType;
                }

                if( isBackward)
                {
                    EdgeDescriptor eD = G.insertEdge( id2Desc( vId), id2Desc( uId));
                    EdgeIterator e = G.getEdgeIterator( eD);
                    e->weight = travelTime;
                    e->length = length;
                    e->speed = speed;
                    e->roadType = roadType;
                }

                ++edge_progress;
                --numEdges;
            }
        }

        catch ( std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << graphFilename << "'\n";
            throw e;
        }
    }

    NodeDescriptor id2Desc( SizeType id)
    {
        return m_ids[id];
    }

    std::vector<NodeDescriptor>& getIds()
    {
        return m_ids;
    }

    GraphType& getGraph()
    {
        return G;
    }

 private:

    std::string m_graphFilename;
    std::vector<NodeDescriptor> m_ids;
    StationContainer& stations;
    unsigned int algTimestamp;
    GraphType G;
};


#endif
