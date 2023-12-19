#ifndef TRANSPORT_NETWORK_H
#define TRANSPORT_NETWORK_H

#include <iostream>
#include <sstream>
#include <fstream>

#include <algorithm>
#include <vector>
#include <string>
#include <cmath>
#include <list>
#include <map>

#include "definitions.h"

//graph tsp node
struct TspNode
{
    TspNode() : stId(0), vhId(0), tpId(0), idf(-1), label('-'), time(0)
    {}

    //network info
    NodeID id;
    StationID stId;
    VehicleID vhId;
    VehicleTypeID tpId;
    int idf;
    void* next;

    //time event info ('d':departure, 'a':arrival, '-':undefined)
    char label;
    Time time;

    bool isSwitch() 
    {
        return ( label == 's');
    }

    bool isDeparture() 
    {
        return ( label == 'd');
    }

    bool operator <( const TspNode& node) const
    {
        if( time < node.time)
            return true;
        else
            return false;
    }
};

//graph tsp edge
struct TspEdge
{
    TspEdge(): weight(0)
    {}

    union{ Distance travelTime, weight; };
};

/**
 * @class TspNetwork
 *
 * @brief Time-expanded transport network.
 *
 * This class supports building a transport network, as a set of departure, arrival, stay and transfer events through the
 * stations.
 *
 * @tparam GraphType The graph data structure.
 * @author Andreas Paraskevopoulos
 *
 */
template<class GraphType>
class TspNetwork
{

 public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
    typedef typename GraphType::EdgeDescriptor                      EdgeDescriptor;
    typedef typename GraphType::InEdgeIterator                      InEdgeIterator;
    typedef typename GraphType::SizeType                            SizeType;
    typedef GraphType                                               GraphImpl;

    struct Station;
    typedef typename std::vector<Station> StationContainer;

    struct Vehicle;
    typedef typename std::vector<Vehicle> VehicleContainer;

    /**
     * @brief Constructor. Creates an instance of a (rail, bus, ...) transport network.
     **/
    TspNetwork()
    {}

    /**
     * @brief Clears the network.
     **/
    void clear()
    {
        stations.clear();
        G.clear();
    }

    /**
     * @brief Reserves space for the time-expanded graph.
     * @param numNodes The number of nodes.
     * @param numEdges The number of edges.
     **/
    void reserve( SizeType numNodes, SizeType numEdges)
    {
        G.reserve( numNodes, numEdges);
    }

    /**
     * @brief Returns the number of stations.
     * @return The number of stations.
     **/
    SizeType getNumStations() const
    {
        return stations.size();
    }

    /**
     * @brief Returns the number of time event-nodes.
     * @return The number of nodes.
     **/
    SizeType getNumNodes() const
    {
        return G.getNumNodes();
    }

    /**
     * @brief Returns the number of edges.
     * @return The number of edges.
     **/
    SizeType getNumEdges() const
    {
        return G.getNumEdges();
    }

    /**
     * @brief Adds a station in the network. 
     * @return The created station.
     **/
    Station& addStation()
    {
        stations.resize( stations.size()+1);
        return stations.back();
    }

    /**
     * @brief Adds a vehicle type in the network.
     * @return The vehicle type.
     **/
    Vehicle& addVehicle()
    {
        vehicles.resize( vehicles.size()+1);
        return vehicles.back();
    }

    bool isValid( StationID stId)
    {
        const Station& currSt = stations[stId];
        const NodeIterator stNode = G.getNodeIterator( currSt.stNode);
        const std::vector<StationID>& adjStations = currSt.adjStations;
        bool isValid = true;

        if( stNode->stId != stId)
        {
            std::cout << "\nerror: wrong station node or id (" << stNode->stId << ", " << stId << ")\n";
            isValid = false;
        }

        const std::vector<EdgeIterator>& stEdges = currSt.stEdges;
        assert( ( adjStations.size() + 1) == stEdges.size());

        unsigned int iSt = 0;
        StationID currStId = std::numeric_limits<StationID>::max();
        Distance prevArr = 0, nextArr;

        for( EdgeIterator e = G.beginEdges( stNode), endEdge = G.endEdges( stNode); e != endEdge; ++e)
        {
            NodeIterator depNode = G.target( e);
            assert( depNode->isDeparture());
            assert( depNode->stId == stId);

            EdgeIterator connEdge = G.beginEdges( depNode);
            NodeIterator nextStNode = G.target( connEdge);
            assert( nextStNode->isSwitch());

            if( nextStNode->stId == currStId)
            {
                assert( nextStNode->stId == adjStations[iSt]);
                assert( e == stEdges[iSt]);
                ++iSt;
            }
            else
                prevArr = 0;

            nextArr = depNode->time + connEdge->weight;
            if( nextArr < prevArr)
            {
                std::cout << "\nwarning: unsorted dep nodes: " << prevArr << "(prev) > " << nextArr << "(next)" << std::endl;
                isValid = false;
            }

            prevArr = nextArr;

            ++connEdge;
            if( connEdge != G.endEdges( depNode))
            {
                NodeIterator nextDepNode = G.target( connEdge);
                assert( G.outdeg( depNode) == 2);
                assert( nextDepNode->isDeparture());
                assert( nextDepNode->stId == nextStNode->stId);
                assert( (nextDepNode->time % 1440) == ( (depNode->time + connEdge->weight) % 1440));
            }
        }

        return isValid;
    }

    /**
     * @brief Checks the validity of dynTM network structure.
     **/
    void isValid()
    {
        ProgressStream check_progress( stations.size());
        check_progress.label() << "\tChecking validity";
       
        for( StationID stId=0, numStations=stations.size(); stId<numStations; stId++)
        {
            isValid( stId);
            ++check_progress;
        }
    }

    /**
     * @brief Prints all stations of network.
     **/
    void printStations()
    {
        for( StationID stId=0, numStations=stations.size(); stId<numStations; stId++)
            printStation( stId);
    }

    /**
     * @brief Prints a station of network.
     * @param stId Station Id.
     **/
    void printStation( StationID stId)
    {
        const Station& currSt = stations[stId];
        const NodeIterator stNode = G.getNodeIterator( currSt.stNode);

        const std::vector<EdgeIterator>& stEdges = currSt.stEdges;

        assert( ( currSt.adjStations.size() + 1) == stEdges.size());
        unsigned int limSt = 0;
   
        std::cout << "\nstation(" << stId << ")";
        std::cout << " - transit time:" << currSt.minTransferTime;

        for( EdgeIterator e = G.beginEdges( stNode), endEdge = G.endEdges( stNode); e != endEdge; ++e)
        {
            NodeIterator depNode = G.target( e);
            assert( depNode->isDeparture());

            EdgeIterator connEdge = G.beginEdges( depNode);

            NodeIterator nextStNode = G.target( connEdge);
            assert( nextStNode->isSwitch());

            if( stEdges[limSt] == e)
            {
               std::cout << "\n== TO ST(" << nextStNode->stId << ") ===================================";
               limSt++;
            }

            std::cout << "\nd["<< depNode->stId <<"] (t" << depNode->time << ") - w(" << connEdge->weight << ") -> s[" << nextStNode->stId << "] (t" << depNode->time + connEdge->weight << ")";

            EdgeIterator tsfEdge = connEdge; 
            ++tsfEdge;
            if( tsfEdge != G.endEdges( depNode))
            {
                NodeIterator nextDepNode = G.target( tsfEdge);
                assert( G.outdeg( depNode) == 2);
                assert( nextDepNode->isDeparture());

            std::cout << "\nd["<< depNode->stId <<"] (t" << depNode->time << ") - w(" << tsfEdge->weight << ") -> d[" << nextDepNode->stId << "] (t" << nextDepNode->time << ")";
            }
        }
    }

    /**
     * @brief Prints the geographical bounds of network.
     **/
    void printGeoBounds()
    {
        double minLon, minLat, maxLon, maxLat;
        minLon = minLat = std::numeric_limits<double>::max();
        maxLon = maxLat = std::numeric_limits<double>::min();

        for( StationID stId=0, numStations=stations.size(); stId<numStations; stId++)
        {
            const Station& station = stations[stId];
            if( station.lon < minLon)
                minLon = station.lon;
            if( station.lon > maxLon)
                maxLon = station.lon;
            if( station.lat < minLat)
                minLat = station.lat;
            if( station.lat > maxLat)
                maxLat = station.lat;
        }

        std::cout << "\nRect coord bounds: (" << minLat << "," << minLon << ") (" << maxLat << "," << maxLon << ")\n";
    }

    /**
     * @brief Prints statistics about vehicle types of network.
     **/
    void printVehicleStats()
    {
        std::vector<float> vehFreq( vehicles.size(), 0);
        unsigned int numConnections = 0;

        for( NodeIterator v = G.beginNodes(), endNode = G.endNodes(); v != endNode; ++v)
            if( v->isDeparture())
            {
                vehFreq[v->tpId]++;
                numConnections++;
            }

        std::cout << "\nVehicle statistics:";

        for( unsigned int i=0, vsize=vehFreq.size(); i<vsize; i++)
        {
            vehFreq[i] = round( ( vehFreq[i] / numConnections) * 100.0);
            std::cout << "\n(" << vehFreq[i] << "\%) " << vehicles[i].name;
        }

        std::cout << "\n\n";
    }

    StationContainer stations;
    VehicleContainer vehicles;
    GraphType G;

    struct DepToEdge
    {
        EdgeIterator e;
        Time t;
    };

    //walk path
    struct WalkPath
    {
        StationID stId;
        Distance travelTime;
    };

    //ev path
    struct EVPath
    {
        StationID stId;
        Distance travelTime;
        double consumption;
    };

    //electric vehicles
    struct ElectricVehicle
    {
        ElectricVehicle(): maximumBatteryLevel(0), batteryLevel(0)
        {}
     
        int maximumBatteryLevel;
        int batteryLevel;

        bool testTravel( int consumption)
        {
            if( consumption <= batteryLevel)
                return true;
            else
                return false;
        }

        int travel( int consumption)
        {
            if( consumption > batteryLevel)
                return -1;

            batteryLevel -= consumption;

            return batteryLevel;
        }

        void charge( int depStart, int depEnd)
        {
            int charging = (depEnd - depStart) * 10;
            batteryLevel += charging;

            if( batteryLevel > maximumBatteryLevel)
                batteryLevel = maximumBatteryLevel;
        }

        bool operator <( const ElectricVehicle &ev) const
        {
            if( this->batteryLevel < ev.batteryLevel)
                return true;
            else
                return false;
        }
    };

    //stations
    struct Station
    {
        Station(): id(0), name("undefined"), isEVStation(0), numEVs(1000)
        {
            ev.resize( numEVs);
        }

        //basic info
        union{double x, lon;};
        union{double y, lat;};
        StationID id;
        std::string name;
        Time minTransferTime;

        //station graph info
        NodeDescriptor stNode;
        std::vector<StationID> adjStations;
        std::vector< std::vector<VehicleTypeID> > usedVehicles;
        std::vector< std::vector<std::vector<DepToEdge> > > depIndex;
        //std::vector< std::vector<NodeDescriptor > > vehFirstDep;
        std::vector< std::vector<EdgeIterator> > stVehEdges;
        std::vector<EdgeIterator> stEdges;

        //modes of transportation
        std::vector<WalkPath> walkPaths;
        std::vector<EVPath> evPaths;
        std::vector<ElectricVehicle> ev;

        //electic vehicles
        bool isEVStation;
        unsigned int numEVs;

        /**
         * @brief Inserts a walk path between two stations.
         * @param stId Target station Id.
         * @param travelTime Travel time throught the walk path.
         **/
        void addWalkPath( StationID stId, Distance travelTime)
        {
            WalkPath wk;
            wk.stId = stId;
            wk.travelTime = travelTime;
            walkPaths.push_back( wk);
        }

        /**
         * @brief Inserts an ev path between two stations.
         * @param stId Target station Id.
         * @param travelTime Travel time throught the ev path.
         * @param consumption Consumption level over the ev path.
         **/
        void addEVPath( StationID stId, Distance travelTime, double consumption)
        {
            EVPath ep;
            ep.stId = stId;
            ep.travelTime = travelTime;
            ep.consumption = consumption;
            evPaths.push_back( ep);
            for( unsigned int i=0; i<numEVs; i++)
	      ev[i].batteryLevel += 4*consumption;
        }

        /**
         * @brief Defines type of station's outogoing vehicles.
         * @param G The Graph.
         **/
        inline void addUsedVehicles( GraphType& G)
        {
            setOutgoingVehicles( G, usedVehicles);
        }

        /**
         * @brief Defines type of station's outogoing vehicles.
         * @param G The Graph.
         * @param outVehicles A vector with the vehicle types per adjacent station.
         **/
        void setOutgoingVehicles( GraphType& G, std::vector< std::vector<VehicleTypeID> >& outVehicles)
        {
            outVehicles.clear();
            outVehicles.resize( adjStations.size());

            for( unsigned int i=0; i<adjStations.size(); i++)
            {
                for( unsigned int j=0; j<stVehEdges[i].size()-1; j++)
                {
                    const NodeIterator depNode = G.target( stVehEdges[i][j]);
                    outVehicles[i].push_back( depNode->tpId);
                }

                std::sort( outVehicles[i].begin(), outVehicles[i].end() );
                outVehicles[i].erase( std::unique( outVehicles[i].begin(), outVehicles[i].end() ), outVehicles[i].end() );
            }
        }

        /**
         * @brief Defines type of station's incoming vehicles.
         * @param G The Graph.
         * @param outVehicles A vector with vehicle types.
         **/
        void setIncomingVehicles( GraphType& G, std::vector< std::vector<VehicleTypeID> >& inVehicles)
        {
            inVehicles.clear();
            inVehicles.resize(1);
            const NodeIterator v = G.getNodeIterator( stNode);
            for( InEdgeIterator k = G.beginInEdges( v), endInEdge = G.endInEdges( v); k != endInEdge; ++k)
            {
                const NodeIterator u = G.source( k);
                if( u->isDeparture())
                    inVehicles[0].push_back( u->tpId);
            }
            
            std::sort( inVehicles[0].begin(), inVehicles[0].end() );
            inVehicles[0].erase( std::unique( inVehicles[0].begin(), inVehicles[0].end() ), inVehicles[0].end() );
        }


        /**
         * @brief Prints type of station's outogoing vehicles.
         * @param G The Graph.
         **/
        void printOutcomingVehicles( GraphType& G)
        {
            std::vector< std::vector<VehicleTypeID> > outVehicles;
            setOutgoingVehicles( outVehicles);

            std::cout << "\noutgoing vehicles:";
            for( unsigned int i=0; i<outVehicles.size(); i++)
            {
                std::cout << "{ ";
                for( unsigned int j=0; j<outVehicles[i].size(); j++)
                    std::cout << outVehicles[i][j] << " ";
                std::cout << "}"; 
            }

            std::cout << std::endl;
        }

        /**
         * @brief Prints type of station's incoming vehicles.
         * @param G The Graph.
         **/
        void printIncomingVehicles( GraphType& G)
        {
            std::vector< std::vector<VehicleTypeID> > inVehicles;
            setIncomingVehicles( inVehicles);

            std::cout << "\nincoming vehicles:{ ";
            for( unsigned int i=0; i<inVehicles[0].size(); i++)
                std::cout << inVehicles[0][i] << " ";
            std::cout << "}" << std::endl; 
        }

        /**
         * @brief Adds an adjacent station id.
         * @param id The adjacent station id.
         **/
        inline void addAdjacentStation( StationID id)
        {
            if( std::find( adjStations.begin(), adjStations.end(), id) == adjStations.end())
                adjStations.push_back( id);
        }

        /**
         * @brief Returns the best in arrival time switch-departure edge, based on dep. 
         * @param dep The departure time.
         * @param i Index-i to an adjacent station.
         * @param j Index-j to a vehicle type.
         * @return An Edge.
         **/
        EdgeIterator getEdge( Time dep, unsigned int i, unsigned j)
        {
            const std::vector<DepToEdge>& dix = depIndex[i][j];

            /*std::cout << "\ndix: ";
            for( unsigned int j=0; j<dix.size(); j++)
                std::cout << dix[j].t << " ";
            std::cout << "\n";*/

            //declare low, mid, and high
            int low = 0, high = dix.size()-1, mid;
        
            if( dep <= dix[low].t)
                return dix[low].e;
            else if( dep >= dix[high].t)
                return dix[high].e;

            //binary search loop
            while(1)
            {
                //compute mid
                mid = (low + high + 1) / 2;

                if( dep <= dix[mid].t)
                {
                    if( dep > dix[mid-1].t)
                        return dix[mid].e;
                    else
                        high = mid;
                }

                else
                    low = mid;
            }
        }

        /**
         * @brief Creates station's Οptimal Αrrival full-Ιndex.
         * @param G The Graph.
         **/
        void createDepIndex( const GraphType& G)
        {
            const unsigned int numAdjStations = adjStations.size();
            depIndex.resize( numAdjStations);

            for( unsigned int i=0; i<numAdjStations; ++i)
            {
                const unsigned int numVehTypes = usedVehicles[i].size();
                depIndex[i].resize( numVehTypes);

                for( unsigned int j=0; j<numVehTypes; ++j)
                {
                    int minDepTime = -1;
                    depIndex[i][j].clear();

                    for( EdgeIterator e = stVehEdges[i][j], endEdge = stVehEdges[i][j+1]; e != endEdge; ++e)
                    {
                        const NodeIterator depNode = G.target( e);

                        if( minDepTime < (int) depNode->time)
                        {
                            DepToEdge dte;
                            dte.e = e;
                            dte.t = depNode->time;
                            depIndex[i][j].push_back( dte);
                            minDepTime = depNode->time;
                        }
                    }
                }
            }
        }

        /**
         * @brief Creates station's Οptimal Αrrival sub-Ιndex.
         * @param G The Graph.
         * @param i Index-i to an adjacent station.
         * @param j Index-j to a vehicle type.
         **/
        void createDepIndex( const GraphType& G, unsigned int i, unsigned j)
        {
            int minDepTime = -1;
            depIndex[i][j].clear();

            for( EdgeIterator e = stVehEdges[i][j], endEdge = stVehEdges[i][j+1]; e != endEdge; ++e)
            {
                const NodeIterator depNode = G.target( e);

                if( minDepTime < (int) depNode->time)
                {
                    DepToEdge dte;
                    dte.e = e;
                    dte.t = depNode->time;
                    depIndex[i][j].push_back( dte);
                    minDepTime = depNode->time;
                }
            }
        }

        /**
         * @brief Prints the contain of DepIndex(i,j)
         * @param G The Graph.
         * @param i Index-i to an adjacent station.
         * @param j Index-j to a vehicle type.
         **/
        void printDepIndex( const GraphType& G, unsigned int i, unsigned j)
        {
            const std::vector<DepToEdge>& dix = depIndex[i][j];
            const unsigned int dixSize = dix.size();

            std::cout << "\nDepIndex(" << i << "," << j << "):\n{\n";
            for( unsigned int i=0; i<dixSize; i++)
            {
                EdgeIterator e = dix[i].e;
                NodeIterator depNode = G.target( e);
                EdgeIterator connx = G.beginEdges( depNode);
                NodeIterator stNode = G.target( connx);

                std::cout << "\nDep:" << dix[i].t << " Arr:" << ( depNode->time + connx->weight) << " St:" << stNode->stId << " Vh:" << depNode->tpId;
            }
            std::cout << "\n}\n";
        }

        /**
         * @brief Prints the contain of DepIndex(*,*).
         * @param G The Graph.
         **/
        void printDepIndex( const GraphType& G)
        {
            const unsigned int numAdjStations = adjStations.size();

            for( unsigned int i=0; i<numAdjStations; ++i)
            {
                const unsigned int numVehTypes = usedVehicles[i].size();

                for( unsigned int j=0; j<numVehTypes; ++j)
                    printDepIndex(G, i, j);
            }
        }

    };

    //public transport vehicles
    struct Vehicle
    {
        Vehicle(): typeId(0), name("undefined")
        {}
        
        VehicleTypeID typeId;
        std::string name;
        
        inline bool operator==( const Vehicle& veh) const
        {
            if( typeId == veh.typeId)
                return true;
            else
                return false;
        }
    };

};

#endif //TRANSPORT_NETWORK_H
