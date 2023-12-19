#ifndef DYNTM_DELAY_PROPAGATOR_H
#define DYNTM_DELAY_PROPAGATOR_H

#include "../Structs/definitions.h"
#include <algorithm>
#include <iterator>
#include <vector>

namespace DynTM
{

/**
 * @class DelayPropagator
 *
 * @brief Performs the necessary changes to apply the delay of a vehicle.
 *
 * This class supports the updating of DTM graph.
 *
 * @tparam TspNetwork The grah data structure and the dynTM time-expanded-model which are used.
 * @author Andreas Paraskevopoulos
 *
 */
template<class TspNetwork>
class DelayPropagator
{

 public:

    typedef typename TspNetwork::Station                           Station;
    typedef typename TspNetwork::StationContainer                  StationContainer;
    typedef typename TspNetwork::GraphImpl                         GraphType;
    typedef typename TspNetwork::NodeIterator                      NodeIterator;
    typedef typename TspNetwork::EdgeIterator                      EdgeIterator;
    typedef typename TspNetwork::InEdgeIterator                    InEdgeIterator;
    typedef typename TspNetwork::NodeDescriptor                    NodeDescriptor;


    /**
     * @brief Constructor. Creates an instance of DelayAdjuster.
     * @param graph The graph to run the algorithm on.
     * @param stationContainer A container with the stations.
     * @param timestamp An address containing a timestamp. A timestamp must be given in order 
     * to check whether a node is visited or not.
     */
    DelayPropagator( GraphType& graph, StationContainer& stationContainer, TTL* timestamp): 
    G(graph), stations(stationContainer), m_timestamp(timestamp)
    {}

    NodeIterator getAffectedDepNode( const StationID stationId, const Time startTime)
    {        
        NodeIterator stNode, depNode = G.endNodes();

        stNode = G.getNodeIterator( stations[stationId].stNode);

        for( EdgeIterator e = G.beginEdges( stNode), endEdge = G.endEdges( stNode); e != endEdge; ++e)
        {
            depNode = G.target( e);
            assert( depNode->isDeparture());

            if( depNode->time >= startTime)
                break;
        }

        return depNode;
    }

    void update( const StationID stationId, NodeIterator depNode, const Distance delay)
    {
        ++(*m_timestamp);
        updatedEdges = 0;
        updatedStations = 1;

        assert( depNode->isDeparture());

        NodeIterator v, nextDepNode;
        EdgeIterator e, endEdge;

        //mark the first affected departure node as visited
        depNode->timestamp = (*m_timestamp);

        nextDepNode = G.endNodes();
        for( e = G.beginEdges( depNode), endEdge = G.endEdges( depNode); e != endEdge; ++e)
        {
            //increase the weight of outgoing connections of the first affected departure node
            updatedEdges++;
            e->weight += delay;

            v = G.target( e);
            if( v->isDeparture())
            {
                //get the next affected dep node of the same vehicle via a departure-departure edge
                nextDepNode = v;
                assert( depNode->vhId == nextDepNode->vhId);
            }
        }

        //mantain the ascending arrival time order of dep nodes
        reorder( depNode, delay);        

        depNode = nextDepNode;
        while( depNode != G.endNodes())
        {
            //mark the next dep node as visited
            depNode->timestamp = (*m_timestamp);
            //update departure time of affected dep node
            depNode->time += delay;
            updatedStations++;
            //mantain the ascending arrival time order of dep nodes
            reorder( depNode, delay);

            //if the delayed vehicle continues its travel to another station, then get the next affected departure node
            nextDepNode = G.endNodes();
            for( EdgeIterator e = G.beginEdges( depNode), endEdge = G.endEdges( depNode); e != endEdge; ++e)
            {
                if( v->isDeparture())
                {
                    //stop if there is a cycle-route
                    if( v->timestamp == (*m_timestamp))
                    {
                        updatedEdges++;
                        e->weight -= delay;
                    }

                    else
                    {
                        //get the next affected dep node of the same vehicle via a departure-departure edge
                        nextDepNode = v;
                        assert( depNode->vhId == nextDepNode->vhId);
                    }
                }
            }

            depNode = nextDepNode;
        }
    }

    
    /**
     * @brief Returns the number of the affected stations due to the delay.
     */
    unsigned int getNumUpdatedStations()
    {
        return updatedStations;
    }

    /**
     * @brief Returns the number of the updated edges due to the delay.
     */
    unsigned int getNumUpdatedEdges()
    {
        return updatedEdges;
    }

 private:

    GraphType& G;
    StationContainer& stations;
    TTL* m_timestamp;

    Time delay;
    unsigned int updatedStations;
    unsigned int updatedEdges;

    void reorder( const NodeIterator& depNode, const Distance& delay)
    {
        NodeIterator nextDepNode, stNode, nextStNode;
        Distance currArrTime, nextArrTime;
        EdgeIterator e, etemp, eNext, endEdge, connx;

        //station
        Station& currSt = stations[depNode->stId];

        //station node
        stNode = G.getNodeIterator( currSt.stNode);

        //if num dep nodes <=1, then there is no need for swapping 
        if( G.outdeg( stNode) <= 1)
            return;

        //get the correspoding switch-departure edge
        for( InEdgeIterator k = G.beginInEdges( depNode), endInEdge = G.endInEdges( depNode); k != endInEdge; ++k)
        {
            if( G.source( k) == stNode)
            {
                e = G.getEdgeIterator( k);
                break;
            }
        }

        //get next station
        connx = G.beginEdges( depNode);
        nextStNode = G.target( connx);
        assert( nextStNode->isSwitch()); 

        const std::vector<StationID>& adjStations = currSt.adjStations;
        const std::vector< std::vector<EdgeIterator> >& stVehEdges = currSt.stVehEdges;
        std::vector< std::vector<VehicleTypeID> >& usedVehicles = currSt.usedVehicles;

        const int iSt = std::distance( adjStations.begin(), std::lower_bound( adjStations.begin(), adjStations.end(), nextStNode->stId));
        const int jVh = std::distance( usedVehicles[iSt].begin(), std::lower_bound( usedVehicles[iSt].begin(), usedVehicles[iSt].end(), depNode->tpId));

        //get the new arrival time
        assert( nextStNode->isSwitch());
        currArrTime = depNode->time + connx->weight;
        eNext = e;    

        //update the order of the outgoing edges of station node
        //and maintain the ascending arrival time order of dep nodes

        //forward-increase
        if( delay > 0)
        {
            ++eNext;
            endEdge = stVehEdges[iSt][jVh+1];
    
            while( eNext != endEdge)
            {
                nextDepNode = G.target( eNext);
                connx = G.beginEdges( nextDepNode);
                nextArrTime = nextDepNode->time + connx->weight;

                assert( nextDepNode->isDeparture());
                assert( G.target( connx)->isSwitch());

                if( nextArrTime < currArrTime)
                {
                    G.swapEdges( e, eNext);
                    updatedEdges++;
                }

                else
                    break;

                e = eNext;
                ++eNext;
            }
        }

        else
        {
            --eNext;
            endEdge = stVehEdges[iSt][jVh];
            --endEdge;

            while( eNext != endEdge)
            {
                nextDepNode = G.target( eNext);
                connx = G.beginEdges( nextDepNode);
                nextArrTime = nextDepNode->time + connx->weight;

                assert( nextDepNode->isDeparture());
                assert( G.target( connx)->isSwitch());

                if( nextArrTime > currArrTime)
                {
                    G.swapEdges( e, eNext);
                    updatedEdges++;
                }

                else
                    break;

                e = eNext;
                --eNext;
            }
        }

        //update the affected dep index
        currSt.createDepIndex( G, iSt, jVh);
    }
};

};

#endif
