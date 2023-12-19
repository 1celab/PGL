#ifndef TIME_EXPANDED_DELAY_PROPAGATOR_H
#define TIME_EXPANDED_DELAY_PROPAGATOR_H

#include "../Structs/definitions.h"
#include <vector>

/**
 * @class DelayPropagator
 *
 * @brief Performs the necessary changes to apply the delay of a vehicle.
 *
 * This class supports the updating of RTE graph.
 *
 * @tparam TspNetwork The grah data structure and the time-expanded-model which are used.
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
    typedef typename TspNetwork::EdgeDescriptor                    EdgeDescriptor;


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
        NodeIterator depNode = G.endNodes();

        const Station &station = stations[stationId];
        if( station.numDepEvents != 0)
        {
            depNode = G.getNodeIterator( station.firstDepNode);
            
            if( station.numDepEvents > 1)
                for( unsigned int j = 0; j < station.numDepEvents; j++)
                {
                    if( depNode->time >= startTime)
                        break;

                    depNode = G.target( --G.endEdges( depNode));
                }
        }

        return depNode;
    }

    void update( const StationID stationId, NodeIterator depNode, const Distance delay)
    {
        ++(*m_timestamp);
        updatedEdges = 0;
        updatedStations = 0;

        if( delay == 0)
            return;

        assert( depNode->isDeparture());

        EdgeIterator e = G.beginEdges( depNode);
        NodeIterator arrNode = G.target( e);

        //propagate the delay over the invloved stations, from which the delayed vehicle passes
        propagateDelay( arrNode, delay);
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

    void propagateDelay( const NodeIterator& root, const Distance& delay)
    {
        EdgeIterator e, endEdge, dArrToArr, dArrToDep, dDepToArr;
        InEdgeIterator k, endInEdge;
        NodeIterator u, v, dArrNode, depNode, arrNode, 
        cDepNode = G.endNodes(), dDepNode = G.endNodes(), nextdArrNode;

        //get the node, which corresponds to the delayed arrival time event
        nextdArrNode = root;
        root->timestamp = (*m_timestamp);
        assert( root->isArrival());

        //update the weight of the delayed connection dep->arr (and arr->arr)
        for( InEdgeIterator k = G.beginInEdges( root), endInEdge = G.endInEdges( root); k != endInEdge; ++k)
        {
            u = G.source( k);
            u->timestamp = (*m_timestamp);
            e = G.getEdgeIterator( k);
            if( ( e->weight + delay) < 0) {std::cout << "Delay(" << delay << ") is not in valid range\n"; exit(-1);}

            e->weight += delay;
            ++updatedEdges;
        }

        while( nextdArrNode != G.endNodes())
        {
            updatedStations++;
            dArrNode = nextdArrNode;
            dArrNode->time = ( dArrNode->time + delay) % 1440;
            assert( dArrNode->isArrival());

            //get the station where the delayed vehicle arrives on
            const Station& station = stations[dArrNode->stId];

            //there are no departure events to proceed
            if( station.numDepEvents == 0)
                break;

            //vehicle's trip ends
            else if( G.outdeg( dArrNode) == 1)
            {
                //arrival-departure edge (transit)
                dArrToDep = G.beginEdges( dArrNode);

                //get the departure event which is connected with the delayed arrival event of the belated-vehicle
                cDepNode = G.target( dArrToDep);
                assert( cDepNode->isDeparture());

                fixArrToDep( dArrNode, dArrToDep, cDepNode);
                break;
            }

            //vehicle's trip continues
            else
            {
                for( e = G.beginEdges( dArrNode), endEdge = G.endEdges( dArrNode); e != endEdge; e++)
                {
                    v = G.target( e);

                    if( v->isDeparture())
                    {
                        dArrToDep = e;
                        cDepNode = v;
                    }

                    else
                    {
                        dArrToArr = e;
                        nextdArrNode = v;
                    }
                }

                //get the departure event which is connected to the next delayed arrival event
                for( k = G.beginInEdges( nextdArrNode), endInEdge = G.endInEdges( nextdArrNode); k != endInEdge; ++k)
                {
                    u = G.source( k);

                    if( u->isDeparture())
                    {
                        dDepNode = u;
                        break;
                    }
                }

                assert( k != endInEdge);

                //mark the next delayed arrival event (first visit)
                if( nextdArrNode->timestamp != (*m_timestamp))
                    nextdArrNode->timestamp = (*m_timestamp);

                //cycle route
                else
                {
                    dArrToArr->weight -= delay;
                    assert( (dArrNode->time + dArrToArr->weight) % 1400 == nextdArrNode->time);
                    nextdArrNode = G.endNodes();
                    dDepNode = G.endNodes();
                }
            }
                
            //reconfigure the topology
            //update-cases:
            //a) the arrival-departure edge of the delayed arrival event
            //b) the (one or more) arrival-departure edges of the delayed departure event
            //c) the next, possibly invalid, arrival-departure edges, above the (replaced) delayed departure event

            if( cDepNode == dDepNode)
            {
                //if the belated vehicle continues its route from this station to another one
                //sort the departure time events applying the delay, and also perform the (b) and (c) updates
                sortTimeEvents( dDepNode, delay, dArrNode); 
            }

            else
            {
                //if the belated vehicle continues its route from this station to another one
                //sort the departure time events applying the delay, and also perform the (b) and (c) updates
                sortTimeEvents( dDepNode, delay, dArrNode);
                fixArrToDep( dArrNode, dArrToDep, cDepNode);
            }   
        }
    }

    void fixArrToDep( const NodeIterator arrNode, EdgeIterator e, const NodeIterator arrDepHead)
    {
        const Station& station = stations[arrNode->stId];
        const NodeIterator& firstDepNode = G.getNodeIterator( station.firstDepNode);
        const NodeIterator& lastDepNode = getPredDepNode( firstDepNode);
        NodeIterator depNode;

        updatedEdges++;

        //next day: all departure events
        if( lastDepNode->time < ( arrNode->time + station.transitTime))
        {
            depNode = firstDepNode;
            const Time offset = 1440 - arrNode->time;
            for( unsigned int i = 0; i < station.numDepEvents; i++)
            {
                Time depTime = depNode->time % 1440;
                if( ( offset + depTime) >= station.transitTime)
                {
                    e->weight = offset + depTime;
                    assert( ( arrNode->time + e->weight) % 1440 == depTime);
                    
                    if( depNode != arrDepHead)
                        G.changeHead( e, depNode);

                    break;
                }

                depNode = getSuccDepNode( depNode);
            }
        }

        //current day: current or previous departure events
        else if( arrDepHead->time >= ( arrNode->time + station.transitTime))
        {
            NodeIterator earliestDepNode = arrDepHead;
            depNode = getPredDepNode( arrDepHead);

            do{

                if( depNode->time >= (arrNode->time + station.transitTime))
                    earliestDepNode = depNode;
                else
                    break;

                depNode = getPredDepNode( depNode);
            }
            while( depNode != lastDepNode);

            assert( earliestDepNode->time >= arrNode->time);
            e->weight = earliestDepNode->time - arrNode->time;

            if( earliestDepNode != arrDepHead)
               G.changeHead( e, earliestDepNode);
        }

        //current day: next departure events
        else
        {     
            depNode = getSuccDepNode( arrDepHead);

            do{

                if( depNode->time >= ( arrNode->time + station.transitTime))
                {
                    assert( depNode->time >= arrNode->time);
                    e->weight = depNode->time - arrNode->time;
                    G.changeHead( e, depNode);
                    break;
                }

                depNode = getSuccDepNode( depNode);

            }
            while( depNode != firstDepNode);
        }
    }


    void sortTimeEvents( NodeIterator dDepNode, Time delay, const NodeIterator& dArrNode)
    {
        NodeIterator u, v;
        InEdgeIterator k, endInEdge;
        EdgeIterator e, e1, e2, e3;
        Distance tempWeight;
        bool movedUp;

        //the old predecessor and successor departure nodes (before the update)
        //of the delayed departure event-node
        NodeIterator pred = getPredDepNode( dDepNode);
        NodeIterator succ = getSuccDepNode( dDepNode);

        //predecessor and successor departure nodes (after the update)
        NodeIterator newPred = pred, newSucc = succ;

        Station& station = stations[dDepNode->stId];
        NodeIterator firstDepNode = G.getNodeIterator( station.firstDepNode);
        NodeIterator lastDepNode = getPredDepNode( firstDepNode);

        assert( firstDepNode->isDeparture());

        //update the departure event of the delayed train in the station
        dDepNode->time += delay;

        if( delay < 0)
            movedUp = false;
        else
            movedUp = true;
    
        //search backward the new position of the affected depNode
        if( movedUp == false)
        {
            assert( dDepNode->time >= 0);
            
            if( dDepNode != firstDepNode)
            {
                //move backward
                while( newPred->time > dDepNode->time)
                {            
                    newSucc = newPred;
                    newPred = getPredDepNode( newPred);

                    if( newPred == lastDepNode)
                        break;
                }

                //replace the first departure if depNode is an earliest time event
                if( firstDepNode->time > dDepNode->time)
                    station.firstDepNode = G.getNodeDescriptor( dDepNode);
            }
        }
            
        //search forward the new position of the delayed arrival event (pact: delay < 1440)
        else
        {
            if( dDepNode != lastDepNode)
            {
                //move forward
                while( newSucc->time < dDepNode->time)
                {
                    newPred = newSucc;
                    newSucc = getSuccDepNode( newSucc);

                    if( newSucc == firstDepNode)
                        break;
                }
            }
        }

        assert( pred->isDeparture());
        assert( newPred->isDeparture());
        assert( succ->isDeparture());
        assert( newSucc->isDeparture());

        //B) switch the head of the outdated edges, with the correct departure node

        //if the topology needs change
        if( newPred != pred)
        {
            //A) set new weights to the re-wired edges

            //e1: predDepNode -> movedDepNode -> succDepNode (before)
            //e1: predDepNode -> succDepNode (after)
            e1 = G.getEdgeIterator( pred, dDepNode);
            assert( G.target(e1) == dDepNode);

            if( succ->time >= pred->time)
                e1->weight = succ->time - pred->time; 
            else
                e1->weight = (1440 - pred->time) + succ->time;

            //e2: movedDepNode -> succDepNode (before)
            //e2: movedDepNode -> newSuccDepNode (after)
            e2 = G.getEdgeIterator( dDepNode, succ);
            assert( G.target(e2) == succ);
            tempWeight = e2->weight;

            if( newSucc->time >= dDepNode->time)
                e2->weight = newSucc->time - dDepNode->time; 
            else
                e2->weight = (1440 - dDepNode->time) + newSucc->time;

            //e3: newPredDepNode -> newSuccDepNode (before)
            //e3: newPredDepNode -> movedDepNode (after)
            e3 = G.getEdgeIterator( newPred, newSucc);
            assert( G.target(e3) == newSucc);


            if( dDepNode->time >= newPred->time)
                e3->weight = dDepNode->time - newPred->time; 
            else
                e3->weight = (1440 - newPred->time) + dDepNode->time; 

            assert( e1->weight < 2880);
            assert( e2->weight < 2880);
            assert( e3->weight < 2880);

            EdgeDescriptor eD1, eD2, eD3;
            eD1 = G.getEdgeDescriptor( e1);
            eD2 = G.getEdgeDescriptor( e2);
            eD3 = G.getEdgeDescriptor( e3);

            e1 = G.getEdgeIterator( eD1);
            G.changeHead( e1, succ);
            e2 = G.getEdgeIterator( eD2);
            G.changeHead( e2, newSucc);
            e3 = G.getEdgeIterator( eD3);
            G.changeHead( e3, dDepNode);
            updatedEdges += 3;

            //(b) update
            //if the delayed departure has been moved then all arrival-departure edges are invalid
            //scan the arrival-departure edges

            //safe iterator
            std::vector<NodeDescriptor> uNodes;
            std::vector<EdgeDescriptor> uEdges;
            for( k = G.beginInEdges( dDepNode), endInEdge = G.endInEdges( dDepNode); k != endInEdge; ++k)
            {
                u = G.source( k);
                e = G.getEdgeIterator( k);
                uNodes.push_back( G.getNodeDescriptor( u));
                uEdges.push_back( G.getEdgeDescriptor( e));
            }

            for( unsigned int i=0; i<uNodes.size(); i++)
            {
                u = G.getNodeIterator( uNodes[i]);
                e = G.getEdgeIterator( uEdges[i]);

                if( u->isArrival())
                {
                    if( u != dArrNode)
                    {
                        e->weight += tempWeight;
                        G.changeHead( e, succ);
                    }

                    else
                        fixArrToDep( dArrNode, e, dDepNode);
                }
            }
            //safe iterator

            /*for( k = G.beginInEdges( dDepNode), endInEdge = G.endInEdges( dDepNode); k != endInEdge;)
            {
                u = G.source( k);
                e = G.getEdgeIterator( k);
                ++k;

                if( u->isArrival())
                {
                    if( u != dArrNode)
                    {
                        e->weight += tempWeight;
                        G.changeHead( e, succ);
                    }

                    else
                        fixArrToDep( dArrNode, e, dDepNode);
                }
            }*/
        }

        //(b) update
        //if the order of the departure events is not changed after the delay
        else 
        {
            e1 = G.getEdgeIterator( pred, dDepNode);

            if( dDepNode->time >= pred->time)
                e1->weight = dDepNode->time - pred->time; 
            else
                e1->weight = (1440 - pred->time) + dDepNode->time;

            e2 = G.getEdgeIterator( dDepNode, succ);

            if( succ->time >= dDepNode->time)
                e2->weight = succ->time - dDepNode->time; 
            else
                e2->weight = (1440 - dDepNode->time) + succ->time;

            assert( e1->weight < 1000000000);
            assert( e2->weight < 1000000000);

            //safe iterator
            std::vector<NodeDescriptor> uNodes;
            std::vector<EdgeDescriptor> uEdges;
            for( k = G.beginInEdges( dDepNode), endInEdge = G.endInEdges( dDepNode); k != endInEdge; ++k)
            {
                u = G.source( k);
                e = G.getEdgeIterator( k);
                uNodes.push_back( G.getNodeDescriptor( u));
                uEdges.push_back( G.getEdgeDescriptor( e));
            }

            for( unsigned int i=0; i<uNodes.size(); i++)
            {
                u = G.getNodeIterator( uNodes[i]);
                e = G.getEdgeIterator( uEdges[i]);

                //when departure time of dDepNode is increased(moveUp)/decreased(moveDown)
                //fix the arrival-departure edges which have as head the dDepNode
                if( u->isArrival())
                    fixArrToDep( u, e, dDepNode);

            }
            //safe iterator

            /*for( k = G.beginInEdges( dDepNode), endInEdge = G.endInEdges( dDepNode); k != endInEdge;)
            {
                u = G.source( k);
                e = G.getEdgeIterator( k);
                ++k;

                //when departure time of dDepNode is increased(moveUp)/decreased(moveDown)
                //fix the arrival-departure edges which have as head the dDepNode
                if( u->isArrival())
                    fixArrToDep( u, e, dDepNode);
            }*/
        }

        //(c) update 
        //check the next, possibly invalid, departure-arrival edges after the (moved up) updated departure node
        if( movedUp == true)
        {
            succ = newSucc;
            do{

                //safe iterator
                std::vector<NodeDescriptor> uNodes;
                std::vector<EdgeDescriptor> uEdges;
                for( k = G.beginInEdges( succ), endInEdge = G.endInEdges( succ); k != endInEdge; ++k)
                {
                    u = G.source( k);
                    e = G.getEdgeIterator( k);
                    uNodes.push_back( G.getNodeDescriptor( u));
                    uEdges.push_back( G.getEdgeDescriptor( e));
                }

                for( unsigned int i=0; i<uNodes.size(); i++)
                {
                    u = G.getNodeIterator( uNodes[i]);
                    e = G.getEdgeIterator( uEdges[i]);

                    //when departure time of dDepNode is increased(moveUp)/decreased(moveDown)
                    //fix the arrival-departure edges which have as head the dDepNode
                    if( u->isArrival())
                        fixArrToDep( u, e, succ);

                }
                //safe iterator

                /*for( k = G.beginInEdges( succ), endInEdge = G.endInEdges( succ); k != endInEdge;)
                {
                    u = G.source( k);
                    e = G.getEdgeIterator( k);
                    ++k;

                    if( u->isArrival())
                        fixArrToDep( u, e, succ);
                }*/

                succ = getSuccDepNode( succ);

            } while( succ != newSucc && succ->time == newSucc->time);
        }

        //check the previous, possibly invalid, departure-arrival edges after the (moved down) updated departure node
        else
        {
            pred = newPred;
            do{

                //safe iterator
                std::vector<NodeDescriptor> uNodes;
                std::vector<EdgeDescriptor> uEdges;
                for( k = G.beginInEdges( pred), endInEdge = G.endInEdges( pred); k != endInEdge; ++k)
                {
                    u = G.source( k);
                    e = G.getEdgeIterator( k);
                    uNodes.push_back( G.getNodeDescriptor( u));
                    uEdges.push_back( G.getEdgeDescriptor( e));
                }

                for( unsigned int i=0; i<uNodes.size(); i++)
                {
                    u = G.getNodeIterator( uNodes[i]);
                    e = G.getEdgeIterator( uEdges[i]);

                    //when departure time of dDepNode is increased(moveUp)/decreased(moveDown)
                    //fix the arrival-departure edges which have as head the dDepNode
                    if( u->isArrival())
                        fixArrToDep( u, e, pred);
                }
                //safe iterator

                /*for( k = G.beginInEdges( pred), endInEdge = G.endInEdges( pred); k != endInEdge;)
                {
                    u = G.source( k);
                    e = G.getEdgeIterator( k);
                    ++k;

                    if( u->isArrival())
                        fixArrToDep( u, e, pred);
                }*/

                pred = getSuccDepNode( pred);

            } while( pred != newPred && pred->time == newPred->time);
        }
    }


    /**
     * @brief Returns the predecessor departure node through the dep-chain.
     * @param depNode The departure node.
     * @return The predecessor departure node.
     */    
    inline NodeIterator getPredDepNode( const NodeIterator& depNode)
    {
        //get the predecessor departure event
        for( InEdgeIterator k = G.beginInEdges( depNode), endInEdge = G.endInEdges( depNode); k != endInEdge; ++k)
        {
            NodeIterator u = G.source( k);

            if( u->isDeparture())
                return u;
        }

        assert( 1);
        return G.endNodes();
    }


    /**
     * @brief Returns the successor departure node through the dep-chain.
     * @param depNode The departure node.
     * @return The successor departure node.
     */   
    inline NodeIterator getSuccDepNode( const NodeIterator& depNode)
    {
        const NodeIterator v = G.target( ( --G.endEdges( depNode)));
        assert( v->isDeparture());
        return v;
    }


    GraphType& G;
    StationContainer& stations;
    TTL* m_timestamp;

    unsigned int updatedStations;
    unsigned int updatedEdges;
};

#endif //TIME_EXPANDED_DELAY_PROPAGATOR_H
