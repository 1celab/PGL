#ifndef LANDMARK_GENERATOR_H_
#define LANDMARK_GENERATOR_H_

#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>

//stl containers
#include <string>
#include <vector>
#include <list>
#include <queue>
#include <map>

#include <Utilities/progressBar.h>
#include <Utilities/timer.h>

#include <Structs/Trees/priorityQueue.h>
#include <Algorithms/ShortestPath/Astar/uniDijkstra.h>


/**
 * @class LandmarkGenerator
 *
 * @brief Provides some important landmark selection techniques. A landmark is a 
 * "special" node in the graph used for computing lower bounds in the distances. 
 * It is more preferable to harvest the landmarks in the periphery of the graph. 
 * The shortest path distances to and from every landmark 
 * can be used to produce tight lower bounds for the distance between 
 * every pair of nodes.
 *
 *
 */

template<class GraphType>
class LandmarkGenerator {

public:

    typedef typename GraphType::NodeIterator                        NodeIterator;
    typedef typename GraphType::EdgeIterator                        EdgeIterator;
	typedef typename GraphType::InEdgeIterator                      InEdgeIterator;
    typedef typename GraphType::NodeDescriptor                      NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor                      EdgeDescriptor;
    
    typedef typename GraphType::SizeType                            SizeType;
    typedef unsigned int                                            WeightType;
    typedef unsigned int                                            PointType;
    
    
    /**
     * @brief Creates an instance of LandmarkGenerator.
     * @param A filled graph.
     */
    LandmarkGenerator(GraphType& graph, const std::vector<NodeDescriptor>& ids) : m_graph(graph) , m_ids(ids)
    {}


    void clear()
    {
        m_landmark.clear();
    }


    /**
     * @brief Random landmark selection.
     * Selects a set of random and different landmarks
     * @param The number of landmarks to be harvested in the graph.
     * @return true if the operation is successful; otherwise, false.
     */
    bool randomLandmarkSelection(unsigned int numLandmarks)
    {
        if(numLandmarks > m_graph.getNumNodes())
        {
            std::cerr << "\nerror: number of landmarks > number of nodes\n";
            return false;
        }
        
        else if(numLandmarks == 0)
        {
            std::cerr << "\nwarning: zero number of landmarks\n";
            return false;
        }

        //allocate the necessary memory for the vector of landmarks
        m_landmark.resize(numLandmarks);
        m_numLandmarks = numLandmarks;
        
        ProgressBar randomLandmarks(numLandmarks, std::string("\tHarvesting random landmarks"));

        //select random and different landmarks
        for(int i = 0; i< (int) numLandmarks; i++)
        {
            m_landmark[i] = m_graph.getNodeDescriptor(getRandomNode());
            for(int j = i-1; j>=0; j--)
            {
                //if a new selected landmark is the same with an old one
                if(m_landmark[i] == m_landmark[j])
                {
                    i--; //repeat the selection
                    break; 
                }
            }
      
            ++randomLandmarks;
        }
    
        return true;
    }


    /**
     * @brief Farthest landmark selection.
     * Selects a set of landmarks so that any landmark be furthest away from each other.
     * @param The number of landmarks to be harvested in the graph.
     * @return true if the operation is successful; otherwise, false.
     */
    bool farthestLandmarkSelection(unsigned int numLandmarks)
    {
        if(numLandmarks > m_graph.getNumNodes())
        {
            std::cerr << "\nerror: number of landmarks > number of nodes\n";
            return false;
        }
        
        else if(numLandmarks == 0)
        {
            std::cerr << "\nwarning: zero number of landmarks\n";
            return false;
        }

        //allocate the necessary memory for the vector of landmarks
        m_landmark.resize(numLandmarks);
        m_numLandmarks = 0;

        std::list<EdgeDescriptor> bridge;

        ProgressBar farthestLandmarks(numLandmarks, std::string("\tHarvesting remote landmarks"));
        
        //pick a peripheral node        
        m_landmark[0] = m_graph.getNodeDescriptor(getFarthestNode(getRandomNode()));
        m_numLandmarks++;
        
        ++farthestLandmarks;
        
        //select random and different landmarks
        for(unsigned int i=1; i<numLandmarks; i++)
        {
            //find a node being farthest away from the current set of current selected landmarks
            m_landmark[i] = m_graph.getNodeDescriptor(getFarthestNode(m_graph.getNodeIterator(m_landmark[i-1])));
            m_numLandmarks++;

            //connect the current selected landmarks with zero weight edges (building a bridge with zero weight)
            //TODO mergeNodes(m_landmark[i-1], m_landmark[i], bridge); TODO avoid parallel edges
            //Main idea : i "merge" the old discovered landmarks into one node, so the new landmark be selected farthest away
            //from the current set of landmarks. Restriction : This works good for almost strongly connected graphs
            //such as a full detailed road network    
            ++farthestLandmarks;
        }
        
        //destroy the bridge
        while (!bridge.empty())
        {
            //remove the new added edges
            m_graph.eraseEdge(bridge.front());
            bridge.pop_front();
        }

        return true;
    }


    /**
     * @brief planar landmark selection - light version.
     * Selects a set of landmarks so that any landmark be furthest away from each other and the center of graph.
     * @param The number of landmarks to be harvested in the graph.
     * @return true if the operation is successful; otherwise, false.
     */
    bool planarLandmarkSelection(unsigned int numLandmarks)
    {    
        if(numLandmarks > (unsigned int) m_graph.getNumNodes())
        {
            std::cerr << "\nerror: number of landmarks > number of nodes\n";
            return false;
        }
        
        else if(numLandmarks == 0)
        {
            std::cerr << "\nwarning: zero number of landmarks\n";
            return false;
        }

        //center, max χ and max y coordinate, min x and min y coordinate
        PointType xMax = std::numeric_limits<PointType>::min();
        PointType yMax = std::numeric_limits<PointType>::min();
        
        PointType xMin = std::numeric_limits<PointType>::max();
        PointType yMin = std::numeric_limits<PointType>::max();
                
        //find the max and min that are associated with the nodes of the graph
        for(NodeIterator v = m_graph.beginNodes(), lastNode = m_graph.endNodes(); v != lastNode; ++v)
        {
            if(xMax < v->x)
                xMax = v->x;

            if(yMax < v->y)
                yMax = v->y;

            if(xMin > v->x)
                xMin = v->x;

            if(yMin > v->y)
                yMin = v->y;
        }

        PointType centerX, centerY;

        //compute the coodinates of the center
        centerX = (xMax + xMin)/2;
        centerY = (yMax + yMin)/2;
        
        //find the "middle/center" node of graph
        WeightType minDistance, distanceToCenter = std::numeric_limits<PointType>::max();
        
        NodeIterator middle = m_graph.chooseNode();
        
        //find the max and min that are associated with the nodes of the graph
        for(NodeIterator v = m_graph.beginNodes(), lastNode = m_graph.endNodes(); v != lastNode; ++v)
        {
            minDistance = euclideanDistance(centerX, centerY, v->x, v->y); 
            if(distanceToCenter > minDistance)
            {
                distanceToCenter = minDistance;
                middle = v;
            }
        }

        //allocate the necessary memory for the vector of landmarks
        m_landmark.resize(numLandmarks);
        m_numLandmarks = 0;

        std::list<EdgeDescriptor> bridge;   

        ProgressBar decentralizedLandmarks(numLandmarks, std::string("\tHarvesting remote and decentralized landmarks"));

        //pick a peripheral node
        m_landmark[0] = m_graph.getNodeDescriptor(getDecentralizedNode(middle));
        m_numLandmarks++;

        ++decentralizedLandmarks;

        //merge the middle node with all next selected landmarks. This makes the difference !
        //TODO mergeNodes(m_landmark[0], m_graph.getNodeDescriptor(middle), bridge); avoid parallel edges

        //select random and different landmarks
        for(unsigned int i=1; i<numLandmarks; i++)
        {
            //find a node being farthest away from the current set of the selected landmarks and the middle node
            m_landmark[i] = m_graph.getNodeDescriptor(getDecentralizedNode(middle));
            m_numLandmarks++;

            //connect the current selected landmarks with zero weight edges (building a bridge with zero weight)
            //TODO mergeNodes(m_graph.getNodeDescriptor(middle), m_landmark[i], bridge); avoid parallel edges
            //Main idea : i "merge" the old discovered landmarks into one node, so the new landmark be selected farthest away
            //from the current set of landmarks. Restriction : This works good for almost strongly connected graphs
            //such as a full detailed road network    
            ++decentralizedLandmarks;
        }
        
        //destroy the bridge
        while (!bridge.empty())
        {
            //remove the new added edges
            m_graph.eraseEdge(bridge.front());
            bridge.pop_front();
        }

        return true;

    }
    

    /**
     * @brief Computes and stores for each node the shortest path distances to and from every landmark.
     *
     * a)landmarkFile format :
     * SPdistance(landmark_1, node1) SPdistance(landmark_1, nodeN)
     * SPdistance(landmark_2, node1) SPdistance(landmark_2, nodeN)
     *   ... etc
     *
     * @param The file (path+name) with the precomputed SP distances.
     * @return true if the operation is successful; otherwise, false.
     */
    bool writeLandmarkDistances(const std::string& landmarkFileName)
    {
        std::ofstream landmarkFile(landmarkFileName.c_str());
        
        landmarkFile.exceptions ( std::ofstream::failbit | std::ofstream::badbit );

        if(m_numLandmarks == 0)
        {
            std::cerr << "\nwarning: zero number of landmarks\n";
            return false;
        }

        Timer timer;
    
        try
        {
            //(pact) comment line
            landmarkFile << "% $(number of Landmarks) $(landmarks)\n";

            //write the number of landmarks
            landmarkFile << m_numLandmarks;
        
            //match node object of landmark and node id of landmark
            for(unsigned int i = 0; i < m_numLandmarks; i++)
            {
                for(unsigned int j = 0; j < m_ids.size(); j++)
                    if(m_landmark[i] == m_ids[j])
                    {
                        //write the node ID of each landmark
                        landmarkFile << " " << j;
                        //std::cout << "\n" << m_graph.getNodeIterator(m_ids[j])->y/1000000.0 << " "
                        //          << - (m_graph.getNodeIterator(m_ids[j])->x/1000000.0);

                        break;
                    }
            }

            //(pact) comment line
            landmarkFile << "\n% $SPdistance(landmark,v)\n";

            ProgressBar show_progress(m_numLandmarks, std::string("\tStoring landmarks"));

            //initilization

            //reset the main timestamp
            unsigned int timestamp = 1;

            //reset the timestamp of each node
            for(NodeIterator v = m_graph.beginNodes(), lastnode = m_graph.endNodes(); v != lastnode; ++v)
                v->timestamp = 0;

            Dijkstra<GraphType> computeFromSPdistances (m_graph, &timestamp);
            //BackwardDijkstra<GraphType> computeToSPdistances (m_graph, &timestamp);

            //landmarkFile.precision(std::numeric_limits<WeightType>::digits10-1);
            timer.start();

            for(unsigned int i = 0; i < m_numLandmarks; i++)
            {

                //set zero disrance - preventing "infinite" lower bounds (if the graph is not strongly connected)          
                for(NodeIterator v = m_graph.beginNodes(), lastNode = m_graph.endNodes(); v != lastNode; ++v)
                {
                    v->dist = 0;
                    //v->distBack = 0;
                }

                //A) compute SP distances from any selected landmark to each node 
                computeFromSPdistances.buildTree(m_graph.getNodeIterator(m_landmark[i]));

                //B) compute SP distances from each node to any selected landmark
                //computeToSPdistances.buildTree(m_graph.getNodeIterator(m_landmark[i]));

                //for(NodeIterator v = m_graph.beginNodes(), lastNode = m_graph.endNodes(); v != lastNode; ++v)
                for(unsigned int nodeID = 1; nodeID < m_ids.size(); nodeID++) //safe, keep the right order
                {
                    //write the shortest path distances
                    //landmarkFile << v->dist << " ";
                    //landmarkFile << v->distBack << " "; 
                    landmarkFile << m_graph.getNodeIterator(m_ids[nodeID])->dist << " "; 
                }

                landmarkFile << "\n";

                ++show_progress;
            }

            landmarkFile.close();
            timer.stop();
        }

        catch (std::ofstream::failure e) 
        {
            std::cerr << "\nError writing to file [" << landmarkFileName << "]. "
                      << "\nThe reason :" << e.what() << "n";
            std::ofstream errorLog("IOerror", std::ios::app);
            errorLog << "Error writing to file [" << landmarkFileName << "]. "
                     << "The reason :" << e.what() << "n";
            errorLog.close();
            return false;
        }        

        std::cout << "\tTime spent to compute the landmark shortest paths: " << timer.getElapsedTime() << "sec\n";

        return true;
    }


    /**
     * @brief Loads for each node the precomputed shortest path distances to and from every landmark.
     *
     * a)landmarkFile format :
     * SPdistance(landmark_1, node1) SPdistance(landmark_1, nodeN)
     * SPdistance(landmark_2, node1) SPdistance(landmark_2, nodeN)
     *   ... etc
     *
     * @param The file (path+name) with the precomputed SP distances.
     * @return true if the operation is successful; otherwise, false.
     */
    bool loadLandmarkDistances(const std::string& landmarkFileName)
    {
        std::ifstream landmarkFile (landmarkFileName.c_str());

        landmarkFile.exceptions( std::ifstream::failbit | std::ifstream::badbit);

        if (!landmarkFile)
        {
            std::cerr << "\nerror: unable to read file [" << landmarkFileName << "]\n";
            return false;
        }

        try
        {
            //get and drop the first line (pact : single-line comment)
            landmarkFile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            //read the total number of landmarks contained in the file
            landmarkFile >> m_numLandmarks;

            std::cout << "\tNumber of landmarks: " << m_numLandmarks << "\n"
                      << "\tRequired memory for landmarks: " << (m_graph.getNumNodes() * m_numLandmarks * sizeof(unsigned int) / 1048576.0) 
                      << " Mbytes.\n";

            for(NodeIterator u = m_graph.beginNodes(), end = m_graph.endNodes(); u != end; ++u)
            {
                u->landmark.reserve(m_numLandmarks);
                u->landmark.resize(m_numLandmarks);
            }

            //get and drop the second line (currently we don't need to know the node ID of landmarks)
            landmarkFile.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

            //get and drop the third line (pact : single-line comment)
            landmarkFile.ignore(std::numeric_limits<std::streamsize>::max(), '\n' );

            ProgressBar show_progress(m_numLandmarks, std::string("\tReading landmarks"));

            //read the shortest path distances
            for(unsigned int i=0; i<m_numLandmarks; i++)
            {
                //for(NodeIterator v = m_graph.beginNodes(), lastNode = m_graph.endNodes(); v != lastNode; ++v)
                for(unsigned int nodeID = 1; nodeID < m_ids.size(); nodeID++) //safe, keep the right order
                {
                    //1) distance from any landmark to each node
                    //landmarkFile >> v->landmark[i].distanceFromLandmark;
                    landmarkFile >> m_graph.getNodeIterator(m_ids[nodeID])->landmark[i].distanceFromLandmark;

                    //2)
                    //landmarkFile >> v->landmark[i].distanceToLandmark
                }

            landmarkFile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            
            ++show_progress;
            }

        }

        catch (std::ifstream::failure e) 
        {
            std::cerr << "\nError reading from file [" << landmarkFileName << "]. "
                      << "\nThe reason :" << e.what() << "n";
            std::ofstream errorLog("IOerror", std::ios::app);
            errorLog << "Error reading from file [" << landmarkFileName << "]. "
                     << "The reason :" << e.what() << "n";
            errorLog.close();
            return false;
        }

        landmarkFile.close();

        return true;
    }


 protected:


    /**
     * @brief Returns a random node.
     * @return A random node.
     */
    inline NodeIterator getRandomNode() const
    {
        return m_graph.chooseNode();
    }


    /**
     * @brief Select farthest nodes
     * @param G The graph to search
     * @param root The node to start the search from
     * @return the farthest node
     *
     */
    NodeIterator getFarthestNode(const NodeIterator& root)
    {

        NodeIterator newLandmark, v, lastNode;
        double temp, minDistToLandmark, maxMinDist = 0;

        for(NodeIterator v = m_graph.beginNodes(), lastnode = m_graph.endNodes(); v != lastnode; ++v)
        {
            minDistToLandmark = euclideanDistance( root->x, root->y, v->x, v->y);

            for( unsigned int i = 0; i < m_numLandmarks; i++)
            {

                if( v == m_graph.getNodeIterator(m_landmark[i]))
                {
                    minDistToLandmark = 0;
                    break;
                }

                temp = euclideanDistance( m_graph.getNodeIterator(m_landmark[i])->x, m_graph.getNodeIterator(m_landmark[i])->y, v->x, v->y);

                if( temp < minDistToLandmark)
                    minDistToLandmark = temp;
            }

            if( maxMinDist < minDistToLandmark)
            {
                maxMinDist = minDistToLandmark;
                newLandmark = v;
            }

        }

        return newLandmark;
    }


    /**
     * @brief Select decentralized Nodes
     * @param G The graph to search
     * @param root The node to start the search from
     * @return the farthest and decentralized node
     *
     */
    NodeIterator getDecentralizedNode(const NodeIterator& center)
    {

        NodeIterator newLandmark, v, lastNode;
        double temp, minDistToLandmark, distToCenter, maxMinDist = 0, dINF;

        if(m_numLandmarks != 0)
                dINF = std::numeric_limits<double>::max();
        else
                dINF = 0;

        for(NodeIterator v = m_graph.beginNodes(), lastnode = m_graph.endNodes(); v != lastnode; ++v)
        {

            minDistToLandmark = dINF;
            distToCenter = euclideanDistance( center->x, center->y, v->x, v->y);

            for( unsigned int i = 0; i < m_numLandmarks; i++)
            {

                if( v == m_graph.getNodeIterator(m_landmark[i]))
                {
                    minDistToLandmark = distToCenter = 0;
                    break;
                }

                temp = euclideanDistance( m_graph.getNodeIterator(m_landmark[i])->x, m_graph.getNodeIterator(m_landmark[i])->y, v->x, v->y);

                if( temp < minDistToLandmark)
                    minDistToLandmark = temp;
            }

            //(1)
            if(m_numLandmarks != 0)
                temp = distToCenter * minDistToLandmark;// + distToCenter;
            else
                temp = distToCenter;

            //(2)
            //temp = distToCenter * ( 2 * minDistToLandmark + 1);
            //temp = distToCenter * ( 1 + minDistToLandmark / 2);

            if( maxMinDist < temp)
            {
                maxMinDist = temp;
                newLandmark = v;
            }

        }

        return newLandmark;

    }


    /**
     * @brief (Virtually) Merges two nodes into one. Sets a bridge with zero weight between two nodes.
     * @param source node of the bridge.
     * @param target node of the bridge.
     */
    void mergeNodes(const NodeDescriptor& startNode, const NodeDescriptor& endNode, std::list<EdgeDescriptor>& bridge)
    {
        EdgeDescriptor eD;
        EdgeIterator e;
		InEdgeIterator k;
        
        eD = m_graph.insertEdge( startNode, endNode);
        e = m_graph.getEdgeIterator( eD);
        k = m_graph.getInEdgeIterator( e);
        e->weight = 0;
        k->weight = 0;
        bridge.push_back( eD);

        eD = m_graph.insertEdge( endNode, startNode);
        e = m_graph.getEdgeIterator( eD);
        k = m_graph.getInEdgeIterator( e);
        e->weight = 0;
        k->weight = 0;
        bridge.push_back( eD);
    }

    //graph
    GraphType& m_graph;
    
    //node IDs
    const std::vector<NodeDescriptor> & m_ids;
    
    //current harvested landmarks
    std::vector<NodeDescriptor> m_landmark;
    
    //number of landmarks
    unsigned int m_numLandmarks;

};

#endif
