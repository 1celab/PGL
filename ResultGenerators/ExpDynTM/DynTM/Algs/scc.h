#ifndef SCC_H_
#define SCC_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include <stack>
#include <Utilities/progressBar.h>

struct SccNode
{
    SccNode(): compId(0), isMarked(false)
    {}

    unsigned short compId;
    bool isMarked;
};

/**
 * @class SccProcessor
 *
 * @brief This class implements a processor for the strongly connected components of graph.
 *
 * @author Andreas Paraskevopoulos
 **/
template <typename GraphType>
class SccProcessor
{

 public:

    typedef typename GraphType::NodeIterator                      NodeIterator;
    typedef typename GraphType::EdgeIterator                      EdgeIterator;
    typedef typename GraphType::InEdgeIterator                    InEdgeIterator;
    typedef typename GraphType::NodeDescriptor                    NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor                    EdgeDescriptor;
    typedef typename GraphType::SizeType                          SizeType;

    /**
     * @brief Constructor. Creates an instance of SccProcessor.
     * @param graph The graph to run the algorithm on.
     */
    SccProcessor( GraphType& graph): G(graph)
    {}
    
    /**
     * @brief Computes the strongly connected components and among them the largest one in size.
     **/
    void computeComponents()
    {
        NodeIterator u, v, s, endNode;
        EdgeIterator e;
        InEdgeIterator k, lastInEdge;
        std::stack<NodeIterator> S;
        std::stack<NodeIterator> orderedNodes;
        SizeType compId = 0, maxSize = 0;
        SizeType compSize = 0; 
        maxSCC = 0;

        //initialize DFS
        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        {
            u->compId = 0;
            u->isMarked = false;
        }

        //run Forward DFS
        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        {
            if( u->isMarked == false)
                runIterativeDFS( u, orderedNodes, G);
        }

        //initialize DFS
        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        {
            u->compId = 0;
            u->isMarked = false;
        }

        components.clear();

        //run Backward DFS
        while( !orderedNodes.empty())
        {
            u = orderedNodes.top();
            orderedNodes.pop();

            if( u->isMarked == true)
                continue;

            S.push( u);
            u->isMarked = true;
            u->compId = compId;
            compSize = 1;

            while( !S.empty())
            {
                s = S.top();
                S.pop();

                for( k = G.beginInEdges( s), lastInEdge = G.endInEdges( s); k != lastInEdge; ++k)
                {
                    v = G.source( k);

                    if( v->isMarked == false)
                    {
                        S.push( v);
                        v->isMarked = true;
                        v->compId = compId;
                        compSize++;
                    }
                }
            }

            components.push_back( compSize);

            if( maxSize < compSize)
            {
                maxSize = compSize;
                maxSCC = compId;
            }

            //next strongly connceted component
            compId++;
        }

        std::cout << "\n\tStrongly connected components: " << components.size()
                  << "\n\tLargest strongly connected component size: " << maxSize << "\n";
    }

    /**
     * @brief Returns id of the largest strongly connected component.
     **/
    unsigned int getLargestCompId() const
    {
        return maxSCC;
    }

    /**
     * @brief Returns size of each strongly connected component in graph.
     **/
    const std::vector<unsigned int>& getComponentSizes() const
    {
        return components;
    }

    /**
     * @brief Removes all components, except the largest one in graph.
     **/
    void makeStronglyConnected()
    {
        //compute strongly connected components of graph, O(m+n)
        computeComponents();

        //remove all components except the largest one, O(m+n)
        eraseWeakComponents();
    }

    /**
     * @brief Removes all components, except the largest one in graph.
     **/
    void makeStronglyConnected( std::vector<NodeDescriptor>& ids)
    {
        //compute strongly connected components of graph, O(m+n)
        computeComponents(); 

        //remove node ids of weak components, O(n)
        resetNodeIds( ids);

        //remove all components except the largest one, O(m+n)
        eraseWeakComponents();
    }

    /**
     * @brief Retains only the largest strongly connected component of the original graph.
     **/
    void eraseWeakComponents()
    {
        NodeIterator u, v, endNode;
        EdgeIterator e, endEdge;
        InEdgeIterator k, lastInEdge;

        std::vector<NodeDescriptor> erasedNodes;
        std::vector<EdgeDescriptor> erasedEdges;

        erasedNodes.reserve( G.getNumNodes());
        erasedEdges.reserve( G.getNumEdges());

        //find nodes and edges which are not in the largest strongly connected components, O(m)
        for( u = G.beginNodes(), endNode = G.endNodes(); u != endNode; ++u)
        {
            if( u->compId != maxSCC)
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
                    if( v->compId != maxSCC)
                        erasedEdges.push_back( G.getEdgeDescriptor( e));
                }
            }
        }

        if( !erasedNodes.empty() || !erasedEdges.empty())
        {
            std::cout << "\tExtracting the largest strongly connected component\n";        
            ProgressStream del_progress( erasedEdges.size() + erasedNodes.size());
            del_progress.label() << "\tRemoving " << erasedEdges.size() << " edges and " << erasedNodes.size() << " nodes";

            //erase selected edges
            while( !erasedEdges.empty())
            {
                G.eraseEdge( erasedEdges.back());
                erasedEdges.pop_back();
                ++del_progress;
            }

            //erase selected nodes
            while( !erasedNodes.empty())
            {
                G.eraseNode( erasedNodes.back());
                erasedNodes.pop_back();
                ++del_progress;
            }
        }
    }

 private:

    GraphType& G;
    SizeType maxSCC;
    std::vector<unsigned int> components;

    /**
     * @brief Performs a DFS ordering of nodes by decreasing finishing time.
     * @param root The root node.
     * @param orderedNodes A stack containing the DFS-ordered nodes.
     **/
    void runIterativeDFS( const NodeIterator& root, std::stack<NodeIterator> &orderedNodes, GraphType& G)
    {
        std::stack<NodeIterator> stack;
        stack.push( root);
        root->isMarked = true;
    
        while( stack.empty() == false)
        {
            NodeIterator u = stack.top();

            unsigned int ssize = stack.size();

            for( EdgeIterator e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                NodeIterator v = G.target( e);

                if( v->isMarked == false)
                {
                    v->isMarked = true;
                    stack.push( v);
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
     * @brief Removes node ids of weak components.
     **/
    void resetNodeIds( std::vector<NodeDescriptor>& ids)
    {
        NodeIterator u;
        std::vector<NodeDescriptor> newIds;
        newIds.push_back(0);

        //erase nodes which are not in the largest strongly connected component
        for( unsigned int i=1, size=ids.size(); i<size; i++)
        {
            u = G.getNodeIterator( ids[i]);

            if( u->compId == maxSCC)
                newIds.push_back( ids[i]);
        }

        ids.swap( newIds);
    }

};

#endif
