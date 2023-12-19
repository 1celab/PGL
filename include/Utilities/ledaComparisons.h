#ifndef LEDACOMPARISONS_H
#define LEDACOMPARISONS_H

#include <Utilities/mersenneTwister.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <LEDA/graph/graph_iterator.h>
#include <fstream>
#include <vector>

template< typename GraphType, class Vtype, class Etype>
void graphFromLEDA( GraphType& G, 
                    leda::GRAPH< Vtype, Etype>& ledaGraph, 
                    leda::node_array<typename GraphType::NodeDescriptor>& map)
{
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;

    leda::node ledaNode;
    leda::edge ledaEdge;
    NodeIterator u;
    EdgeIterator e;
    NodeDescriptor uDesc;
    EdgeDescriptor eDesc;

    unsigned int numnodes = ledaGraph.number_of_nodes();
    unsigned int numedges = ledaGraph.number_of_edges();
    unsigned int i = 0;

    std::cout << "\rCopying nodes..." << std::flush;
    for (leda::NodeIt ledaNode(ledaGraph); ledaNode.valid(); ledaNode++)
    {
        /*std::stringstream ss;
        ss << "/home/michai/Projects/THESIS/include/CorrectnessCheckers/leda/out" << i << "n.dot";
        std::string s = ss.str();
        G.printInternalDot( s);*/

        uDesc = G.insertNode();
        map[ledaNode] = uDesc;

        //std::cout << i << " node -> " << uDesc;

        u = G.getNodeIterator( uDesc);

        //std::cout << " -> " << u.getPoolIndex() << std::endl;

        //G.assign( u, i);

        //G.assign( u, ledaGraph.inf(ledaNode));
        if( i % (numnodes/100) == 0)
        {
            std::cout << "\rCopying nodes..." << i / (numnodes/100) << "%" << std::flush;
        }

        ++i;
    }
    std::cout << "\rCopying nodes...done!\n";
   
    i = 0;

    forall_edges( ledaEdge, ledaGraph)
    {
        /*std::stringstream ss;
        ss << "/home/michai/Projects/THESIS/include/CorrectnessCheckers/leda/out" << i << ".dot";
        std::string s = ss.str();
        G.printInternalDot( s);
        */
        NodeIterator u = G.getNodeIterator(map[ leda::source(ledaEdge)]);
        NodeIterator v = G.getNodeIterator(map[ leda::target(ledaEdge)]);
        
        //std::cout << "Inserting " << u.getPoolIndex() << " -> " << v.getPoolIndex() << "..." << std::endl;
        //std::cout << "Inserting " << u.getAddress() << " -> " << v.getAddress() << "..." << std::endl;

        eDesc = G.insertEdge( map[ leda::source(ledaEdge)], map[ leda::target(ledaEdge)]);
                
        e = G.getEdgeIterator( eDesc);

        //std::cout << "Adding " << map[ leda::source(ledaEdge)] << "->" << map[ leda::target(ledaEdge)] << std::endl;
        //std::cout << "Edge is " << e->getDescriptor()->first << "," << e->getDescriptor()->second << std::endl;
       
        //G.assign( e, i);        

        if( i % (numedges/100) == 0)
        {
            std::cout << "\rCopying edges..." << i / (numedges/100) << "%" << std::flush;
        }

        ++i;
    }
    std::cout << "\rCopying edges...done!\n";
}


template< typename GraphType, class Vtype, class Etype>
bool compareGraphs( GraphType& G, 
                    leda::GRAPH< Vtype, Etype>& ledaGraph, 
                    leda::node_array<typename GraphType::NodeDescriptor>& map)
{

    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;

    leda::edge ledaEdge;
    EdgeIterator e,end;
    bool found;

    unsigned int numedges = ledaGraph.number_of_edges();
    unsigned int i = 0;

    G.addNodeProperty("marked",false);
    G.addEdgeProperty("marked",false);

    forall_edges( ledaEdge, ledaGraph)
    {
        NodeIterator u = G.getNodeIterator(map[ leda::source(ledaEdge)]);
        NodeIterator v = G.getNodeIterator(map[ leda::target(ledaEdge)]);
        
        //std::cout << "Checking if edge " << u.getPoolIndex() << " -> " << v.getPoolIndex() << " exists..." << std::endl;

        found = false;
        end = G.endEdges(u);
        for( e = G.beginEdges(u); e != end; ++e)
        {       
            if ( v == G.opposite( u, e))
            {       
                G.setProperty( e, "marked", true);
                found = true;
                break;            
            }
        }
        
        //std::cout << "\rChecking LEDA edges..." << i  << "/" << numedges << std::flush;
        
        if( !found) return false;

        if( i % (numedges/100) == 0)
        {
            std::cout << "\rChecking edges..." << i / (numedges/100) << "%" << std::flush;
        }

        ++i;
    }
    std::cout << "\rChecking LEDA edges...done!\n";

    i = 0;
    NodeIterator lastNode = G.endNodes();
    for( NodeIterator u = G.beginNodes(); u != lastNode; ++u)
    {
        end = G.endEdges(u);
        for( e = G.beginEdges(u); e != end; ++e)
        {
            if( G.getProperty( e, "marked") == false) 
            {
                
                return false;
            }
            if( i % (numedges/100) == 0)
            {
                std::cout << "\rChecking marked edges..." << i / (numedges/100) << "%" << std::flush;
            }

            ++i;
        }
    }

    std::cout << "\rChecking marked edges...done!\nGraphs are the same!\n";
    G.removeNodeProperty("marked");
    G.removeEdgeProperty("marked");

    return true;
}




template< typename GraphType, class Vtype, class Etype>
void readGraphwin( GraphType& G, const std::string& filename)
{
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    std::ifstream in(filename.c_str());
    SizeType numnodes, numedges, source, target;
    std::vector<NodeDescriptor> V;

    for( unsigned int i = 0; i < 4; i++)
    {
        in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    
    in >> numnodes;
    std::cout << "\rReading nodes..." << std::flush;
    for( SizeType i = 0; i < numnodes + 1; i++)
    {
        /*std::stringstream ss;
        ss << "/home/michai/Projects/THESIS/include/CorrectnessCheckers/leda/out" << i << "n.dot";
        std::string s = ss.str();
        G.printInternalDot( s);*/

        V.push_back(G.insertNode());
        in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
        /*if( i % (numnodes/100) == 0)
        {
            std::cout << "\rReading nodes..." << i / (numnodes/100) << "%" << std::flush;
        }*/
    }
    std::cout << "\rReading nodes...done!\n";
    
    
    in >> numedges;
    std::cout << "\rReading edges..." << std::flush;
    for( SizeType i = 0; i < numedges; i++)
    {
        /*std::stringstream ss;
        ss << "/home/michai/Projects/THESIS/include/CorrectnessCheckers/leda/out" << i << ".dot";
        std::string s = ss.str();
        G.printInternalDot( s);*/

        in >> source;
        in >> target;
        G.insertEdge( V[source], V[target]);
        in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
        if( i % (numedges/100) == 0)
        {
            std::cout << "\rReading edges..." << i / (numedges/100) << "%" << std::flush;
        }
    }
    std::cout << "\rReading edges...done!\n";
    in.close();
}


#endif //LEDACOMPARISONS_H
