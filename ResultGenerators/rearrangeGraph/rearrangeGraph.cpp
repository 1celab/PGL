#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <Algorithms/basicGraphAlgorithms.h>
#include <Algorithms/ShortestPath/dijkstra.h>
#include <Algorithms/ShortestPath/aStarDijkstra.h>
#include <Algorithms/ShortestPath/bidirectionalDijkstra.h>
#include <boost/utility/enable_if.hpp>
#include <iomanip>
#include <Utilities/geographic.h>
#include <Utilities/graphGenerators.h>
#include <Utilities/timer.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
std::string basePath = "/home/michai/Projects/Graphs/DIMACS10/";


struct node: DefaultGraphItem
{
	node():x(0),y(0)
	{	
	}

    void print( std::ofstream& out)
    {
        return;
    }

    void setProperty( const std::string& name, const std::string& value)
    {
        DefaultGraphItem::setProperty( name, value);
        if( !name.compare("x"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>x;
        }
        if( !name.compare("y"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>y;
        }
    }

    void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "x" << valueDelimiter << x << propertyDelimiter;
        out << "y" << valueDelimiter << y << propertyDelimiter;
    }
    
    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"x\":" << x << "," << "\"y\":" << y << "";
    }

    unsigned int x,y;
    bool marked;
    unsigned int rank;
    unsigned int krank;
};

struct edge: DefaultGraphItem
{
	edge():weight(0)
	{	
	}

    void print( std::ofstream& out)
    {
        out << weight;
        return;
    }

    void setProperty( const std::string& name, const std::string& value)
    {
        DefaultGraphItem::setProperty( name, value);
        if( !name.compare("weight"))
        {
            std::istringstream stm;
            stm.str(value);
            stm >>weight;
        }
    }

    void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "w" << valueDelimiter << weight << propertyDelimiter;
    }

    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"w\":" << weight << "";
    }

    unsigned int weight;
};


template < typename GraphType>
void calcWeights( GraphType& G)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typedef typename GraphType::EdgeIterator EdgeIterator;
    typedef typename GraphType::InEdgeIterator InEdgeIterator;
    typedef typename GraphType::SizeType SizeType;

    NodeIterator u,v,lastnode;
    EdgeIterator e,lastedge;
    InEdgeIterator k;

    unsigned int max = 0;
    unsigned int max_ux, max_uy, max_vx, max_vy, u_id, v_id;

    std::stringstream sstr;
    sstr << "Calculating weights of " << G.getNumEdges() << " edges";
    ProgressBar edge_progress( G.getNumEdges(),sstr.str());
    Timer timer; 
    timer.start();
    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
	    for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
	    {
            
            v = G.target(e);
            k = G.getInEdgeIterator(e);
            
            e->weight = euclideanDistance( u->x, u->y, v->x, v->y);
            k->weight = e->weight;

            if( e->weight > max)
            {
                max_ux = u->x;
                max_uy = u->y;  
                max_vx = v->x;
                max_vy = v->y;
                u_id = G.getRelativePosition(u);
                v_id = G.getRelativePosition(v);
                max = e->weight;
            }
            ++edge_progress;
            //std::cout << u->lat << " " << v->lat << std::endl;
        }
    }

    std::cout << "Max weight: " << max << "\tTime:\t" << timer.getElapsedTime() << std::endl;
    //std::cout << "( " << max_ux << ", " << max_uy << ") " << u_id << "->" << v_id << " ( " << max_vx << ", " << max_vy << ")\n";
}

template< typename GraphType>
void bfsShuffle( GraphType& G, std::vector<typename GraphType::NodeDescriptor>& ids,const std::string& map)
{
    class Visitor : public SearchVisitor<GraphType>
    {
    public:
        Visitor(GraphType& G, std::vector<typename GraphType::NodeDescriptor>& newIds):m_G(G),m_newIds(newIds){}
        
        virtual void visitOnMarking( const typename GraphType::NodeIterator& u)
        {
            m_auxNodeIterator = u;
            //std::cout << "Connectivity Visitor\n";
            m_auxNodeIterator->rank = m_newIds.size();
            m_newIds.push_back( m_G.getNodeDescriptor(m_auxNodeIterator));
        }
    
    private:
        GraphType& m_G;
        std::vector<typename GraphType::NodeDescriptor>& m_newIds;
        typename GraphType::NodeIterator m_auxNodeIterator;
    };
    
    std::cout << "Shuffling BFS...\n";
    
    std::vector<typename GraphType::NodeDescriptor> newIds;
    Visitor vis(G,newIds);
    typename GraphType::NodeIterator s = G.chooseNode();
    bfsCore( G, s, &vis);
    
    DIMACS10Shuffler<GraphType> writer( basePath + map + "-bfs.osm.graph",
                                basePath + map + "-bfs.osm.xyz",newIds);
                            G.write(&writer);
    //std::cout << "Ids: " << newIds.size();
    return;
}


template< typename GraphType>
void dfsShuffle( GraphType& G, std::vector<typename GraphType::NodeDescriptor>& ids, const std::string& map)
{
    class Visitor : public SearchVisitor<GraphType>
    {
    public:
        Visitor(GraphType& G, std::vector<typename GraphType::NodeDescriptor>& newIds):m_G(G),m_newIds(newIds){}
        
        void visitOnMarking( const typename GraphType::NodeIterator& u)
        {
            m_auxNodeIterator = u;
            //std::cout << "Connectivity Visitor\n";
            m_auxNodeIterator->rank = m_newIds.size();
            m_newIds.push_back( m_G.getNodeDescriptor(m_auxNodeIterator));
        }
    
    private:
        GraphType& m_G;
        std::vector<typename GraphType::NodeDescriptor>& m_newIds;
        typename GraphType::NodeIterator m_auxNodeIterator;
    };
    
    std::cout << "Shuffling DFS...\n";
    
    std::vector<typename GraphType::NodeDescriptor> newIds;
    Visitor vis(G,newIds);
    typename GraphType::NodeIterator s = G.chooseNode();
    dfsCore( G, s, &vis);
    
    DIMACS10Shuffler<GraphType> writer( basePath + map + "-dfs.osm.graph",
                                basePath + map + "-dfs.osm.xyz",newIds);
                            G.write(&writer);
    //std::cout << "Ids: " << newIds.size();
    return;
}


template< typename GraphType>
void rankNodes( GraphType& G, std::vector<typename GraphType::NodeDescriptor>& ids, const std::string& map)
{
    class Visitor : public SearchVisitor<GraphType>
    {
    public:
        Visitor(GraphType& G, std::vector<typename GraphType::NodeDescriptor>& newIds):m_G(G),m_newIds(newIds){}
        
        void visitOnMarking( const typename GraphType::NodeIterator& u)
        {
            m_auxNodeIterator = u;
            //std::cout << "Connectivity Visitor\n";
            m_auxNodeIterator->rank = m_newIds.size();
            m_newIds.push_back( m_G.getNodeDescriptor(m_auxNodeIterator));
        }
    
    private:
        GraphType& m_G;
        std::vector<typename GraphType::NodeDescriptor>& m_newIds;
        typename GraphType::NodeIterator m_auxNodeIterator;
    };
    
    std::cout << "Ranking with DFS...\n";
    
    std::vector<typename GraphType::NodeDescriptor> newIds;
    Visitor vis(G,newIds);
    typename GraphType::NodeIterator s = G.chooseNode();
    dfsCore( G, s, &vis);
    
    std::stringstream sstr;
    sstr << "Moving " <<  newIds.size() << " nodes";
    ProgressBar move_progress( newIds.size(),sstr.str());
    for( unsigned int i = 0; i < newIds.size(); ++i)
    {
        G.move( newIds[i], G.getNodeDescriptor( G.beginNodes()));
        ++move_progress;
    }
    
    DIMACS10Writer<GraphType> writer( basePath + map + "-ranked.osm.graph",
                                basePath + map + "-ranked.osm.xyz");
                            G.write(&writer);
    //std::cout << "Ids: " << newIds.size();
    return;
}

template< typename GraphType>
void dummyShuffle( GraphType& G, std::vector<typename GraphType::NodeDescriptor>& ids, const std::string& map)
{
    std::cout << "Dummy Shuffling...\n";
    
    std::vector<typename GraphType::NodeDescriptor> newIds;
    
    for(  unsigned int i = 0; i < ids.size() ; ++i)
    {
        typename GraphType::NodeIterator u = G.getNodeIterator( ids[i]);
        u->rank = i;
        newIds.push_back(ids[i]);
    }
    
    DIMACS10Shuffler<GraphType> writer( basePath + map + "-dummy.osm.graph",
                                basePath + map + "-dummy.osm.xyz",newIds);
                            G.write(&writer);
    //std::cout << "Ids: " << newIds.size();
    return;
}

template< typename GraphType>
void testEdgeInsertions( GraphType& G, std::vector< std::pair< double, double> >& insertions, std::vector<typename GraphType::NodeDescriptor>& ids, const std::string& map)
{
    typedef typename GraphType::NodeDescriptor NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor EdgeDescriptor;

    unsigned int sourceId, targetId;    
    std::vector<typename GraphType::EdgeDescriptor> newEdges;

    ProgressStream insertionProgress( insertions.size());
    insertionProgress.label() << "Inserting " << insertions.size() << " edges";
    ProgressStream erasureProgress( insertions.size());
    erasureProgress.label() << "Erasing " << insertions.size() << " edges";

    Timer timer;
    timer.start();
    for( typename std::vector< std::pair< double, double> >::iterator it = insertions.begin(), end = insertions.end(); it != end; ++it)
    {
        sourceId = it->first * ids.size();
        targetId = it->second * ids.size();
        newEdges.push_back(G.insertEdge( ids[sourceId], ids[targetId]));
        ++insertionProgress;
    }    
    std::cout << "\tTime:\t" << timer.getElapsedTime() << "sec ( " << (1000 * timer.getElapsedTime() / insertions.size()) << "ms per insertion)\n";
    timer.start();

    for( typename std::vector< EdgeDescriptor>::iterator it = newEdges.begin(), end = newEdges.end(); it != end; ++it)
    {
        G.eraseEdge(*it);
        ++erasureProgress;
    }
    std::cout << "\tTime:\t" << timer.getElapsedTime() << "sec ( " << (1000 * timer.getElapsedTime() / insertions.size()) << "ms per erasure)\n";
    

    return;
}



typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   pmaGraph;
typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;

int main( int argc, char* argv[])
{
    MersenneTwister gen;  
    unsigned int graphVariant = 0;
    unsigned int numInsertions = 0;
    std::string map ="luxembourg";

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("graphtype,g", po::value< unsigned int>(), "graph type. All[0], Adjacency List[1], Packed Memory Graph[2]. Default:0")
        ("map,m", po::value< std::string>(), "input map. The name of the map to read. Maps should reside in '$HOME/Projects/Graphs/DIMACS10/' and should consist of 2 files, both with the same map name prefix, and suffixes 'osm.graph' and 'osm.xyz'. Default:'luxembourg'")
        ("insertions,i", po::value< unsigned int>(), "number of random edge insertions to execute")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if (vm.empty()) {
        std::cout << desc << "\n";
        return 0;
    }

    if (vm.count("graphtype"))
    {
        graphVariant = vm["graphtype"].as<unsigned int>();
    }
    
    if (vm.count("map"))
    {
        map = vm["map"].as<std::string>();
    }

    if (vm.count("graphtype"))
    {
        numInsertions = vm["insertions"].as<unsigned int>();
    }

    Graph G;
    pmaGraph pmaG;

    DIMACS10Reader<Graph> reader( basePath + map + ".osm.graph",
                                basePath + map + ".osm.xyz");
    
    DIMACS10Reader<pmaGraph> pmaReader( basePath + map + ".osm.graph",
                                basePath + map + ".osm.xyz");



    std::vector< std::pair<double,double> > insertions;
    for( unsigned int i = 0; i < numInsertions; i++)
    {
        insertions.push_back( std::pair< double,double>( gen.getRandomNormalizedDouble(), gen.getRandomNormalizedDouble()));
    }


    switch( graphVariant)
    {
        case 1:
            G.read(&reader);
            std::cout << "Graph has " << G.memUsage() << " bytes " << std::endl;
            calcWeights(G);
            //G.move(G.getNodeDescriptor(G.chooseNode()), G.getNodeDescriptor(G.beginNodes()));
            dfsShuffle( G, reader.getIds(), map);
            bfsShuffle( G, reader.getIds(), map);
            testEdgeInsertions( G, insertions, reader.getIds(), map);
            //rankNodes( G, reader.getIds(), map);
            break;
        case 2:
            pmaG.read(&pmaReader);
            std::cout << "Graph has " << pmaG.memUsage() << " bytes " << std::endl;
            calcWeights(pmaG);
            //pmaG.move(pmaG.getNodeDescriptor(pmaG.chooseNode()), pmaG.getNodeDescriptor(pmaG.beginNodes()));
            dfsShuffle( pmaG, pmaReader.getIds(), map);
            bfsShuffle( pmaG, pmaReader.getIds(), map);
            testEdgeInsertions( pmaG, insertions, pmaReader.getIds(), map);
            //rankNodes( pmaG, pmaReader.getIds(), map);
            break;
        default:
            break;
    }

    return 0;
}
