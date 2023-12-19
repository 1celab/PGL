#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <Structs/Graphs/forwardStarImpl.h>
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

struct node: DefaultGraphItem
{
	node():x(0),y(0),dist(std::numeric_limits<unsigned int>::max()),pqitem(0),pred(0),timestamp(0)
	{	
	}

    void print( std::ofstream& out)
    {
        out << "\\|" << dist;
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
    unsigned int dist,distBack;
    unsigned int pqitem,pqitemBack;
    void* pred;
    void* succ;
    unsigned int timestamp;
    //bool marked;
    //unsigned int rank;
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

    std::cout << "\tMax weight: " << max << "\tTime:\t" << timer.getElapsedTime() << std::endl;
    //std::cout << "( " << max_ux << ", " << max_uy << ") " << u_id << "->" << v_id << " ( " << max_vx << ", " << max_vy << ")\n";
}

template<typename GraphType>
void runExperimentsAt( GraphType& G, std::vector< std::pair<double,double> >& queries, std::vector<typename GraphType::NodeDescriptor>& ids, const std::string& name, unsigned int mode)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typedef typename GraphType::EdgeIterator EdgeIterator;
    typedef typename GraphType::InEdgeIterator InEdgeIterator;
    typedef typename GraphType::EdgeDescriptor EdgeDescriptor;

    typename GraphType::NodeIterator s,t;
    typename GraphType::EdgeDescriptor desc;    

    NodeIterator u,v,lastnode;
    EdgeIterator e,endEdges;
    InEdgeIterator k,endInEdges;

    unsigned int sourceId, targetId, dist;
    unsigned int timestamp = 0;

    //create output message
    std::string message("Moving at ");
    message.append( name);
    ProgressBar show_progress( queries.size(), message);

    std::vector<EdgeDescriptor> toDelete;

    //run queries
    Timer timer; 
    timer.start();
    for( std::vector< std::pair<double,double> >::iterator it = queries.begin(); it != queries.end(); ++it)
    {
        sourceId = it->first * ids.size();

        if( sourceId == 0) ++sourceId;
        if( sourceId == ids.size()) --sourceId;
        targetId = it->second * ids.size();

        if( targetId == sourceId) ++targetId;
        if( targetId == 0) ++targetId;
        if( targetId == ids.size()) --targetId;

        //s = G.getNodeIterator( ids[sourceId]);
        //t = G.getNodeIterator( ids[targetId]);

        //G.move(ids[sourceId],ids[targetId]);
        switch (mode)
        {
            case 0:
                G.insertEdge( ids[sourceId], ids[targetId]);
                break;
            case 1:
                u = G.getNodeIterator(ids[sourceId]);
                if( G.hasEdges(u))
                {
                    e = G.beginEdges(u);
                    G.eraseEdge( G.getEdgeDescriptor(e));
                }
            case 2:
                G.insertNode();
                break;
            case 3:
                u = G.getNodeIterator(ids[sourceId]);
                toDelete.clear();
                for( e = G.beginEdges(u), endEdges = G.endEdges(u); e != endEdges; ++e)
                {
                    toDelete.push_back( G.getEdgeDescriptor(e));
                }
				for( k = G.beginInEdges(u), endInEdges = G.endInEdges(u); k != endInEdges; ++k)
                {
				    e = G.getEdgeIterator( k);
                    toDelete.push_back( G.getEdgeDescriptor(e));
                }
                for( unsigned int j = 0; j < toDelete.size(); ++j)
                {
                    G.eraseEdge( toDelete[j]);
                }
                if( G.hasNode(ids[sourceId]))
                    G.eraseNode( ids[sourceId]);
                break;
            case 4:
                G.move(ids[sourceId],ids[targetId]);
                break;
        }
        
//        G.eraseEdge( desc);

        ++show_progress;
    }

    std::cout << "\tTime:\t" << timer.getElapsedTime() << "sec ( " << (1000 * timer.getElapsedTime() / queries.size()) << "ms per query)";
    std::cout << "\n\n";
}



template< typename GraphType>
void shuffle( GraphType& G, std::vector<typename GraphType::NodeDescriptor>& ids, std::vector<typename GraphType::NodeDescriptor>& newIds)
{
    class Visitor : public SearchVisitor<GraphType>
    {
    public:
        Visitor(GraphType& G, std::vector<typename GraphType::NodeDescriptor>& newIds):m_G(G),m_newIds(newIds){}
        
        virtual void visitOnMarking( const typename GraphType::NodeIterator& u)
        {
            m_auxNodeIterator = u;
            //std::cout << "Connectivity Visitor\n";
            m_newIds.push_back( m_G.getNodeDescriptor(m_auxNodeIterator));
            m_auxNodeIterator->rank = m_newIds.size();
        }
    
    private:
        GraphType& m_G;
        std::vector<typename GraphType::NodeDescriptor>& m_newIds;
        typename GraphType::NodeIterator m_auxNodeIterator;
    };
    
    Visitor vis(G,newIds);
    typename GraphType::NodeIterator s = G.chooseNode();
    bfsCore( G, s, &vis);
    //std::cout << "Ids: " << newIds.size();
    return;
    for(  unsigned int i = 1; i < ids.size() ; ++i)
    {
        typename GraphType::NodeIterator u = G.getNodeIterator( ids[i]);
        u->rank = i;
        newIds.push_back(ids[i]);
    }
}


typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   pmaGraph;
typedef DynamicGraph< ForwardStarImpl, node, edge>         fsGraph;
typedef DynamicGraph< AdjacencyListImpl, node, edge>         Graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;

typedef GraphReader< Graph> AdjReader;
typedef GraphReader< pmaGraph> PmaReader;
typedef GraphReader< fsGraph> FsReader;

int main( int argc, char* argv[])
{
    MersenneTwister gen;  

    std::string basePath = "/home/michai/Projects/Graphs/DIMACS10/";

    unsigned int numQueries = 100;
    unsigned int dijkstraVariant = 0;
    unsigned int graphVariant = 0;
    std::string map ="luxembourg";
    unsigned int mode = 0;
    unsigned int format = 0;

    AdjReader* reader = 0;
	PmaReader* pmaReader = 0;	
    FsReader* fsReader = 0;


    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("size,s", po::value< unsigned int>(), "number of queries. Default:100")
        ("graphtype,g", po::value< unsigned int>(), "graph type. All[0], Adjacency List[1], Packed Memory Graph[2], Forward Star[3]. Default:0")
        ("map,m", po::value< std::string>(), "input map. The name of the map to read. Maps should reside in '$HOME/Projects/Graphs/DIMACS10/' and should consist of 2 files, both with the same map name prefix, and suffixes 'osm.graph' and 'osm.xyz'. Default:'luxembourg'")
        ("format,f", po::value< unsigned int>(), "map format. DIMACS10[0], DIMACS9[1]. Default:0")
        ("operation,o", po::value< unsigned int>(), "Operation Mode. Insert Edges[0], Remove Edges[1], Insert Nodes[2], Remove Nodes[3], Rellocate Nodes[4]. Default:0")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if (vm.empty()) {
        std::cout << desc << "\n";
        return 0;
    }

    if (vm.count("size"))
    {
        numQueries = vm["size"].as<unsigned int>();
    }

    if (vm.count("operation"))
    {
        mode = vm["operation"].as<unsigned int>();
    }

    if (vm.count("graphtype"))
    {
        graphVariant = vm["graphtype"].as<unsigned int>();
    }
    
    if (vm.count("map"))
    {
        map = vm["map"].as<std::string>();
    }

    std::vector< std::pair<double,double> > queries;
    for( unsigned int i = 0; i < numQueries; i++)
    {
        queries.push_back( std::pair< double,double>( gen.getRandomNormalizedDouble(), gen.getRandomNormalizedDouble()));
    }

    std::vector<unsigned int> results,pmaResults;

    Graph G;
    pmaGraph pmaG;
    fsGraph fsG;

    if ( vm.count("format") && ( format = vm["format"].as<unsigned int>()) == 1)
    {
        basePath = basePath + "DIMACS9/" + map + "/";

        reader = new DIMACS9Reader<Graph>( basePath + map + ".gr",
                                           basePath + map + ".co");

        pmaReader = new DIMACS9Reader<pmaGraph>( basePath + map + ".gr",
                                                 basePath + map + ".co");

        fsReader = new DIMACS9Reader<fsGraph>( basePath + map + ".gr",
                                               basePath + map + ".co");
    }

    else
    {
        basePath = basePath + "DIMACS10/" + map + "/";

        reader = new DIMACS10Reader<Graph>( basePath + map + ".osm.graph",
                                            basePath + map + ".osm.xyz");

        pmaReader = new DIMACS10Reader<pmaGraph>( basePath + map + ".osm.graph",
                                                  basePath + map + ".osm.xyz");

        fsReader = new DIMACS10Reader<fsGraph>( basePath + map + ".osm.graph",
                                                basePath + map + ".osm.xyz");
    }


    Timer timer;
    switch( graphVariant)
    {
        
        case 1:
            timer.start();
            G.read(reader);
            std::cout << "Graph has " << (double)G.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            break;
        case 2:
            timer.start();
            pmaG.read(pmaReader);
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            break;
        case 3:
            timer.start();
            fsG.read(fsReader);
            std::cout << "Graph has " << (double)fsG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            break;
        default:
            timer.start();
            G.read(reader);
            std::cout << "Graph has " << (double)G.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            timer.start();
            pmaG.read(pmaReader);
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            timer.start();
            fsG.read(fsReader);
            std::cout << "Graph has " << (double)fsG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            break;
    }
    

    switch( graphVariant)
    {
        case 1:
            runExperimentsAt( G, queries, reader->getIds(),  "Adjacency", mode);
            break;
        case 2:
            runExperimentsAt( pmaG, queries, pmaReader->getIds(),  "PMA", mode);
            break;
        case 3:
            runExperimentsAt( fsG, queries, fsReader->getIds(),  "Forward Star", mode);
            break;
        default:
            runExperimentsAt( G, queries, reader->getIds(),  "Adjacency", mode);
            runExperimentsAt( pmaG, queries, pmaReader->getIds(),  "PMA", mode);
            runExperimentsAt( fsG, queries, fsReader->getIds(),  "Forward Star", mode);
            break;
    }

    std::cout << "Graph has " << G.getNumEdges() << " edges" << std::endl;

    return 0;
}
