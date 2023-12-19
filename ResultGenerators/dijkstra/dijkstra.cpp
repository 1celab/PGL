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
#include <Algorithms/ShortestPath/ContractionHierarchies/chDijkstra.h>
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
    unsigned int rank;
	unsigned int selectionID;
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
	void* viaNode;
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
				//std::cout << "max:\n";
				//std::cout << u->x << " " << u->y << "\n";
				//std::cout << v->x << " " << v->y << "\n";
                //u_id = G.getRelativePosition(u);
                //v_id = G.getRelativePosition(v);
				//std::cout << u_id << " " << v_id << "\n";
                max = e->weight;
            }
            ++edge_progress;
            //std::cout << u->lat << " " << v->lat << std::endl;
        }
    }

    std::cout << "\tMax weight: " << max << "\tTime:\t" << timer.getElapsedTime() << std::endl;
    //std::cout << "( " << max_ux << ", " << max_uy << ") " << u_id << "->" << v_id << " ( " << max_vx << ", " << max_vy << ")\n";
}

template< class DijkstraVariant, typename GraphType>
void runSPQueries( GraphType& G, std::vector< std::pair<double,double> >& queries, std::vector<typename GraphType::NodeDescriptor>& ids, std::vector<unsigned int>& results, const std::string& name)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typename GraphType::NodeIterator s,t;

    unsigned int sourceId, targetId, dist;
    unsigned int timestamp = 0;

    //clear nodes
    NodeIterator u, lastnode;
    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
        u->timestamp = 0;
        u->dist = std::numeric_limits<unsigned int>::max();
        u->distBack = std::numeric_limits<unsigned int>::max();
        u->pred = 0;
        u->succ = 0;
    }
    results.clear();

    //create dijkstra
    DijkstraVariant dijkstra(G, &timestamp);

    //create output message
    std::string message("Experiments at ");
    message.append( name);
    ProgressBar show_progress( queries.size(), message);

    //run queries
    double numSettled = 0;
    Timer timer;
    double exeTime = 0.0;
    for( std::vector< std::pair<double,double> >::iterator it = queries.begin(); it != queries.end(); ++it)
    {
        sourceId = it->first * ids.size();
        targetId = it->second * ids.size();
        //std::cout << sourceId << "->" << targetId << std::endl;
        s = G.getNodeIterator( ids[sourceId]);
        t = G.getNodeIterator( ids[targetId]);

        timer.start();
        dist = dijkstra.runQuery( s, t);
        timer.stop();
		//std::cout << "dist = " << dist << "\n";
        results.push_back(t->dist);
        exeTime += timer.getElapsedTime();
        numSettled += dijkstra.getNumSettledNodes();
        ++show_progress;

        std::string outputFilename = std::string("path_") + std::to_string(it-queries.begin()) + std::string(".out");
        std::ofstream out(outputFilename);
        while( t != s)
        {
            t = G.getNodeIterator(t->pred);
            std::pair<double,double> mercator( t->y, t->x);
            std::pair<double,double> geographic = ToGeographic(mercator);
            out << geographic.first << "," << geographic.second << std::endl;
        }
        out.close();
    }

    std::cout << "\tTime: " << exeTime << "secs   (" << (1000 * exeTime / queries.size()) << "ms per query)";
    std::cout << "   (" << std::fixed << ((double) numSettled / queries.size()) << " settled nodes per query)";
    std::cout.flush();
}


template< class DijkstraVariant, typename GraphType>
void runSSQueries( GraphType& G, std::vector< std::pair<double,double> >& queries, std::vector<typename GraphType::NodeDescriptor>& ids, std::vector<unsigned int>& results, const std::string& name)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typename GraphType::NodeIterator s,t;

    unsigned int sourceId, targetId, dist;
    unsigned int timestamp = 0;

    //clear nodes
    NodeIterator u, lastnode;
    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
        u->timestamp = 0;
        u->dist = std::numeric_limits<unsigned int>::max();
        u->distBack = std::numeric_limits<unsigned int>::max();
        u->pred = 0;
        u->succ = 0;
    }
    results.clear();

    //create dijkstra
    DijkstraVariant dijkstra(G, &timestamp);

    //create output message
    std::string message("Experiments at ");
    message.append( name);
    ProgressBar show_progress( queries.size(), message);

    //run queries
    double numSettled = 0;
    double exeTime = 0;
    Timer timer;
    for( std::vector< std::pair<double,double> >::iterator it = queries.begin(); it != queries.end(); ++it)
    {
        sourceId = it->first * ids.size();
        //std::cout << sourceId << "->" << targetId << std::endl;
        s = G.getNodeIterator( ids[sourceId]);

        timer.start();
        dijkstra.buildTree( s);
        timer.stop();
        exeTime += timer.getElapsedTime();
        numSettled += dijkstra.getNumSettledNodes();
        ++show_progress;
    }

    std::cout << "\tTime: " << exeTime << "secs   (" << (1000 * exeTime / queries.size()) << "ms per query)";
    std::cout << "   (" << std::fixed << ((double) numSettled / queries.size()) << " settled nodes per query)";
    std::cout.flush();
}

template< typename GraphType>
void runExperimentsAt( GraphType& G, std::vector< std::pair<double,double> >& queries, std::vector<typename GraphType::NodeDescriptor>& ids, std::vector<unsigned int>& results, const std::string& name, const unsigned int& dijkstraVariant)
{
	unsigned int timestamp = 0;
	CHPreprocessing< GraphType> preprocessing( G, &timestamp);
    switch( dijkstraVariant)
    {
        case 1:
            runSSQueries<Dijkstra<GraphType> >( G, queries, ids, results, name + " with Single-Source Dijkstra");
            break;
        case 2:
            runSPQueries<Dijkstra<GraphType> >( G, queries, ids, results, name + " with Single-Pair Dijkstra");
            break;
        case 3:
            runSPQueries<AStarDijkstra<GraphType> >( G, queries, ids, results, name + " with A* Dijkstra");
            break;
        case 4:
            runSPQueries<BidirectionalDijkstra<GraphType> >( G, queries, ids, results, name + " with Bidirectional Dijkstra");
            break;
		case 5:
			preprocessing.process();
			runSPQueries<CHDijkstra<GraphType> >( G, queries, ids, results, name + " with Contraction Hierarchies Dijkstra");
			break;
        default:
            runSSQueries<Dijkstra<GraphType> >( G, queries, ids, results, name + " with Single-Source Dijkstra");
            runSPQueries<Dijkstra<GraphType> >( G, queries, ids, results, name + " with Single-Pair Dijkstra");
            runSPQueries<AStarDijkstra<GraphType> >( G, queries, ids, results, name + " with A* Dijkstra");
            runSPQueries<BidirectionalDijkstra<GraphType> >( G, queries, ids, results, name + " with Bidirectional Dijkstra");
            break;
    }
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
typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;

typedef GraphReader< Graph> AdjReader;
typedef GraphReader< pmaGraph> PmaReader;
typedef GraphReader< fsGraph> FsReader;

int main( int argc, char* argv[])
{
    MersenneTwister gen;

    std::string basePath = "";

    unsigned int numQueries = 100;
    unsigned int dijkstraVariant = 0;
    unsigned int graphVariant = 2;
    std::string mapDir ="./luxembourg";
    std::string mapFile = "luxembourg";
	std::string format = "DIMACS10";
	unsigned int choice;

	Graph G;
    pmaGraph pmaG;
    fsGraph  fsG;
	AdjReader* reader = 0;
	PmaReader* pmaReader = 0;
    FsReader* fsReader = 0;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("size,s", po::value< unsigned int>(), "number of queries. Default:100")
        ("dijkstra,d", po::value< unsigned int>(), "dijkstra algorithm. All[0], DijkstraSS[1], DijkstraSP[2], A*[3], Bidirectional[4], Contraction Hierarchies[5]. Default:0")
        ("graphtype,g", po::value< unsigned int>(), "graph type. All[0], Adjacency List[1], Packed Memory Graph[2], Forward Star [3]. Default:2")
		("format,f", po::value< unsigned int>(), "map format. DIMACS10[0], DIMACS9[1]. Default:0")
        ("map,m", po::value< std::string>(), "input map. The full directory path of the map to read. Maps should reside in the given directory path and should consist of 2 files, both with the same map name prefix, and suffixes ('/.../GraphDataBasePath/mapName/mapName.osm.graph' and '/.../GraphDataBasePath/mapName/mapName.osm.xyz' for DIMACS10, '/.../GraphDataBasePath/mapName/mapName.gr' and '/.../GraphDataBasePath/mapName/mapName.co' for DIMACS9). Default:'./luxembourg'")
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

    if (vm.count("dijkstra"))
    {
        dijkstraVariant = vm["dijkstra"].as<unsigned int>();
    }

    if (vm.count("graphtype"))
    {
        graphVariant = vm["graphtype"].as<unsigned int>();
    }

	if (vm.count("map"))
    {
        mapDir = vm["map"].as<std::string>();
        std::size_t found = mapDir.find_last_of("/\\");
        mapFile = mapDir.substr(found+1);
    }

	if (vm.count("format"))
    {
		choice = vm["format"].as<unsigned int>();
		switch( choice)
		{
			case 0:
                DEFAULT_GRAPH_INPUT_FORMAT:
				format = "DIMACS10";
				reader = new DIMACS10Reader<Graph>(
                                mapDir + "/" + mapFile + ".osm.graph",
                                mapDir + "/" + mapFile + ".osm.xyz");
				pmaReader = new DIMACS10Reader<pmaGraph>(
                                mapDir + "/" + mapFile + ".osm.graph",
                                mapDir + "/" + mapFile + ".osm.xyz");
                fsReader = new DIMACS10Reader<fsGraph>(
                                mapDir + "/" + mapFile +  ".osm.graph",
                                mapDir + "/" + mapFile +  ".osm.xyz");
				break;
			case 1:
				format = "DIMACS9";
				reader = new DIMACS9Reader<Graph>(
                                mapDir + "/" + mapFile +  ".gr",
                                mapDir + "/" + mapFile +  ".co");
				pmaReader = new DIMACS9Reader<pmaGraph>(
                                mapDir + "/" + mapFile +  ".gr",
                                mapDir + "/" + mapFile +  ".co");
                fsReader = new DIMACS9Reader<fsGraph>(
                                mapDir + "/" + mapFile +  ".gr",
                                mapDir + "/" + mapFile +  ".co");
				break;
		}
    }
	else
	{
		goto DEFAULT_GRAPH_INPUT_FORMAT;
	}

    std::vector< std::pair<double,double> > queries;
    for( unsigned int i = 0; i < numQueries; i++)
    {
        queries.push_back( std::pair< double,double>( gen.getRandomNormalizedDouble(), gen.getRandomNormalizedDouble()));
    }

    std::vector<unsigned int> results,pmaResults,fsResults;


    Timer timer;
    switch( graphVariant)
    {

        case 1:
            timer.start();
            G.read(reader);
            std::cout << "Graph has " << (double)G.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            if (format == "DIMACS10") calcWeights(G);
            break;
        case 2:
            timer.start();
            pmaG.read(pmaReader);
            pmaG.compress();
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            if (format == "DIMACS10") calcWeights(pmaG);
            break;
        case 3:
            timer.start();
            fsG.read(fsReader);
            std::cout << "Graph has " << (double)fsG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            if (format == "DIMACS10") calcWeights(fsG);
            break;
        default:
            timer.start();
            G.read(reader);
            std::cout << "Graph has " << (double)G.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            if (format == "DIMACS10") calcWeights(G);

            timer.start();
            pmaG.read(pmaReader);
            pmaG.compress();
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            if (format == "DIMACS10") calcWeights(pmaG);

            timer.start();
            fsG.read(fsReader);
            std::cout << "Graph has " << (double)fsG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            if (format == "DIMACS10") calcWeights(fsG);
            break;
    }


    /*std::vector< pmaGraph::NodeDescriptor> newIds;
    std::cout << "Shuffling...\n";
    shuffle( pmaG, pmaReader.getIds(), newIds);
    DIMACS10Shuffler<pmaGraph> pmaWriter( basePath + map + "-bfs.osm.graph",
                                basePath + map + "-bfs.osm.xyz",newIds);
                            pmaG.write(&pmaWriter);
                            return 0;
    */
    //sleep(1);

    switch( graphVariant)
    {
        case 1:
            runExperimentsAt( G, queries, reader->getIds(), results, "Adjacency", dijkstraVariant);
            break;
        case 2:
            runExperimentsAt( pmaG, queries, pmaReader->getIds(), pmaResults, "PMA", dijkstraVariant);
            break;
        case 3:
            runExperimentsAt( fsG, queries, fsReader->getIds(), fsResults, "Forward Star", dijkstraVariant);
            break;
        default:
            runExperimentsAt( G, queries, reader->getIds(), results, "Adjacency", dijkstraVariant);
            runExperimentsAt( pmaG, queries, pmaReader->getIds(), pmaResults, "PMA", dijkstraVariant);
            runExperimentsAt( fsG, queries, fsReader->getIds(), fsResults, "Forward Star", dijkstraVariant);
            break;
    }

    return 0;
}
