#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/adjacencyListImpl.h>
#include <Structs/Graphs/packedMemoryArrayImpl.h>
#include <Algorithms/ShortestPath/Multicriteria/multicriteriaDijkstra.h>
#include <Algorithms/ShortestPath/Multicriteria/namoaStar.h>
#include <Algorithms/ShortestPath/Multicriteria/multicriteriaArc.h>
#include <Utilities/geographic.h>
#include <Utilities/timer.h>
#include <boost/program_options.hpp>
#define NUM_CRITERIA 2

namespace po = boost::program_options;
MersenneTwister gen; 

struct node: DefaultGraphItem
{
	node( unsigned int data = 0): timestamp(0),heuristicList(NUM_CRITERIA)
	{	
	}

    template <class GraphType>
    void printLabels( std::ostream& out, GraphType& G)
    {
        for ( std::vector<Label>::iterator it = labels.begin(); it != labels.end(); ++it)
        {
            out << "\t";
            it->print(out, G);
            out << "\n";
        }
        return;
    }

    unsigned int x, y;
    std::vector<Label> labels;
	unsigned int timestamp;
	
    unsigned int dist;
    unsigned int pqitem, secondary_pqitem;
    unsigned int selectionID;
	CriteriaList heuristicList;

    unsigned int distBack, pqitemBack;
    void* succ;
    bool marked;
    unsigned int cell;
};


struct edge: DefaultGraphItem
{
	edge( unsigned int data = 0): criteriaList(NUM_CRITERIA),flags(0),local(true)
	{	
	}

    CriteriaList criteriaList;
    unsigned int flags;
    bool local;
};



class Results
{
public:
    Results( const std::vector< std::pair<unsigned int, unsigned int> >& queries):m_queries(queries)
    {
    }

    void add( const std::string& header, const std::vector< double >& results)
    {
        m_headers.push_back(header);
        m_data.push_back(results);
    }

    void print(std::ostream& out)
    {
        out << "source\tdestination";
        for( unsigned int i = 0; i < m_headers.size(); ++i)
        {
            out << "\t" << m_headers[i];
        }
        out << "\n";
        for( unsigned int i = 0; i < m_queries.size(); ++i)
        {
            out << m_queries[i].first << "\t" << m_queries[i].second;
            for( unsigned int j = 0; j < m_data.size(); ++j)
            {
                out << "\t" << m_data[j][i];
            }
            out << "\n";
        }
    }

private:
    std::vector< std::string> m_headers;
    std::vector< std::vector<double> > m_data;
    const std::vector< std::pair<unsigned int, unsigned int> >& m_queries;
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
	double max_speed = 0;

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

            if( e->criteriaList[1] > max)
            {
                max_ux = u->x;
                max_uy = u->y;  
                max_vx = v->x;
                max_vy = v->y;
                u_id = G.getRelativePosition(u);
                v_id = G.getRelativePosition(v);
                max = e->criteriaList[1];
            }

			if( double( e->criteriaList[0])/double( e->criteriaList[1]) > max_speed)
			{
				max_speed = double( e->criteriaList[0])/double( e->criteriaList[1]);
			}

            ++edge_progress;
        }
    }

    std::cout << "\tMax weight: " << max << "\tTime:\t" << timer.getElapsedTime() << std::endl;
}

template< class DijkstraVariant, typename GraphType>
void runQueries( GraphType& G, std::vector< std::pair<unsigned int,unsigned int> >& queries, std::vector<typename GraphType::NodeDescriptor>& ids, Results& results, const std::string& graphname, const std::string& algoname)
{
    std::vector<double> times;
    std::vector<double> paths;

    typedef typename GraphType::NodeIterator NodeIterator;
    typename GraphType::NodeIterator s,t;
    
    unsigned int sourceId, targetId;
    unsigned int timestamp = 0;
    
    //clear nodes
    NodeIterator u, lastnode;
    for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
    {
        u->labels.clear();
    }

    //create dijkstra
    DijkstraVariant dijkstra(G, NUM_CRITERIA, &timestamp);
    
    //create output message
    std::string message("Experiments at ");
    message.append( graphname + " " + algoname);
    ProgressBar show_progress( queries.size(), message);

    std::cout << message << std::endl;

    //run queries
    Timer timer; 
    
    for( std::vector< std::pair<unsigned int,unsigned int> >::iterator it = queries.begin(); it != queries.end(); ++it)
    {
        sourceId = it->first;
        targetId = it->second;
        std::cout << sourceId << "->" << targetId << std::endl;
        s = G.getNodeIterator( ids[sourceId]);
        t = G.getNodeIterator( ids[targetId]);
        
		std::cout << "Initializing...\n";
		timer.start();
		dijkstra.init(s,t);
		std::cout << "Running query:\n";
		
        dijkstra.runQuery( s, t);
        times.push_back(timer.getElapsedTime());
        paths.push_back(t->labels.size());
		std::cout << "\tTime:\t" << times[times.size()-1] << "sec\n"; 
		std::cout << "\tNon-dominated solutions:\t" << t->labels.size() << "\n"; 
		std::cout << "\tGenerated labels: " << dijkstra.getGeneratedLabels() << "\n\n";
    }

    results.add(graphname + " " + algoname,times);
    results.add(graphname + " " + algoname + " |C|",paths);
}

template< typename GraphType>
void runExperimentsAt( GraphType& G, std::vector< std::pair<unsigned int,unsigned int> >& queries, std::vector<typename GraphType::NodeDescriptor>& ids, Results& results, const std::string& name, const unsigned int& dijkstraVariant, const unsigned int& heuristicVariant)
{
    switch( dijkstraVariant)
    {
        case 1:
            runQueries<MulticriteriaDijkstra<GraphType> >( G, queries, ids, results, name, "All Pareto");
            break;
        case 2:
            switch (heuristicVariant)
            {
                case 1:
			        runQueries<NamoaStarDijkstra<GraphType,BlindHeuristic> >( G, queries, ids, results, name, "NAMOA* Blind");
                    break;
                case 2:
			        runQueries<NamoaStarDijkstra<GraphType,GreatCircleHeuristic> >( G, queries, ids, results, name, "NAMOA* GC");
                    break;
                case 3:
			        runQueries<NamoaStarDijkstra<GraphType,TCHeuristic> >( G, queries, ids, results, name, "NAMOA* TC");
                    break;
                case 4:
			        runQueries<NamoaStarDijkstra<GraphType,BoundedTCHeuristic> >( G, queries, ids, results, name, "NAMOA* Bounded TC");
                    break;
                default:
                    runQueries<NamoaStarDijkstra<GraphType,BlindHeuristic> >( G, queries, ids, results, name, "NAMOA* Blind");
                    runQueries<NamoaStarDijkstra<GraphType,GreatCircleHeuristic> >( G, queries, ids, results, name, "NAMOA* GC");
                    runQueries<NamoaStarDijkstra<GraphType,TCHeuristic> >( G, queries, ids, results, name, "NAMOA* TC");
                    runQueries<NamoaStarDijkstra<GraphType,BoundedTCHeuristic> >( G, queries, ids, results, name, "NAMOA* Bounded TC");
            }
            break;
        case 3:
            runQueries<NamoaStarArc<GraphType,TCHeuristic> >( G, queries, ids, results, name, "Multicriteria Arc");
            break;
        default:
            runQueries<MulticriteriaDijkstra<GraphType> >( G, queries, ids, results, name, "All Pareto");
            switch (heuristicVariant)
            {
                case 1:
			        runQueries<NamoaStarDijkstra<GraphType,BlindHeuristic> >( G, queries, ids, results, name, "NAMOA* Blind");
                    break;
                case 2:
			        runQueries<NamoaStarDijkstra<GraphType,GreatCircleHeuristic> >( G, queries, ids, results, name, "NAMOA* GC");
                    break;
                case 3:
			        runQueries<NamoaStarDijkstra<GraphType,TCHeuristic> >( G, queries, ids, results, name, "NAMOA* TC");
                    break;
                default:
			        runQueries<NamoaStarDijkstra<GraphType,BoundedTCHeuristic> >( G, queries, ids, results, name, "NAMOA* Bounded TC");
                    break;
            }
			runQueries<MulticriteriaArc<GraphType> >( G, queries, ids, results, name, "Multicriteria Arc");
            break;
    }
}


void readQueries( std::vector< std::pair<unsigned int,unsigned int> >& queries, const std::string& filename, const unsigned int& numQueries)
{
	std::string token;
	unsigned int uID,vID;
	std::ifstream in;
    in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
    std::cout << "Reading queries from " << filename << std::endl;
	unsigned int readQueries = 0;

	try {
    	in.open( filename.c_str());
        assert( in.good());
        while ( ((readQueries < numQueries) || numQueries == 0) && getline(in,token)) 
        {
			//std::cout << numQueries << std::endl;
			std::stringstream graphinfo;                  
			graphinfo.str(token);   
        	graphinfo >> uID >> vID;
			queries.push_back( std::pair<unsigned int, unsigned int>( uID, vID));
			++readQueries;
        }
        in.close();
    }
    catch (std::ifstream::failure e) {
        std::cerr << "Exception opening/reading file '" << filename << "'\n";
        throw e;
    }
}

typedef DynamicGraph< AdjacencyListImpl, node, edge>       Graph;
//typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   Graph;
typedef DynamicGraph< PackedMemoryArrayImpl, node, edge>   pmaGraph;
typedef DynamicGraph< AdjacencyListImpl>                   graph;
typedef Graph::NodeIterator                                NodeIterator;
typedef Graph::EdgeIterator                                EdgeIterator;
typedef Graph::NodeDescriptor                              NodeDescriptor;


int main( int argc, char* argv[])
{
    MersenneTwister gen;  

    std::string basePath = "/home/michai/Projects/Graphs/DIMACS9/no_antiparallels/";

    unsigned int numQueries = 100;
    unsigned int dijkstraVariant = 0;
    unsigned int graphVariant = 0;
    unsigned int heuristicVariant = 0;
    std::string map ="NY";

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("size,s", po::value< unsigned int>(), "number of queries. Default:100")
        ("dijkstra,d", po::value< unsigned int>(), "dijkstra algorithm. All[0], Plain[1], A*[2], Bidirectional[3]. Default:0")
        ("graphtype,g", po::value< unsigned int>(), "graph type. All[0], Adjacency List[1], Packed Memory Graph[2]. Default:0")
        ("heuristic,h", po::value< unsigned int>(), "heuristic variant. All[0], Blind[1], Great Circle[2], TC[3], Bounded TC[4]. Default:0")
        ("map,m", po::value< std::string>(), "input map. The name of the map to read. Maps should reside in '$HOME/Projects/Graphs/DIMACS10/' and should consist of 2 files, both with the same map name prefix, and suffixes 'osm.graph' and 'osm.xyz'. Default:'luxembourg'")
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

    if (vm.count("heuristic"))
    {
        heuristicVariant = vm["heuristic"].as<unsigned int>();
    }

    if (vm.count("graphtype"))
    {
        graphVariant = vm["graphtype"].as<unsigned int>();
    }
    
    if (vm.count("map"))
    {
        map = vm["map"].as<std::string>();
    }


    Graph G;
    pmaGraph pmaG;

    DIMACS9DoubleReader<Graph> reader( basePath + map + "_dist.gr",
                                basePath + map + "_travel.gr", basePath + map + ".co");
    
    DIMACS9DoubleReader<pmaGraph> pmaReader( basePath + map + "_dist.gr",
                                basePath + map + "_travel.gr", basePath + map + ".co");


    Timer timer;
	unsigned int numNodes = 0;
    switch( graphVariant)
    {
        
        case 1:
            timer.start();
            G.read(&reader);
			numNodes = reader.getIds().size();
            std::cout << "Graph has " << (double)G.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            calcWeights(G);
            break;
        case 2:
            timer.start();
            pmaG.read(&pmaReader);
			numNodes = pmaReader.getIds().size();
            pmaG.compress();
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            calcWeights(pmaG);
            break;
        default:
            timer.start();
            G.read(&reader);
			numNodes = reader.getIds().size();
            std::cout << "Graph has " << (double)G.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            calcWeights(G);
            timer.start();
            pmaG.read(&pmaReader);
            pmaG.compress();
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            calcWeights(pmaG);
            break;
    }
    
	
	std::vector< std::pair<unsigned int,unsigned int> > queries;
    /*for( unsigned int i = 0; i < numQueries; i++)
    {
        queries.push_back( std::pair< unsigned int,unsigned int>( gen.getRandomNormalizedDouble()*numNodes, gen.getRandomNormalizedDouble()*numNodes));
    }*/


	readQueries( queries, basePath + map + ".queries", numQueries);
	//queries.push_back( std::pair< unsigned int,unsigned int>( 251416,53900));

    /*std::vector< pmaGraph::NodeDescriptor> newIds;
    std::cout << "Shuffling...\n";
    shuffle( pmaG, pmaReader.getIds(), newIds);
    DIMACS10Shuffler<pmaGraph> pmaWriter( basePath + map + "-bfs.osm.graph",
                                basePath + map + "-bfs.osm.xyz",newIds);
                            pmaG.write(&pmaWriter);
                            return 0;
    */
    //sleep(1);

    Results results(queries);

    switch( graphVariant)
    {
        case 1:
            runExperimentsAt( G, queries, reader.getIds(), results, "ADJ", dijkstraVariant, heuristicVariant);
            break;
        case 2:
            runExperimentsAt( pmaG, queries, pmaReader.getIds(), results, "PMG", dijkstraVariant, heuristicVariant);
            break;
        default:
            runExperimentsAt( G, queries, reader.getIds(), results, "ADJ", dijkstraVariant, heuristicVariant);
            runExperimentsAt( pmaG, queries, pmaReader.getIds(), results, "PMG", dijkstraVariant, heuristicVariant);
            break;
    }

    std::ofstream out("results");
    results.print(out);
    out.close();
    return 0;
}
