#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <Structs/Graphs/dynamicGraph.h>
#include <Structs/Graphs/forwardStarImpl.h>
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

#include <Algorithms/ShortestPath/Astar/bidAstarAveLmk.h>
#include <Algorithms/ShortestPath/Astar/LandmarkGenerator/lmkGen.h>
//the number of the active landmarks that will be used for each shortest path query
unsigned int numActiveLandmarks;

namespace po = boost::program_options;
MersenneTwister gen;
double insert_percentage = 0.01;
unsigned int numQueries = 100;
std::string basePath;
 
union LandmarkData
{
    unsigned int distanceToLandmark;
    unsigned int distanceFromLandmark;
};

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
    
    std::vector<LandmarkData> landmark; 
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
void runExperimentsAt( GraphType& G, std::vector< std::pair<double,double> >& queries, std::vector<typename GraphType::NodeDescriptor>& ids, const std::string& name)
{
    typedef typename GraphType::NodeIterator NodeIterator;
    typename GraphType::NodeIterator s,t;
    typename GraphType::EdgeIterator e;
    typename GraphType::InEdgeIterator k;
    typename GraphType::EdgeDescriptor desc;

    unsigned int sourceId, targetId;
    unsigned int timestamp = 0;
    unsigned int insertions = 0;
    unsigned int erasures = 0;
    unsigned int numqueries = 0;
    unsigned int op;
    double random;

    std::vector< typename GraphType::EdgeDescriptor> insertedEdges;

    //A* euclidean-based
    //AStarDijkstra<GraphType> astarSearch(G, &timestamp);
  
    //reading the landmarks
    Timer timer;

    numActiveLandmarks = 8;
    LandmarkGenerator<GraphType> landmarkGen(G, ids); 
    timer.start();
    landmarkGen.loadLandmarkDistances(basePath + "landmarks.dat");
    timer.stop();
    std::cout << "\tLandmark container capacity: "
              << (G.getNumNodes() * sizeof(unsigned int) * G.chooseNode()->landmark.capacity() / 1048576.0) << " Mbytes."
              << "\n\tTime spent to read the landmarks: " << timer.getElapsedTime() << "sec\n";


    //A* landmark-based
    BidAstarAveLmk<GraphType> astarSearch(G, &timestamp);
    Dijkstra<GraphType> dijkstra(G, &timestamp);

    //checking feasibility - enabled
    if(astarSearch.hasFeasiblePotentials(G.chooseNode()) == false)
        std::cout << "Warning, feasibility isn't achieved! There is no guarantee finding the optimal SPs .\n";

    //create output message
    std::string message("Experiments at ");
    message.append( name);

    ProgressBar show_progress( numQueries, message);

    //run queries
    timer.start();

    //for( unsigned int i = 0; i < numExperiments; ++i)
	while( numqueries < numQueries)
    {
		random = gen.getRandomNormalizedDouble();
        if( random > insert_percentage)
        {
            op = 1;
        }
        else
        {
            //std::cout << random << "\n";
            (random < insert_percentage / 2)? op=2: op=0;
        }
     
        switch ( op)
        {
        case 0:
            //std::cout <<"\nInsert\n";
			sourceId = gen.getRandomNormalizedDouble() * ids.size();
            if( sourceId == 0) ++sourceId;
            if( sourceId == ids.size()) --sourceId;
        	targetId = gen.getRandomNormalizedDouble() * ids.size();
            if( targetId == 0) ++targetId;
            if( targetId == ids.size()) --targetId;
			while( (sourceId == targetId) || G.hasEdge(ids[sourceId],ids[targetId]))
			{
				targetId = gen.getRandomNormalizedDouble() * ids.size();
                if( targetId == 0) ++targetId;
                if( targetId == ids.size()) --targetId;
			}

            ++insertions;
            desc = G.insertEdge( ids[sourceId], ids[targetId]);
            e = G.getEdgeIterator(desc);
            k = G.getInEdgeIterator(e);
            e->weight = std::numeric_limits<unsigned int>::max();
            k->weight = e->weight;
            insertedEdges.push_back(desc);
            break;
        case 1:
			//std::cout <<"\nQuery\n";
			sourceId = queries[numqueries].first * ids.size();
        	targetId = queries[numqueries].second * ids.size();

        	s = G.getNodeIterator( ids[sourceId]);
        	t = G.getNodeIterator( ids[targetId]); 

            ++numqueries;
            ++show_progress;
            astarSearch.runQuery( s, t);
            //dijkstra.runQuery( s, t);
            break;
        case 2:
			sourceId = gen.getRandomNormalizedDouble() * ids.size();
            if( sourceId == 0) ++sourceId;
            if( sourceId == ids.size()) --sourceId;
        	targetId = gen.getRandomNormalizedDouble() * ids.size();
            if( targetId == 0) ++targetId;
            if( targetId == ids.size()) --targetId;
			while( (sourceId == targetId) || G.hasEdge(ids[sourceId],ids[targetId]))
			{
				targetId = gen.getRandomNormalizedDouble() * ids.size();
                if( targetId == 0) ++targetId;
                if( targetId == ids.size()) --targetId;
			}
            //std::cout <<"\nErase\n";
            if( insertions > erasures)
			{
		       	++erasures;
			   	G.eraseEdge( insertedEdges.back());
				insertedEdges.pop_back(); 
			}
			else
			{
		 		++insertions;	
				desc = G.insertEdge( ids[sourceId], ids[targetId]);
            	e = G.getEdgeIterator(desc);
            	k = G.getInEdgeIterator(e);
            	e->weight = std::numeric_limits<unsigned int>::max();
            	k->weight = e->weight;
            	insertedEdges.push_back(desc);
            }
            break;
        }
        
    }

    std::cout << "\tInsertions: " << insertions << "\tErasures: " << erasures << "\tQueries: " << numqueries <<"\n";
    std::cout << "\tTime:\t" << timer.getElapsedTime() << "sec";
    std::cout << "\n\n";
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
    basePath = "/home/michai/Projects/Graphs/";
    //basePath = "/home/michai/Projects/Graphs/";

    unsigned int graphVariant = 0;
    unsigned int format = 0;
    std::string map ="luxembourg";

	AdjReader* reader = 0;
	PmaReader* pmaReader = 0;	
    FsReader* fsReader = 0;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("size,s", po::value< unsigned int>(), "number of queries. Default:100")
        ("graphtype,g", po::value< unsigned int>(), "graph type. All[0], Adjacency List[1], forward Star[2], Packed Memory Graph[3]. Default:0")
		("format,f", po::value< unsigned int>(), "map format. DIMACS10[0], DIMACS9[1]. Default:0")
        ("insertions,i", po::value< unsigned int>(), "number of insert/erase operations in thousands. Default: 1")
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

    if (vm.count("graphtype"))
    {
        graphVariant = vm["graphtype"].as<unsigned int>();
    }
    
    if (vm.count("insertions"))
    {
        unsigned int numInserts = 1000 * vm["insertions"].as<unsigned int>();
        insert_percentage = double(numInserts) / ( numQueries + numInserts);
    }

    if (vm.count("map"))
    {
        map = vm["map"].as<std::string>();
    }

    std::vector<unsigned int> results, pmaResults, fsResults;

    Graph G;
    pmaGraph pmaG;
    fsGraph  fsG;

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


	std::vector< std::pair<double,double> > queries;
    for( unsigned int i = 0; i < numQueries; i++)
    {
        queries.push_back( std::pair< double,double>( gen.getRandomNormalizedDouble(), gen.getRandomNormalizedDouble()));
    }


    Timer timer;
    switch( graphVariant)
    {
        case 1:
            timer.start();
            G.read(reader);
            std::cout << "Graph has " << (double)G.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            if(format == 0)
                calcWeights(G);           
            break;

        case 2:
            timer.start();
            fsG.read(fsReader);
            std::cout << "Graph has " << (double)fsG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            if(format == 0)
                calcWeights(fsG); 
            break;

        case 3:
            timer.start();
            pmaG.read(pmaReader);
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            if(format == 0)
                calcWeights(pmaG); 
            break;
    }
    

    switch( graphVariant)
    {
        case 1:
            runExperimentsAt( G, queries, reader->getIds(),  "Adjacency");
            break;
        case 2:
            runExperimentsAt( fsG, queries, fsReader->getIds(),  "FAS");
            break;
        case 3:
            runExperimentsAt( pmaG, queries, pmaReader->getIds(),  "PMA");
            break;
        /*default:
            runExperimentsAt( G, queries, reader->getIds(),  "Adjacency");
            runExperimentsAt( fsG, queries, fsReader->getIds(),  "FAS");
            runExperimentsAt( pmaG, queries, pmaReader->getIds(),  "PMA");
            break;
        */
    }

    //std::cout << "Graph has " << G.getNumEdges() << " edges" << std::endl;

    return 0;
}
