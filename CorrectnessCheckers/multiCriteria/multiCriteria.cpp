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

namespace po = boost::program_options;
MersenneTwister gen; 

struct node: DefaultGraphItem
{
	node( unsigned int data = 0): timestamp(0),heuristicList(2)
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

   /* void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "lon" << std::setprecision(10) << valueDelimiter << lon << propertyDelimiter;
        out << "lat" << std::setprecision(10) << valueDelimiter << lat << propertyDelimiter;
    }
    
    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"lon\":\"" << lon << "\"," << "\"lat\":\"" << lat << "\"";
    }*/

    unsigned int x, y;
    std::vector<Label> labels;
	unsigned int timestamp;
	CriteriaList heuristicList;
    /*unsigned int m_data;
    unsigned int dist;
    DynamicGraph< AdjacencyListImpl, wrapper, wrapper>::NodeDescriptor m_pred;
    unsigned int weight;*/
    unsigned int pqitem;
    unsigned int selectionID;
};


struct edge: DefaultGraphItem
{
	edge( unsigned int data = 0): criteriaList(2)
	{	
	}

   

    /*void writeProperties( std::ofstream& out, const std::string& propertyDelimiter = "\n", const std::string& valueDelimiter = " ")
    {
        DefaultGraphItem::writeProperties( out, propertyDelimiter, valueDelimiter);
        out << "weight" << valueDelimiter << weight << propertyDelimiter;
    }

    void writeJSON( std::ofstream& out)
    {
        DefaultGraphItem::writeJSON( out);
        out << ",\"weight\":" << weight << "";
    }*/

    CriteriaList criteriaList;
    unsigned int flags;
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
	unsigned int min_speed = 50;
	unsigned int max_speed = 90;

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
            
            e->criteriaList[0] = euclideanDistance( u->x, u->y, v->x, v->y);
            k->criteriaList[0] = e->criteriaList[0];

			unsigned int speed = (unsigned int)(gen.getRandomNormalizedDouble() * ( max_speed - min_speed) + min_speed);
            e->criteriaList[1] = (unsigned int) ceil(  (double(3.6)*e->criteriaList[0])/speed );
            k->criteriaList[1] = e->criteriaList[1];

			//e->criteriaList[1] = e->criteriaList[0];
            //k->criteriaList[1] = k->criteriaList[0];

            if( e->criteriaList[0] > max)
            {
                max_ux = u->x;
                max_uy = u->y;  
                max_vx = v->x;
                max_vy = v->y;
                u_id = G.getRelativePosition(u);
                v_id = G.getRelativePosition(v);
                max = e->criteriaList[0];
            }
            ++edge_progress;
            //std::cout << u->lat << " " << v->lat << std::endl;
        }
    }

    std::cout << "\tMax weight: " << max << "\tTime:\t" << timer.getElapsedTime() << std::endl;
    //std::cout << "( " << max_ux << ", " << max_uy << ") " << u_id << "->" << v_id << " ( " << max_vx << ", " << max_vy << ")\n";
}

template< class DijkstraVariant, typename GraphType>
void runQueries( GraphType& G, std::vector< std::pair<double,double> >& queries, std::vector<typename GraphType::NodeDescriptor>& ids, std::vector<unsigned int>& results, const std::string& name)
{
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
    results.clear();

    //create dijkstra
    DijkstraVariant dijkstra(G, 2, &timestamp);
    
    //create output message
    std::string message("Experiments at ");
    message.append( name);
    ProgressBar show_progress( queries.size(), message);

    //run queries
    double numLabels = 0;
    Timer timer; 
    timer.start();
    for( std::vector< std::pair<double,double> >::iterator it = queries.begin(); it != queries.end(); ++it)
    {
        sourceId = it->first * ids.size();
        targetId = it->second * ids.size();
        //std::cout << sourceId << "->" << targetId << std::endl;
        s = G.getNodeIterator( ids[sourceId]);
        t = G.getNodeIterator( ids[targetId]);
        dijkstra.init(s,t);
		//multiCriteriaDijkstra( G, 2, s);
        dijkstra.runQuery( s, t);
		std::cout << t->labels.size() << std::endl;		
		//t->printLabels( std::cout, G);
        //results.push_back(t->dist);
        numLabels += ((double)dijkstra.getGeneratedLabels())/queries.size();
        ++show_progress;
    }

    std::cout << "\tTime:\t" << timer.getElapsedTime() << "sec ( " << (1000 * timer.getElapsedTime() / queries.size()) << "ms per query)";
    std::cout << "\tGenerated labels: " << numLabels << " ( " << numLabels/G.getNumNodes() << " per node)\n\n";
}

template< typename GraphType>
void runExperimentsAt( GraphType& G, std::vector< std::pair<double,double> >& queries, std::vector<typename GraphType::NodeDescriptor>& ids, std::vector<unsigned int>& results, const std::string& name, const unsigned int& dijkstraVariant)
{
    switch( dijkstraVariant)
    {
        case 1:
            runQueries<MulticriteriaDijkstra<GraphType> >( G, queries, ids, results, name + " with Multi Criteria Dijkstra");
            break;
        case 2:
			runQueries<NamoaStarDijkstra<GraphType,GreatCircleHeuristic> >( G, queries, ids, results, name + " with NAMOA* Dijkstra");
            break;
        case 3:
            runQueries<NamoaStarArc<GraphType,GreatCircleHeuristic> >( G, queries, ids, results, name + " with Multicriteria Arc Flags Dijkstra");
            break;
        default:
            runQueries<MulticriteriaDijkstra<GraphType> >( G, queries, ids, results, name + " with Multi Criteria Dijkstra");
			runQueries<NamoaStarDijkstra<GraphType,GreatCircleHeuristic> >( G, queries, ids, results, name + " with NAMOA* Dijkstra");
            break;
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

    std::string basePath = "/home/michai/Projects/Graphs/DIMACS10/";

    unsigned int numQueries = 100;
    unsigned int dijkstraVariant = 0;
    unsigned int graphVariant = 0;
    std::string map ="luxembourg";

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("size,s", po::value< unsigned int>(), "number of queries. Default:100")
        ("dijkstra,d", po::value< unsigned int>(), "dijkstra algorithm. All[0], Plain[1], A*[2], Bidirectional[3]. Default:0")
        ("graphtype,g", po::value< unsigned int>(), "graph type. All[0], Adjacency List[1], Packed Memory Graph[2]. Default:0")
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

    DIMACS10Reader<Graph> reader( basePath + map + ".osm.graph",
                                basePath + map + ".osm.xyz");
    
    DIMACS10Reader<pmaGraph> pmaReader( basePath + map + ".osm.graph",
                                basePath + map + ".osm.xyz");


    Timer timer;
    switch( graphVariant)
    {
        
        case 1:
            timer.start();
            G.read(&reader);
            std::cout << "Graph has " << (double)G.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            calcWeights(G);
            break;
        case 2:
            timer.start();
            pmaG.read(&pmaReader);
            pmaG.compress();
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            calcWeights(pmaG);
            break;
        default:
            timer.start();
            G.read(&reader);
            std::cout << "Graph has " << (double)G.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            calcWeights(G);
            timer.start();
            pmaG.read(&pmaReader);
            pmaG.compress();
            std::cout << "Graph has " << (double)pmaG.memUsage()/1048576 << " Mbytes. Time spent to read:\t" << timer.getElapsedTime() << "sec" << std::endl;
            calcWeights(pmaG);
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
            runExperimentsAt( G, queries, reader.getIds(), results, "Adjacency", dijkstraVariant);
            break;
        case 2:
            runExperimentsAt( pmaG, queries, pmaReader.getIds(), pmaResults, "PMA", dijkstraVariant);
            break;
        default:
            runExperimentsAt( G, queries, reader.getIds(), results, "Adjacency", dijkstraVariant);
            runExperimentsAt( pmaG, queries, pmaReader.getIds(), pmaResults, "PMA", dijkstraVariant);
            break;
    }

    return 0;
}
