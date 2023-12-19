#include <iostream>
#include <utility>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <time.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

void fillQueries(std::vector< std::pair<unsigned int, unsigned int> > & queries, unsigned int numQueries, unsigned int numNodes);
void queriesWriter(const std::string& filename, unsigned int numQueries, unsigned int numNodes);
void queriesReader(const std::string& filename, std::vector< std::pair<unsigned int, unsigned int> > & queries);
void readNumGraphNodes(std::string graphFileName, unsigned int& numNodes);

int main(int argc, char** argv) 
{
    unsigned int numQueries = 1000, numNodes = 0;
    std::string basePath, map;

    basePath = std::string(getenv("HOME")) + "/Projects/Graphs/";
    map = "luxembourg";

    // Declare the supported opvoid readNumNodes(std::string graphFileName, unsigned int& numNodes)tions.
    po::options_description desc("Supported options");

    desc.add_options()
        ("queries,q", po::value<unsigned int>(), "number of queries.Default:1000")
        ("nodes,n", po::value<unsigned int>(), "number of nodes. The format-map parameters can be used instead.")
		("format,f", po::value< unsigned int>(), "map format. DIMACS10[0], DIMACS9[1]. Default:0")
        ("map,m", po::value< std::string>(), "input map. The name of the map to read. Default:'luxembourg'. Maps should reside in '$HOME/Projects/Graphs/' and should consist of 2 files, both with the same map name prefix, and suffixes 'osm.graph' and 'osm.xyz' in the case of DIMACS10 and, '.gr' and '.co' in the case of DIMACS9.")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if ( vm.empty()) 
    {
        std::cout << desc << "\n";
        return 0;
    }

    if ( vm.count("queries"))
    {
        numQueries = vm["queries"].as< unsigned int>();
    }

    if ( vm.count("map"))
    {
        map = vm["map"].as<std::string>();
    }

    if( vm.count("nodes"))
    {
        numNodes = vm["nodes"].as< unsigned int>();
        queriesWriter("./queries.dat", numQueries, numNodes);
    }

    else
    {

	    if ( vm.count("format") && vm["format"].as<unsigned int>() == 1)
        {
            basePath = basePath + "DIMACS9/" + map + "/";
            readNumGraphNodes(basePath + map + ".gr", numNodes);
        }

        else
        {
            basePath = basePath + "DIMACS10/" + map + "/";
            readNumGraphNodes(basePath + map + ".osm.graph", numNodes);
        }
    
        queriesWriter(basePath + "queries.dat", numQueries, numNodes);
    }

    return 0;
}


void readNumGraphNodes(std::string graphFileName, unsigned int& numNodes)
{
    std::string token;
    std::ifstream graphFile(graphFileName.c_str());

    if (!graphFile)
    {
        std::cerr << "\nerror: unable to open file [" << graphFileName << "]\n";
        exit(1);
    }

    while (getline(graphFile, token)) 
    {
        if( token.c_str()[0] == '%' || token.c_str()[0] == 'c')
            continue;

        else
        {
            std::stringstream graphinfo;

            //DIMACS9
            if( token.c_str()[0] == 'p')
            {    
                graphinfo.str(token);   
                graphinfo >> token >> token >> numNodes;  
            }

            //DIMACS10
            else
            {
                graphinfo.str(token);   
                graphinfo >> numNodes;  
            }

            break;
        }
    }

    std::cout << "\nnum nodes:" << numNodes << "\n";
}

void fillQueries(std::vector< std::pair<unsigned int, unsigned int> > & queries, unsigned int numQueries, unsigned int numNodes)
{
    unsigned int sourceID, targetID;
    boost::mt19937 gen(time(NULL));
    boost::uniform_real<> dist(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random(gen, dist);    

    if( numNodes < 2)
    {
        std::cerr << "\nwarning : number of nodes < 2\n";
        return;
    }

    for( unsigned int i = 0; i < numQueries; i++)
    {
        sourceID = (unsigned int) (random() * numNodes);
        do{ targetID = (unsigned int) (random() * numNodes); } while( sourceID == targetID);

        queries.push_back( std::pair<unsigned int, unsigned int>( sourceID, targetID));
    }
}


void queriesWriter(const std::string& filename, unsigned int numQueries, unsigned int numNodes)
{
    unsigned int sourceID, targetID;
    boost::mt19937 gen(time(NULL));
    boost::uniform_real<> dist(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random(gen, dist);    

    std::ofstream fileQueries(filename.c_str());

    if (!fileQueries)
    {
        std::cerr << "\nerror: unable to open file [" << filename << "]\n";
        exit(1);
    }

    fileQueries << "% number of queries\n";
    fileQueries << numQueries << "\n";

    //set the range (pact : min ID = 0 and max ID = numNodes-1)
    fileQueries << "% range\n";
    fileQueries << "0 " << (numNodes-1) << "\n";

    fileQueries << "% source node - target node\n";

    if( numNodes < 2)
    {
        std::cerr << "\nwarning : number of nodes < 2\n";
        return;
    }

    for( unsigned int i = 0; i < numQueries; i++)
    {
        sourceID = (unsigned int) (random() * numNodes);
        do{ targetID = (unsigned int) (random() * numNodes); } while( sourceID == targetID);
        fileQueries << sourceID << " " << targetID << "\n";
    }
}


void queriesReader(const std::string& filename, std::vector< std::pair<unsigned int, unsigned int> > & queries)
{
    std::ifstream fileQueries(filename.c_str());
    unsigned int numQueries, minNodeID, maxNodeID;
    unsigned int sourceID, targetID;

    if (!fileQueries)
    {
        std::cerr << "\nerror: unable to open file [" << filename << "]\n";
        exit(1);
    }

    //get and drop the first line (pact : single-line comment)
    fileQueries.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    fileQueries >> numQueries;
    fileQueries.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    //get and drop the third line (pact : single-line comment)
    fileQueries.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    fileQueries >> minNodeID >> maxNodeID;
    fileQueries.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    //get and drop the fifth line (pact : single-line comment)
    fileQueries.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    for(unsigned int i = 0; i < numQueries; i++)
    {
        fileQueries >> sourceID >> targetID;
        queries.push_back( std::pair<unsigned int, unsigned int>( sourceID, targetID) );
    }
}
