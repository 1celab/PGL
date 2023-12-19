#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <Structs/Maps/pmMap.h>
#include <Utilities/timer.h>
#include <Utilities/mersenneTwister.h>
#include <Utilities/progressBar.h>
#include <list>
#include <boost/program_options.hpp>


namespace po = boost::program_options;


int main( int argc, char* argv[])
{ 
    MersenneTwister gen;  

    //volatile unsigned int temp;

    unsigned int size = 1000;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("size,s", po::value< unsigned int>(), "map maximum size. Default:1000")
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
        size = vm["size"].as<unsigned int>();
    }
    

    PMMap< double, double> map;
    
    Timer timer; 
    std::string message("Inserting to map ");
    ProgressBar show_progress(size, message);
    timer.start();

    double random;
    for( unsigned int i = 0; i < size; i++)
    {
        random = gen.getRandomNormalizedDouble();
        map[random] = random;
        ++show_progress;
        //std::cout << "R: " << random << "\n";
    }
    std::cout << "Time:\t" << timer.getElapsedTime() << "sec\n";
    std::cout << "Size: " << map.size() << std::endl;
    
    /*for( PMMap< double, double>::Iterator it = map.begin(); it != map.end(); ++it)
    {
        std::cout << it->m_key << "," << it->m_data << "\n";
    }*/
    map.clear();
   
    return 0;
}
