#include <iostream>
#include <fstream>
#include <Utilities/mersenneTwister.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/program_options.hpp>


namespace po = boost::program_options;

int main( int argc, char* argv[])
{
    std::string filename;
    unsigned int iterations = 1000;
    int min, max;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("file,f", po::value< std::string>(), "output file")
        ("iterations,i", po::value< unsigned int>(), "number of iterations. Default:1000")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if (vm.empty()) {
        std::cout << desc << "\n";
        return 0;
    }

    if (vm.count("file"))
    {
        filename = vm["file"].as<std::string>();
    }
    else
    {
        std::cout << "Output file was not set!\n";
        return(-1);
    }

    if (vm.count("iterations"))
    {
        iterations = vm["iterations"].as<unsigned int>();
    }

    MersenneTwister m;

    std::ofstream out(filename.c_str());
    for( unsigned int i = 0; i < iterations; i++)
    {
        out << m.getRandomNormalizedDouble() << "\n";
    }
    
    out.close();

    return 0;
}
