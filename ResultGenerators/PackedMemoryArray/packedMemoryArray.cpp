#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <Structs/Arrays/packedMemoryArray.h>
#include <Utilities/timer.h>
#include <Utilities/mersenneTwister.h>
#include <Utilities/progressBar.h>
#include <list>
#include <boost/program_options.hpp>


namespace po = boost::program_options;

class wrapper
{
public:
	wrapper( unsigned int data = 0):m_data(data)
	{	
	}
	
	bool operator ==( const wrapper& other) const
	{
		return m_data == other.m_data;
	}
	
	bool operator !=( const wrapper& other) const
	{
		return m_data != other.m_data;
	}

    bool operator < (const wrapper& other) const
    {
        return m_data < other.m_data;
    }
    
    bool operator > (const wrapper& other) const
    {
        return m_data > other.m_data;
    }

    bool isFull() const
    {
        return (~m_data);
    }
	
	friend std::ostream& operator << ( std::ostream& out, wrapper other)
	{
		out << other.m_data;
		return out;
	}
	
	unsigned int m_data;
};


struct experiment
{
    unsigned int op;
    double randomPos;
};

template< typename Container>
void insertAt( Container& container, std::vector<double>::iterator begin, std::vector<double>::iterator end, const std::string& name)
{
    Timer timer; 
    unsigned int position; 
    std::string message("Inserting at ");
    message.append( name);
    ProgressBar show_progress(end-begin, message);
    timer.start();
    for( std::vector<double>::iterator it = begin; it != end; ++it)
    {
        position = (*it) * container.size();
        container.insert( container.begin() + position, wrapper(position));
        ++show_progress;
    }
    std::cout << "\tTime " << name << ":\t" << timer.getElapsedTime() << "sec\n\n";
}

void insertAt( std::list<wrapper>& container, std::vector<double>::iterator begin, std::vector<double>::iterator end, const std::string& name)
{
    Timer timer; 
    unsigned int position;
    std::string message("Inserting at ");
    message.append( name);
    ProgressBar show_progress(end-begin, message);
    timer.start();
    std::list<wrapper>::iterator lit;
    for( std::vector<double>::iterator it = begin; it != end; ++it)
    {
        position = (*it) * container.size();

        lit = container.begin();
        std::advance(lit, position);
        container.insert( lit, wrapper(position));
        ++show_progress;
    }
    std::cout << "\tTime " << name << ":\t" << timer.getElapsedTime() << "sec\n\n";
}

template< typename Container>
void eraseAt( Container& container, std::vector<double>::iterator begin, std::vector<double>::iterator end, const std::string& name)
{
    Timer timer; 
    unsigned int position;
    std::string message("Erasing at ");
    message.append( name);
    ProgressBar show_progress(end-begin, message);
    timer.start();
    for( std::vector<double>::iterator it = begin; it != end; ++it)
    {
        position = (*it) * container.size();
        container.erase( container.begin() + position);
        ++show_progress;
    }
    std::cout << "\tTime " << name << ":\t" << timer.getElapsedTime() << "sec\n\n";
}

void eraseAt( std::list<wrapper>& container, std::vector<double>::iterator begin, std::vector<double>::iterator end, const std::string& name)
{
    Timer timer; 
    unsigned int position;
    std::string message("Erasing at ");
    message.append( name);
    ProgressBar show_progress(end-begin, message);
    timer.start();
    std::list<wrapper>::iterator lit;
    for( std::vector<double>::iterator it = begin; it != end; ++it)
    {
        position = (*it) * container.size();
        lit = container.begin();
        std::advance(lit, position);
        container.erase( lit);
        ++show_progress;
    }
    std::cout << "\tTime " << name << ":\t" << timer.getElapsedTime() << "sec\n\n";
}


template< typename Container, typename Iterator>
void experimentsAt( Container& container, std::vector<std::pair<unsigned int,double> >::iterator begin, std::vector<std::pair<unsigned int,double> >::iterator end, const std::string& name)
{
    Timer timer; 
    unsigned int position;
    std::string message("Experiments at ");
    message.append( name);
    ProgressBar show_progress(end-begin, message);
    timer.start();
    
    unsigned int insertions = 0;
    unsigned int erasures = 0;
    unsigned int scans = 0;

    for( std::vector<std::pair<unsigned int,double> >::iterator it = begin; it != end; ++it)
    {
        position = (it->second) * container.size();
        
        switch ( it->first)
        {
        case 0:
            //std::cout <<"\nInsert\n";
            ++insertions;
            container.insert( container.begin() + position, wrapper(position));
            break;
        case 1:
            ++scans;
            break;
        case 2:
            //std::cout <<"\nErase\n";
            ++erasures;
            container.erase( container.begin() + position);
            break;
        }
        ++show_progress;
    }
    std::cout << "\tInsertions: " << insertions << "\tErasures: " << erasures << "\tScans: " << scans <<"\n";
    std::cout << "\tTime " << name << ":\t" << timer.getElapsedTime() << "sec\n\n";
}

template< typename PMA, typename Vector>
bool checkValidity( PMA& p, Vector& v)
{
    std::cout << "\nChecking validity...";
    unsigned int i = 0;
    bool failed = false;
    
    for( typename PMA::Iterator it = p.begin(); it != p.end(); ++it)
    {
        if( (*it) != v[i])
        {
            failed = true;
            break;
        }  
        ++i;
    }

    i = 0;
    for( typename Vector::iterator it = v.begin(); it != v.end(); ++it)
    {
        if( (*it) != *(p.begin()+i))
        {
            failed = true;
            break;
        }  
        ++i;
    }

    if( failed)
    {
        std::cout << "Failure at " << i << "!\n";
    }
    else
    {
        std::cout << "Success!\n";
    }

    return !failed;
}

template< typename PMA>
void printStats( PMA& array)
{
    std::cout << "\tCapacity: " << array.capacity();
    std::cout << "\tExpected Memory Usage: " << double(array.capacity()) * sizeof(unsigned int) /1048576 << "Mb";
	unsigned int memUsage = array.getMemoryUsage();
    std::cout << "\t\tActual Memory Usage: " << double( memUsage)/1048576 << "Mb";
    std::cout << "\tRatio: " << ( double( memUsage ))/( double(array.capacity()) * sizeof(unsigned int) ) * 100 << "%\n";
}




int main( int argc, char* argv[])
{ 
    MersenneTwister gen;  

    //volatile unsigned int temp;

    unsigned int iterations = 1000;
    unsigned int size = 1000;
    double insert_percentage = 0.01;
    unsigned int range = 100;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("size,s", po::value< unsigned int>(), "array maximum size. Default:10000")
        ("iterations,i", po::value< unsigned int>(), "number of iterations. Default:1000")
        ("percentage,p", po::value< double>(), "percentage of insert/erase operations relative to scan operations. Default:0.01")
        ("range,r", po::value< unsigned int>(), "range of scan operations. Default:100")
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
    
    if (vm.count("iterations"))
    {
        iterations = vm["iterations"].as<unsigned int>();
    }
    
    if (vm.count("percentage"))
    {
        insert_percentage = vm["percentage"].as<double>();
    }

    if (vm.count("range"))
    {
        range = vm["range"].as<unsigned int>();
    }


    PackedMemoryArray< wrapper> array;
    std::vector< wrapper> vector;
    std::list< wrapper> list;

    std::vector<double> insertions;
    for( unsigned int i = 0; i < size; i++)
    {
        insertions.push_back(gen.getRandomNormalizedDouble());
    }


    double random;
    unsigned int op;
    std::vector<std::pair< unsigned int, double> > experiments;
    for( unsigned int i = 0; i < iterations; i++)
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
        experiments.push_back( std::pair< unsigned int, double> ( op,gen.getRandomNormalizedDouble()));
    }

    //array.reserve(size);
    vector.reserve(size);

    std::cout << "\n***Insertion\n" << std::endl;
    insertAt( array, insertions.begin(), insertions.end(), "Pma");    
    insertAt( vector, insertions.begin(), insertions.end(), "Vector");
    //insertAt( list, insertions.begin(), insertions.end(), "List");
    printStats(array);
    
    if( !checkValidity(array,vector))
    {
        return -1;
    }


    std::cout << "\n***Experiments\n" << std::endl;
    experimentsAt<PackedMemoryArray< wrapper>, PackedMemoryArray< wrapper>::Iterator>( array, experiments.begin(), experiments.end(), "Pma");
    experimentsAt<std::vector< wrapper>, std::vector< wrapper>::iterator>( vector, experiments.begin(), experiments.end(), "Vector");


    if( !checkValidity(array,vector))
    {
        return -1;
    }

    while( array.size() < insertions.size())
    {
       insertions.pop_back();
    }

    std::cout << "\n***Erasure\n" << std::endl;
    eraseAt( array, insertions.begin(), insertions.end(), "Pma");
    eraseAt( vector, insertions.begin(), insertions.end(), "Vector");
    //eraseAt( list, insertions.begin(), insertions.end(), "List");

    if( !checkValidity(array,vector))
    {
        return -1;
    }


    

 /*    
    std::cout << "\n***Scanning" << std::endl;

    
    timer.start();
	in.open( numbersFile.c_str());
   
    PackedMemoryArray< wrapper>::Iterator aEnd = array.end();
    for( unsigned int i = 0; i < scan_iterations; i++)
    {
        for( PackedMemoryArray< wrapper>::Iterator it = array.begin(); it != aEnd; ++it)
        {
            temp = (*it).m_data;
        }
        if( i % (scan_iterations/100) == 0)
        {
            std::cout << "\rScanning PMA..." << i / (scan_iterations/100) << "%" << std::flush;
        }
    }	
    std::cout << "\rScanning PMA...done!\n";	
		
	in.close();
    std::cout << "\tTime PMA:\t" << timer.getElapsedTime() << "sec\n";
    
    timer.start();
    in.open( numbersFile.c_str());
	std::vector< unsigned int>::iterator vEnd = vector.end();
    for( unsigned int i = 0; i < scan_iterations; i++)
    {	
        for( std::vector< unsigned int>::iterator it = vector.begin(); it != vEnd; ++it)
        {
            temp = *it;
        }
        if( i % (scan_iterations/100) == 0)
        {
            std::cout << "\rScanning VECTOR..." << i / (scan_iterations/100) << "%" << std::flush;
        }
    }	
    std::cout << "\rScanning VECTOR...done!\n";	
	
	in.close();
    std::cout << "\tTime Vector:\t" << timer.getElapsedTime() << "sec\n";

    std::cout << std::endl;
    std::ofstream out("out.dot");
    array.printDot( out);
    out.close();*/

    return 0;
}
