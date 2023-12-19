#ifndef GRAPHIO_H
#define GRAPHIO_H

#include <vector>
#include <map>
#include <stack>
#include <unordered_map>
#include <Utilities/progressBar.h>
#include <Structs/Arrays/nodeArray.h>
#include <typeinfo>
#include <assert.h>
#include <sstream>
#include <fstream>

//--------------------------------------- READERS --------------------------------------//

template<typename GraphType>
class GraphReader
{

 public:

    GraphReader( const std::string& filename=""):m_filename(filename)
    {}

    std::vector< typename GraphType::NodeDescriptor>& getNodeIds()
    {
        return m_ids;
    }

    virtual void read( GraphType& G)
    {}

 protected:

    std::string m_filename;
    std::vector< typename GraphType::NodeDescriptor> m_ids;
};


template<typename GraphType>
class CSVReader : public GraphReader<GraphType>
{

public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::InEdgeIterator  InEdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    class EdgeList
    {

     public:

        EdgeList( unsigned int init = 0): source( 0), target(0)
        {}

        EdgeList( const SizeType& s, const SizeType& t): source( s), target( t)
        {}

        bool operator == ( const EdgeList& other)
        {
            return ( source == other.source) && ( target == other.target);
        }

        bool operator != ( const EdgeList& other)
        {
            return ( source != other.source) || ( target != other.target);
        }

        SizeType source;
        SizeType target;
    };

    CSVReader( const std::string& nodeFilename, const std::string& edgeFilename, unsigned int dataFlag = 1) :
    GraphReader<GraphType>(""), m_nodeFilename(nodeFilename), m_edgeFilename(edgeFilename), m_dataFlag(dataFlag)
    {}

    void read( GraphType& G)
    {
        initGraph( G);
        importNodes( G);

        const std::string pmg("PackedMemoryArrayImpl");
        const std::string graphType( typeid(G).name());

        //PMG
        if( graphType.find(pmg) != std::string::npos)
            importSortedEdges( G);

        //ADJ, dynFS
        else
           // importEdges( G);
           importSortedEdges( G);
    }

    void importNodes( GraphType& G)
    {
        std::ifstream in;
        SizeType numNodes;
        float x, y;

        NodeIterator u, end;

        GraphReader<GraphType>::m_ids.clear();
        in.exceptions( std::ifstream::failbit | std::ifstream::badbit );

        //import the nodes
        try
        {
            in.open(m_nodeFilename.c_str());

            std::string token;
            std::stringstream data;

            getline( in, token);
            data.str( token);
            data >> numNodes;
            data.clear();
            data.str( std::string());

            std::cout << "\nReading CSV from " << m_nodeFilename << std::endl;
            std::cout << "\tReserving " << numNodes << " IDs\n";

            //DIMACS{9|10}, CSV pact : id=[1, numNodes]
            GraphReader<GraphType>::m_ids.reserve( numNodes + 1);

            GraphReader<GraphType>::m_ids.push_back( 0);

            ProgressStream node_progress( numNodes);
            node_progress.label() << "\tReading " << numNodes << " nodes";

            for( SizeType i = 0; i < numNodes; ++i)
                G.insertNode();

            for( u = G.beginNodes(), end = G.endNodes(); u != end; ++u)
            {
                getline( in,token);
                data.str( token);
                data >> x >> y;
                data.clear();
                data.str( std::string());

                GraphReader<GraphType>::m_ids.push_back( G.getNodeDescriptor(u));
                u->x = x;
                u->y = y;

                ++node_progress;
            }

            in.close();
        }

        catch (std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_nodeFilename << "'\n";
            throw e;
        }
    }

    void importSortedEdges( GraphType& G)
    {
        std::ifstream in;
        SizeType counter, numEdges;
        SizeType startNodeID, endNodeID;
        double distance, travelTime;

        EdgeIterator e;
        InEdgeIterator k;
        EdgeDescriptor eD;

        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

        //import the edges
        try
        {
            in.open( m_edgeFilename.c_str());

            std::string token;
            std::stringstream data;

            getline( in, token);
            data.str( token);
            data >> numEdges;
            data.clear();
            data.str( std::string());

            ProgressStream sort_progress( numEdges);
            PackedMemoryArray<EdgeList> edgeLists;
            edgeLists.reserve( numEdges);

            sort_progress.label() << "\tSorting " << numEdges << " edges";
            counter = numEdges;

            while( counter > 0 && getline( in, token))
            {
                data.str( token);
                data >> startNodeID >> endNodeID;
                data.clear();
                data.str( std::string());

                if( startNodeID == 0 || endNodeID == 0) {std::cout << "\n>" << startNodeID << " " << endNodeID; exit(1);}

                edgeLists.optimalInsert( EdgeList( startNodeID, endNodeID));
                ++sort_progress;
                --counter;
            }


            ProgressStream edge_progress( 2 * numEdges);
            edge_progress.label() << "\tReading " << numEdges << " edges";

            typename PackedMemoryArray<EdgeList>::Iterator it;
            while( !edgeLists.empty())
            {
                it = edgeLists.chooseCell();

                if( it->source == 0 || it->target == 0) //TODO edgeLists
                    continue;

                if( !G.hasEdge( id2Desc( it->source), id2Desc( it->target)))
                {
                    eD = G.insertEdge( id2Desc(it->source), id2Desc( it->target));
                    e = G.getEdgeIterator( eD);
                    e->weight = std::numeric_limits<unsigned int>::max();
                }

                ++edge_progress;
                edgeLists.erase(it);
            }


            //go to the begin of file
            in.seekg ( 0, std::ios::beg);

            //read first line (numEdges)
            getline( in, token);
            data.str( token);
            data >> numEdges;
            data.clear();
            data.str( std::string());
            counter = numEdges;

            if( m_dataFlag == 1)
                while( counter > 0 && getline( in, token))
                {
                    data.str( token);
                    data >> startNodeID >> endNodeID >> distance;
                    data.clear();
                    data.str( std::string());

                    e = G.getEdgeIterator( id2Desc( startNodeID), id2Desc( endNodeID));
                    k = G.getInEdgeIterator( e);
                    //e->distance = e->weight = (e->weight > distance ? distance : e->weight);
                    e->weight = (e->weight > distance ? distance : e->weight);
                    //k->distance = k->weight = e->weight;
                    k->weight = e->weight;

                    ++edge_progress;
                    --counter;
                }


            else if( m_dataFlag == 2)
                while( counter > 0 && getline( in, token))
                {
                    data.str( token);
                    data >> startNodeID >> endNodeID >> distance >> travelTime;
                    data.clear();
                    data.str( std::string());

                    e = G.getEdgeIterator( id2Desc( startNodeID), id2Desc( endNodeID));
                    k = G.getInEdgeIterator( e);
                    e->weight = (e->weight > travelTime ? travelTime : e->weight);
                    k->weight = e->weight;

                    ++edge_progress;
                    --counter;
                }

            /*
            else
                while( counter > 0 && getline( in, token))
                {
                    data.str( token);
                    data >> startNodeID >> endNodeID >> distance >> travelTime;
                    data.str( std::string());

                    e = G.getEdgeIterator( id2Desc( startNodeID), id2Desc( endNodeID));
                    k = G.getInEdgeIterator( e);
                    e->distance = distance;
                    e->travelTime = travelTime;
                    k->distance = distance;
                    k->travelTime = travelTime;

                    ++edge_progress;
                    --counter;
                }
            */

            in.close();

        }

        catch ( std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_edgeFilename << "'\n";
            throw e;
        }
    }

    void importEdges( GraphType& G)
    {
        std::ifstream in;
        SizeType counter, numEdges;
        unsigned int startNodeID, endNodeID;
        float distance, travelTime;

        EdgeIterator e;
        InEdgeIterator k;
        EdgeDescriptor eD;

        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

        //import the edges
        try
        {
            in.open( m_edgeFilename.c_str());

            std::string token;
            std::stringstream data;

            getline( in, token);
            data.str( token);
            data >> numEdges;
            data.clear();
            data.str( std::string());

            ProgressStream edge_progress( numEdges);
            edge_progress.label() << "\tReading " << numEdges << " edges";
            counter = numEdges;

            if( m_dataFlag == 1)
                while( counter > 0 && getline( in, token))
                {
                    data.str( token);

                    data >> startNodeID >> endNodeID >> distance;
                    data.clear();
                    data.str( std::string());

                    if( G.hasEdge( id2Desc( startNodeID), id2Desc( endNodeID)))
                    {
                        e = G.getEdgeIterator( id2Desc( startNodeID), id2Desc( endNodeID));
                        k = G.getInEdgeIterator( e);
                        e->weight = (e->weight > distance ? distance : e->weight);
                        k->weight = e->weight;
                    }
                    else
                    {
                        eD = G.insertEdge( id2Desc( startNodeID), id2Desc( endNodeID));
                        e = G.getEdgeIterator( eD);
                        k = G.getInEdgeIterator( e);

                        e->weight = distance;
                        k->weight = distance;
                    }

                    ++edge_progress;
                    --counter;
                }

            else if( m_dataFlag == 2)
                while( counter > 0 && getline( in, token))
                {
                    data.str( token);
                    data >> startNodeID >> endNodeID >> distance >> travelTime;
                    data.clear();
                    data.str( std::string());

                    if( G.hasEdge( id2Desc( startNodeID), id2Desc( endNodeID)))
                    {
                        e = G.getEdgeIterator( id2Desc( startNodeID), id2Desc( endNodeID));
                        k = G.getInEdgeIterator( e);
                        e->weight = (e->weight > travelTime ? travelTime : e->weight);
                        k->weight = e->weight;
                    }
                    else
                    {
                        eD = G.insertEdge( id2Desc( startNodeID), id2Desc( endNodeID));
                        e = G.getEdgeIterator( eD);
                        k = G.getInEdgeIterator( e);

                        e->weight = travelTime;
                        k->weight = travelTime;
                    }

                    ++edge_progress;
                    --counter;
                }

            /*
            else
                while( counter > 0 && getline( in, token))
                {
                    data.str( token);
                    data >> startNodeID >> endNodeID >> distance >> travelTime;
                    data.str( std::string());

                    eD = G.insertEdge( id2Desc( startNodeID), id2Desc( endNodeID));
                    e = G.getEdgeIterator( eD);
                    k = G.getInEdgeIterator( e);

                    e->distance = distance;
                    e->travelTime = travelTime;

                    k->distance = distance;
                    k->travelTime = travelTime;

                    ++edge_progress;
                    --counter;
                }
            */

            in.close();

        }

        catch (std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_edgeFilename << "'\n";
            throw e;
        }
    }

    void initGraph( GraphType& G)
    {
        std::string token;
        std::stringstream data;
        SizeType numNodes, numEdges;

        std::ifstream in;
        in.exceptions( std::ifstream::failbit | std::ifstream::badbit );

        try{

            in.open( m_nodeFilename.c_str());
            getline( in,token);
            data.str( token);
            data >> numNodes;
            data.clear();
            data.str( std::string());
            in.close();


            in.open( m_edgeFilename.c_str());
            getline( in,token);
            data.str( token);
            data >> numEdges;
            data.clear();
            data.str( std::string());
            in.close();

            G.reserve( numNodes, numEdges);
        }

        catch ( std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_nodeFilename << "/" << m_edgeFilename << "'\n";
            throw e;
        }
    }

    NodeDescriptor id2Desc( SizeType id)
    {
        return GraphReader<GraphType>::m_ids[id];
    }

 private:

    std::string m_nodeFilename;
    std::string m_edgeFilename;
    unsigned int m_dataFlag;
};


template<typename GraphType>
class CSVDoubleReader : public GraphReader<GraphType>
{

public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::InEdgeIterator  InEdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    class EdgeList
    {
    public:
        EdgeList( unsigned int init = 0): source( 0), target(0)
        {}

        EdgeList( const SizeType& s, const SizeType& t): source( s), target( t)
        {}

        bool operator == ( const EdgeList& other)
        {
            return ( source == other.source) && ( target == other.target);
        }

        bool operator != ( const EdgeList& other)
        {
            return ( source != other.source) || ( target != other.target);
        }

        SizeType source;
        SizeType target;
    };


    CSVDoubleReader(const std::string& nodeFilename, const std::string& edgeFilename) :
    GraphReader<GraphType>(""), m_nodeFilename(nodeFilename), m_edgeFilename(edgeFilename)
    {}

    void read( GraphType& G)
    {
        initGraph( G);

        importNodes( G);

        //recommended for ADJ, Forward star
        //importEdges( G);

        //recommended for PMG
        importSortedEdges( G);
    }

    void importNodes( GraphType& G)
    {
        std::ifstream in;
        SizeType numNodes;
        float x, y;

        NodeIterator u, end;

        GraphReader<GraphType>::m_ids.clear();
        in.exceptions( std::ifstream::failbit | std::ifstream::badbit );

        //import the nodes
        try
        {
            in.open(m_nodeFilename.c_str());

            std::string token;
            std::stringstream data;

            getline( in, token);
            data.str( token);
            data >> numNodes;
            data.clear();
            data.str( std::string());

            std::cout << "\nReading TGF from " << m_nodeFilename << std::endl;
            std::cout << "\tReserving " << numNodes << " IDs\n";

            //DIMACS{9|10}, TGF pact : id=[1, numNodes]
            GraphReader<GraphType>::m_ids.reserve( numNodes + 1);

            GraphReader<GraphType>::m_ids.push_back( 0);

            ProgressStream node_progress( numNodes);
            node_progress.label() << "\tReading " << numNodes << " nodes";

            for( SizeType i = 0; i < numNodes; ++i)
                G.insertNode();

            for( u = G.beginNodes(), end = G.endNodes(); u != end; ++u)
            {
                getline( in,token);
                data.str( token);
                data >> x >> y;
                data.clear();
                data.str( std::string());

                GraphReader<GraphType>::m_ids.push_back( G.getNodeDescriptor(u));
                u->x = x;
                u->y = y;

                ++node_progress;
            }

            in.close();
        }

        catch (std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_nodeFilename << "'\n";
            throw e;
        }
    }

    void importSortedEdges( GraphType& G)
    {
        std::ifstream in;
        SizeType counter, numEdges;
        unsigned int startNodeID, endNodeID;
        float distance, travelTime;

        EdgeIterator e;
        InEdgeIterator k;
        EdgeDescriptor eD;

        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

        //import the edges
        try
        {
            in.open( m_edgeFilename.c_str());

            std::string token;
            std::stringstream data;

            getline( in, token);
            data.str( token);
            data >> numEdges;
            data.clear();
            data.str( std::string());

            ProgressStream sort_progress( numEdges);
            PackedMemoryArray<EdgeList> edgeLists;
            edgeLists.reserve( numEdges);

            sort_progress.label() << "\tSorting " << numEdges << " edges";
            counter = numEdges;

            while( counter > 0 && getline( in, token))
            {
                data.str( token);
                data >> startNodeID >> endNodeID;
                data.clear();
                data.str( std::string());

                edgeLists.optimalInsert( EdgeList( startNodeID, endNodeID));
                ++sort_progress;
                --counter;
            }

            ProgressStream edge_progress( 2 * numEdges);
            edge_progress.label() << "\tReading " << numEdges << " edges";

            typename PackedMemoryArray<EdgeList>::Iterator it;
            while( !edgeLists.empty())
            {
                it = edgeLists.chooseCell();

                if( it->source == 0 || it->target == 0)
                    continue;

                if( !G.hasEdge( id2Desc( it->source), id2Desc( it->target)))
                {
                    eD = G.insertEdge( id2Desc(it->source), id2Desc( it->target));
                    e = G.getEdgeIterator( eD);
                }

                ++edge_progress;
                edgeLists.erase(it);
            }

            //go to the begin of file
            in.seekg ( 0, std::ios::beg);
            //remove first line (numEdges)
            getline( in, token);
            data.str( token);
            data >> numEdges;
            data.clear();
            data.str( std::string());
            counter = numEdges;

            while( counter > 0 && getline( in, token))
            {
                data.str( token);
                data >> startNodeID >> endNodeID >> distance >> travelTime;
                data.clear();
                data.str( std::string());

                e = G.getEdgeIterator( id2Desc( startNodeID), id2Desc( endNodeID));
                k = G.getInEdgeIterator( e);

                e->distance = distance;
                e->travelTime = travelTime;
                k->distance = distance;
                k->travelTime = travelTime;

                ++edge_progress;
                --counter;
            }

            in.close();

        }

        catch ( std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_edgeFilename << "'\n";
            throw e;
        }
    }

    void importEdges( GraphType& G)
    {
        std::ifstream in;
        SizeType counter, numEdges;
        unsigned int startNodeID, endNodeID;
        float distance, travelTime;

        EdgeIterator e;
        InEdgeIterator k;
        EdgeDescriptor eD;

        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

        //import the edges
        try
        {
            in.open( m_edgeFilename.c_str());

            std::string token;
            std::stringstream data;

            getline( in, token);
            data.str( token);
            data >> numEdges;
            data.clear();
            data.str( std::string());

            ProgressStream edge_progress( numEdges);
            edge_progress.label() << "\tReading " << numEdges << " edges";
            counter = numEdges;

            while( counter > 0 && getline( in, token))
            {
                data.str( token);
                data >> startNodeID >> endNodeID >> distance >> travelTime;
                data.clear();
                data.str( std::string());

                eD = G.insertEdge( id2Desc( startNodeID), id2Desc( endNodeID));
                e = G.getEdgeIterator( eD);
                k = G.getInEdgeIterator( e);

                e->distance = distance;
                e->travelTime = travelTime;

                k->distance = distance;
                k->travelTime = travelTime;

                ++edge_progress;
                --counter;
            }

            in.close();
        }

        catch (std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_edgeFilename << "'\n";
            throw e;
        }
    }

    void initGraph( GraphType& G)
    {
        std::string token;
        std::stringstream data;
        SizeType numNodes, numEdges;

        std::ifstream in;
        in.exceptions( std::ifstream::failbit | std::ifstream::badbit );

        try{

            in.open( m_nodeFilename.c_str());
            getline( in,token);
            data.str( token);
            data >> numNodes;
            data.clear();
            data.str( std::string());
            in.close();


            in.open( m_edgeFilename.c_str());
            getline( in,token);
            data.str( token);
            data >> numEdges;
            data.clear();
            data.str( std::string());
            in.close();

            G.reserve( numNodes, numEdges);
        }

        catch ( std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_nodeFilename << "/" << m_edgeFilename << "'\n";
            throw e;
        }
    }

    NodeDescriptor id2Desc( SizeType id)
    {
        return GraphReader<GraphType>::m_ids[id];
    }

 private:

    std::string m_nodeFilename;
    std::string m_edgeFilename;
};


template<typename GraphType>
class TDCSVReader: public GraphReader<GraphType>
{

public:

    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::InEdgeIterator  InEdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    class EdgeList
    {

    public:

        EdgeList( unsigned int init = 0): source(0), target(0)
        {}

        EdgeList( const SizeType& s, const SizeType& t): source(s), target(t)
        {
        }

        bool operator == ( const EdgeList& other)
        {
            return (source == other.source) && (target == other.target);
        }

        bool operator != ( const EdgeList& other)
        {
            return (source != other.source) || (target != other.target);
        }

        SizeType source;
        SizeType target;
    };

    TDCSVReader( const std::string& nodeFilename, const std::string& edgeLengthFilename,
                 const std::string& edgeSpeedFilename) :
                 GraphReader<GraphType>(""), m_nodeFilename(nodeFilename), m_edgeLengthFilename(edgeLengthFilename),
                 m_edgeSpeedFilename(edgeSpeedFilename)
    {}

    void read( GraphType& G)
    {
        importNodes( G);

        importEdges( G);
    }


    void importNodes( GraphType& G)
    {
        std::ifstream in;
        SizeType numNodes = 0;
        unsigned int id;
        double x, y;

        NodeIterator u;
        NodeDescriptor uD;

        GraphReader<GraphType>::m_ids.clear();
        in.exceptions( std::ifstream::badbit);

        //import the nodes
        try
        {
            in.open( m_nodeFilename.c_str());

            std::string token;
            std::stringstream data;

            //ignore comment line
            in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');

            std::cout << "\nReading CSV data from " << m_nodeFilename << std::endl;

            //DIMACS{9|10}, CSV pact : id=[1, numNodes]
            //insert a null node
            GraphReader<GraphType>::m_ids.push_back( 0);

            while( getline( in, token))
            {
                //id;x;y

                data.str( token);
                //data >> id;
                //data.ignore( 1, ';');
                data >> x;
                //data.ignore( 1, ';');
                data >> y;
                data.clear();
                data.str( std::string());

                uD = G.insertNode();
                GraphReader<GraphType>::m_ids.push_back( uD);
                u = G.getNodeIterator( uD);

                //get lat and lon values (for the case of Berlin)
                u->x = x / 100000.0;
                u->y = y / 100000.0;

                numNodes++;
            }

            std::cout << "\tReading " << numNodes << " nodes...done!\n";

            in.close();
        }

        catch (std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_nodeFilename << "'\n";
            throw e;
        }
    }

    void importEdges( GraphType& G)
    {
        std::ifstream in;
        SizeType numEdges;

        NodeIterator u, v;
        EdgeIterator e;
        InEdgeIterator k;
        EdgeDescriptor eD;

        in.exceptions( std::ifstream::badbit);

        std::string token;
        std::stringstream data;

        //import the speed of the edges
        try
        {
            in.open( m_edgeSpeedFilename.c_str());

            //ignore comment line
            in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
            numEdges = 0;

            while( getline( in, token))
            {
                unsigned int from = 0, to = 0;
                double weekSpeed = 0, weekEndSpeed = 0, freeFlowSpeed = 0;

                data.str( token);
                data >> from;
                //data.ignore( 1, ';');
                data >> to;
                //data.ignore( 1, ';');
                data >> weekSpeed;
                //data.ignore( 1, ';');
                data >> weekEndSpeed;
                //data.ignore( 1, ';');
                data >> freeFlowSpeed;
                data.clear();
                data.str( std::string());

                if( from == to)
                    continue;

                //set 30 km/h speed for undefined week speed
                if( weekSpeed == 0)
                    freeFlowSpeed = weekSpeed = weekEndSpeed = 30;

                //set the weekend speed for undefined free flow speed
                else if( freeFlowSpeed == 0)
                    freeFlowSpeed = weekEndSpeed;

                from++; to++;
                eD = G.insertEdge( id2Desc( from), id2Desc( to));
                e = G.getEdgeIterator( eD);
                k = G.getInEdgeIterator( e);

                //compute the average speed
                e->weight = weekSpeed;
                k->weight = e->weight;

                numEdges++;
            }

            std::cout << "\tReading " << numEdges << " speeds ...done";
            in.close();
        }

        catch (std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_edgeSpeedFilename << "'\n";
            throw e;
        }

        //import the length of the edges
        try
        {
            in.open( m_edgeLengthFilename.c_str());

            //ignore comment line
            in.ignore( std::numeric_limits<std::streamsize>::max(), '\n');
            numEdges = 0;

            while( getline( in, token))
            {
                unsigned int from = 0, to = 0;
                double length = 0;

                data.str( token);
                data >> from;
                //data.ignore( 1, ';');
                data >> to;
                //data.ignore( 1, ';');
                data >> length;
                data.clear();
                data.str( std::string());

                from++; to++;
                u = G.getNodeIterator( id2Desc( from));
                v = G.getNodeIterator( id2Desc( to));

                if( G.hasEdge(u, v))
                {
                    e = G.getEdgeIterator(u, v);
                    k = G.getInEdgeIterator( e);

                    //compute average travel time (length/speed) in secs
                    e->weight = ceil( (length * 3.6) / e->weight);
                    k->weight = e->weight;

                    k->distance = e->distance = length;
                    numEdges++;
                }

                if( G.hasEdge(v, u))
                {
                    e = G.getEdgeIterator(v, u);
                    k = G.getInEdgeIterator( e);

                    //compute average travel time (length/speed) in secs
                    e->weight = ceil( (length * 3.6) / e->weight);
                    //e->weight = length;
                    k->weight = e->weight;
                    k->distance = e->distance = length;
                    numEdges++;
                }
            }

            std::cout << "\n\tReading " << numEdges << " lengths ...done";

            in.close();
        }

        catch (std::ifstream::failure e)
        {
            std::cerr << "Exception opening/reading file '" << m_edgeLengthFilename << "'\n";
            throw e;
        }
    }

    NodeDescriptor id2Desc( SizeType id)
    {
        return GraphReader<GraphType>::m_ids[id];
    }

 private:

    std::string m_nodeFilename;
    std::string m_edgeLengthFilename;
    std::string m_edgeSpeedFilename;
};


template<typename GraphType>
class GMLReader : public GraphReader<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    GMLReader( const std::string& filename):GraphReader<GraphType>(filename)
    {
    }

    void read( GraphType& G)
    {
        std::ifstream in;
        std::string token;
        std::cout << "Reading GML from " << GraphReader<GraphType>::m_filename << std::endl;
        GraphReader<GraphType>::m_ids.clear();
        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

        try {
            in.open( GraphReader<GraphType>::m_filename.c_str());

            if (in.is_open())
            {
                in.exceptions ( std::ios_base::goodbit);
                while ( in.good() && !in.eof() )
                {
                    in >> token;
                    //std::cout << token << '\n';
                    if( !token.compare("node"))
                    {
                        readNode( G, in);
                    }
                    if( !token.compare("edge"))
                    {
                        readEdge( G, in);
                    }
                }
                in.close();
            }
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << GraphReader<GraphType>::m_filename << "'\n";
            throw e;
        }
    }

    void readNode(GraphType& G, std::ifstream& in)
    {
        std::string token,value;
        NodeIterator u;
        NodeDescriptor uD;

        uD = G.insertNode();
        GraphReader<GraphType>::m_ids.push_back(uD);
        u = G.getNodeIterator(uD);

        in >> token;
        in >> token;

        while( token.compare("]"))
        {
            in >> value;
            u->setProperty( token, value);
            in >> token;
        }
    }

    void readEdge(GraphType& G, std::ifstream& in)
    {
        std::string token,value;
        unsigned int source,target;
        EdgeIterator e;
        EdgeDescriptor eD;

        in >> token;
        in >> token;
        if( !token.compare("source"))
        {
            in >> source;
        }

        in >> token;

        if( !token.compare("target"))
        {
            in >> target;
        }

        eD = G.insertEdge( GraphReader<GraphType>::m_ids[source], GraphReader<GraphType>::m_ids[target]);
        e = G.getEdgeIterator(eD);

        in >> token;

        while( token.compare("]"))
        {
            in >> value;
            e->setProperty( token, value);
            in >> token;
        }
    }

};


template<typename GraphType>
class DIMACS9Reader : public GraphReader<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::InEdgeIterator  InEdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    class EdgeList
    {
    public:
        EdgeList( unsigned int init = 0):source(0)
        {
        }

        EdgeList( const SizeType& s, const SizeType& t, const SizeType& w):source(s),target(t),weight(w)
        {
        }

        bool operator == (const EdgeList& other)
        {
            return (source == other.source) && (target == other.target);
        }

        bool operator != (const EdgeList& other)
        {
            return (source != other.source) || (target != other.target);
        }

        SizeType source;
        SizeType target;
        SizeType weight;
    };

    DIMACS9Reader( const std::string& filename, const std::string& coordinatesFilename = ""):GraphReader<GraphType>(filename),m_coordinatesFilename(coordinatesFilename)
    {
    }

    void read( GraphType& G)
    {
        GraphReader<GraphType>::m_ids.clear();
        std::string token, dummy;
        SizeType numNodes, numEdges;
        SizeType uID, vID, weight;
        NodeIterator u,end;
        EdgeIterator e;
        InEdgeIterator k;
        std::ifstream in;
        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
        std::cout << "Reading DIMACS9 from " << GraphReader<GraphType>::m_filename << std::endl;
        try {
            in.open( GraphReader<GraphType>::m_filename.c_str());
            assert( in.good());

            unsigned int readEdges = 0;

            ProgressStream edge_progress( numEdges);
            ProgressStream sort_progress( numEdges);

            PackedMemoryArray<EdgeList> edgeLists;

            while ( ((readEdges < numEdges) || numEdges == 0) && getline(in,token))
            {
                switch (token.c_str()[0])
                {
                    case 'p':
                        initGraph( G, token, numNodes, numEdges);
                        sort_progress.reset(numEdges);
                        sort_progress.label() << "\tSorting " << numEdges << " edges";
                        edge_progress.reset(numEdges);
                        edge_progress.label() << "\tReading " << numEdges << " edges";
                        edgeLists.reserve( numEdges);
                        break;
                    case 'a':
                        ++readEdges;
                        std::stringstream graphinfo;
                        graphinfo.str(token);
                        graphinfo >> dummy >> uID >> vID >> weight;

                        //assert( G.getRelativePosition( G.getNodeIterator( GraphReader<GraphType>::m_ids[uID])) == uID -1);
                        edgeLists.optimalInsert( EdgeList( uID, vID, weight));
                        ++sort_progress;
                        break;
                }
            }
            in.close();

            typename PackedMemoryArray<EdgeList>::Iterator it;
            while( !edgeLists.empty())
            {
                it = edgeLists.chooseCell();

                G.insertEdge( id2Desc(it->source), id2Desc(it->target));

                e = G.getEdgeIterator( id2Desc(it->source), id2Desc(it->target));
                k = G.getInEdgeIterator( e);
                e->weight = it->weight;
                k->weight = it->weight;

                ++edge_progress;
                edgeLists.erase(it);
            }

        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << GraphReader<GraphType>::m_filename << "'\n";
            throw e;
        }

        readCoordinates(G);
    }

    void readCoordinates( GraphType& G)
    {
        std::ifstream in;
        SizeType numNodes, source;
        int x,y;
        std::string token,dummy1;
        NodeIterator u;
        try {
            in.open( m_coordinatesFilename.c_str());
            assert( in.good());

            std::stringstream coordinatestream;
            numNodes = G.getNumNodes();
            coordinatestream << "\tReading " << numNodes << " coordinates";
            ProgressBar coordinate_progress(numNodes,coordinatestream.str());

            SizeType readCoords = 0;
            while ( (readCoords < numNodes) && getline(in,token))
            {
                switch (token.c_str()[0])
                {
                    case 'v':
                        std::stringstream coordinfo;
                        coordinfo.str(token);
                           coordinfo >> dummy1 >> source >> x >> y;
                        //std::cout << token << "\n";
                        //std::cout << "s: " << source << " x: " << x << " y: " << y << std::endl;

                        u = G.getNodeIterator( id2Desc(source));
                        x > 0? u->x = x: u->x = -x;
                        y > 0? u->y = y: u->y = -y;
                        ++coordinate_progress;
                        ++readCoords;
                        break;
                }
            }
            in.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << m_coordinatesFilename << "'\n";
            throw e;
        }
    }

    void initGraph( GraphType& G, const std::string& line, SizeType& numNodes, SizeType& numEdges)
    {
        std::string dummy1, dummy2;
        NodeIterator u, end;

        std::stringstream graphinfo;
        graphinfo.str(line);
        graphinfo >> dummy1 >> dummy2 >> numNodes >> numEdges;
        //std::cout << line << "\n";
        //std::cout << "n: " << numNodes << " m: " << numEdges << std::endl;

        G.reserve( numNodes, numEdges);

        ProgressStream node_progress( numNodes);
        node_progress.label() << "\tReading " << numNodes << " nodes";

        for( SizeType i = 0; i < numNodes; ++i)
        {
            G.insertNode();
            ++node_progress;
        }

        std::cout << "\tReserving " << G.getNumNodes() << " IDs\n";
        GraphReader<GraphType>::m_ids.reserve( G.getNumNodes() + 1);
        //GraphReader<GraphType>::m_ids.push_back(NodeDescriptor());

        GraphReader<GraphType>::m_ids.push_back( 0);

        ProgressStream id_progress( numNodes);
        id_progress.label() << "\tMapping IDs to " << numNodes << " nodes";
        for( u = G.beginNodes(), end = G.endNodes(); u != end; ++u)
        {
            GraphReader<GraphType>::m_ids.push_back( G.getNodeDescriptor(u));
            ++id_progress;
        }
    }

    NodeDescriptor id2Desc( SizeType id)
    {
        return GraphReader<GraphType>::m_ids[id];
    }

private:
    std::string m_coordinatesFilename;
};



template<typename GraphType>
class DIMACS9DoubleReader : public GraphReader<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::InEdgeIterator  InEdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    class EdgeList
    {
    public:
        EdgeList( unsigned int init = 0):source(0)
        {
        }

        EdgeList( const SizeType& s, const SizeType& t, const SizeType& w):source(s),target(t),weight(w)
        {
        }

        bool operator == (const EdgeList& other)
        {
            return (source == other.source) && (target == other.target);
        }

        bool operator != (const EdgeList& other)
        {
            return (source != other.source) || (target != other.target);
        }

        SizeType source;
        SizeType target;
        SizeType weight;
    };

    DIMACS9DoubleReader( const std::string& distanceFilename, const std::string& traveltimeFilename, const std::string& coordinatesFilename = ""):m_distanceFilename(distanceFilename),m_traveltimeFilename(traveltimeFilename),m_coordinatesFilename(coordinatesFilename)
    {
    }

    void read( GraphType& G)
    {
        GraphReader<GraphType>::m_ids.clear();
        std::string token, dummy;
        SizeType numNodes, numEdges;
        SizeType uID, vID, weight;
        NodeIterator u,end;
        EdgeIterator e;
        InEdgeIterator k;
        std::ifstream in;
        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
        std::cout << "Reading DIMACS9 distances from " << m_distanceFilename << std::endl;
        try {
            in.open( m_distanceFilename.c_str());
            assert( in.good());

            unsigned int readEdges = 0;

            ProgressStream edge_progress( numEdges);
            ProgressStream sort_progress( numEdges);

            PackedMemoryArray<EdgeList> edgeLists;

            while ( ((readEdges < numEdges) || numEdges == 0) && getline(in,token))
            {
                switch (token.c_str()[0])
                {
                    case 'p':
                        initGraph( G, token, numNodes, numEdges);
                        sort_progress.reset(numEdges);
                        sort_progress.label() << "\tSorting " << numEdges << " edges";
                        edge_progress.reset(numEdges);
                        edge_progress.label() << "\tReading " << numEdges << " edges";
                        edgeLists.reserve( numEdges);
                        break;
                    case 'a':
                        ++readEdges;
                        std::stringstream graphinfo;
                        graphinfo.str(token);
                        graphinfo >> dummy >> uID >> vID >> weight;
                        edgeLists.optimalInsert( EdgeList( uID, vID, weight));
                        ++sort_progress;
                        break;
                }
            }
            in.close();

            typename PackedMemoryArray<EdgeList>::Iterator it;
            while( !edgeLists.empty())
            {
                it = edgeLists.chooseCell();

                G.insertEdge( id2Desc(it->source), id2Desc(it->target));

                e = G.getEdgeIterator( id2Desc(it->source), id2Desc(it->target));
                k = G.getInEdgeIterator( e);
                e->criteriaList[0] = it->weight;
                k->criteriaList[0] = it->weight;

                ++edge_progress;
                edgeLists.erase(it);
            }

        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << m_distanceFilename << "'\n";
            throw e;
        }

        readTraveltimes(G);

        readCoordinates(G);
    }

    void readTraveltimes( GraphType& G)
    {
        std::string token, dummy, dummy1, dummy2;
        SizeType numNodes, numEdges;
        SizeType uID, vID, weight;
        NodeIterator u,end;
        EdgeIterator e;
        InEdgeIterator k;
        std::ifstream in;
        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
        std::cout << "Reading DIMACS9 travel times from " << m_traveltimeFilename << std::endl;
        try {
            in.open( m_traveltimeFilename.c_str());
            assert( in.good());

            unsigned int readEdges = 0;

            ProgressStream edge_progress( numEdges);


            while ( ((readEdges < numEdges) || numEdges == 0) && getline(in,token))
            {
                std::stringstream graphinfo;
                switch (token.c_str()[0])
                {

                    case 'p':
                        graphinfo.str(token);
                        graphinfo >> dummy1 >> dummy2 >> numNodes >> numEdges;

                        edge_progress.reset(numEdges);
                        edge_progress.label() << "\tReading " << numEdges << " edges";
                        break;
                    case 'a':
                        ++readEdges;
                        graphinfo.str(token);
                        graphinfo >> dummy >> uID >> vID >> weight;

                        e = G.getEdgeIterator( id2Desc(uID), id2Desc(vID));
                        k = G.getInEdgeIterator( e);
                        e->criteriaList[1] = weight;
                        k->criteriaList[1] = weight;

                        ++edge_progress;
                        break;
                }
            }
            in.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << m_distanceFilename << "'\n";
            throw e;
        }
    }

    void readCoordinates( GraphType& G)
    {
        std::ifstream in;
        SizeType numNodes, source;
        int x,y;
        std::string token,dummy1;
        NodeIterator u;
        try {
            in.open( m_coordinatesFilename.c_str());
            assert( in.good());

            std::stringstream coordinatestream;
            numNodes = G.getNumNodes();
            coordinatestream << "\tReading " << numNodes << " coordinates";
            ProgressBar coordinate_progress(numNodes,coordinatestream.str());

            SizeType readCoords = 0;
            while ( (readCoords < numNodes) && getline(in,token))
            {
                switch (token.c_str()[0])
                {
                    case 'v':
                        std::stringstream coordinfo;
                        coordinfo.str(token);
                           coordinfo >> dummy1 >> source >> x >> y;
                        //std::cout << token << "\n";
                        //std::cout << "s: " << source << " x: " << x << " y: " << y << std::endl;

                        u = G.getNodeIterator( id2Desc(source));
                        x > 0? u->x = x: u->x = -x;
                        y > 0? u->y = y: u->y = -y;
                        ++coordinate_progress;
                        ++readCoords;
                        break;
                }
            }
            in.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << m_coordinatesFilename << "'\n";
            throw e;
        }
    }

    void initGraph( GraphType& G, const std::string& line, SizeType& numNodes, SizeType& numEdges)
    {
        std::string dummy1,dummy2;
        NodeDescriptor uD;
        NodeIterator u,end;

        std::stringstream graphinfo;
        graphinfo.str(line);
        graphinfo >> dummy1 >> dummy2 >> numNodes >> numEdges;

        G.reserve( numNodes, numEdges);

        ProgressStream node_progress( numNodes);
        node_progress.label() << "\tReading " << numNodes << " nodes";

        for( SizeType i = 0; i < numNodes; ++i)
        {
            uD = G.insertNode();
            ++node_progress;
        }

        std::cout << "\tReserving " << G.getNumNodes() << " IDs\n";
        GraphReader<GraphType>::m_ids.reserve( G.getNumNodes() + 1);

        GraphReader<GraphType>::m_ids.push_back( 0);

        ProgressStream id_progress( numNodes);
        id_progress.label() << "\tMapping IDs to " << numNodes << " nodes";
        for( u = G.beginNodes(), end = G.endNodes(); u != end; ++u)
        {
            GraphReader<GraphType>::m_ids.push_back( G.getNodeDescriptor(u));
            ++id_progress;
        }
    }

    NodeDescriptor id2Desc( SizeType id)
    {
        return GraphReader<GraphType>::m_ids[id];
    }

private:
    std::string m_distanceFilename;
    std::string m_traveltimeFilename;
    std::string m_coordinatesFilename;
};



template<typename GraphType>
class DIMACS10Reader : public GraphReader<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    class NodeList
    {
    public:
        NodeList( unsigned int init = 0):source(0)
        {
        }

        NodeList( const NodeDescriptor& s):source(s)
        {
        }

        bool operator == (const NodeList& other)
        {
            return source == other.source;
        }

        bool operator != (const NodeList& other)
        {
            return source != other.source;
        }

        NodeDescriptor source;
        std::vector<NodeDescriptor> targets;
    };

    DIMACS10Reader( const std::string& filename, const std::string& coordinatesFilename):GraphReader<GraphType>(filename),m_coordinatesFilename(coordinatesFilename)
    {
    }

    void read( GraphType& G)
    {
        GraphReader<GraphType>::m_ids.clear();
        std::string token;
        SizeType numNodes, numEdges, source, target, x, y;
        NodeIterator u,end;
        EdgeIterator e;
        std::ifstream in;
        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
        std::cout << "Reading DIMACS10 from " << GraphReader<GraphType>::m_filename << std::endl;
        try {
            in.open( GraphReader<GraphType>::m_filename.c_str());
            assert( in.good());

            while (getline(in,token))
            {

                if( token.c_str()[0] == '%') continue;
                //std::cout << "Token: " << token << std::endl;
                std::stringstream graphinfo;
                graphinfo.str(token);
                graphinfo >> numNodes >> numEdges;
                break;
            }

            G.reserve( numNodes, numEdges<<1);

            ProgressStream node_progress( numNodes);
            node_progress.label() << "\tReading " << numNodes << " nodes";

            for( SizeType i = 0; i < numNodes; ++i)
            {
                G.insertNode();
                ++node_progress;
            }

            std::cout << "\tReserving " << G.getNumNodes() << " IDs\n";
            GraphReader<GraphType>::m_ids.reserve( G.getNumNodes() + 1);
            GraphReader<GraphType>::m_ids.push_back(NodeDescriptor());

            ProgressStream id_progress( numNodes);
            id_progress.label() << "\tMapping IDs to " << numNodes << " nodes";
            //GraphReader<GraphType>::m_ids.push_back( 0);
            for( u = G.beginNodes(), end = G.endNodes(); u != end; ++u)
            {
                GraphReader<GraphType>::m_ids.push_back( G.getNodeDescriptor(u));
                ++id_progress;
            }


            PackedMemoryArray<NodeList> nodeLists;
            std::cout << "\tReserving " << G.getNumNodes() << " node lists\t";
            nodeLists.reserve(numNodes);
            NodeList newList;

            ProgressStream sort_progress(numNodes);
            sort_progress.label() << "\tSorting " << numEdges << " edges";

            for( source = 1; source <= numNodes; ++source)
            {
                getline(in,token);
                while( token.c_str()[0] == '%') getline(in,token);
                std::stringstream edgeinfo;
                edgeinfo.str(token);

                newList.targets.clear();
                newList.source = GraphReader<GraphType>::m_ids[source];
                while( edgeinfo.good())
                {
                    edgeinfo >> target;
                    newList.targets.push_back(GraphReader<GraphType>::m_ids[target]);
                }
                nodeLists.optimalInsert( newList);
                ++sort_progress;
            }
            assert( nodeLists.size() == numNodes);


            ProgressStream edge_progress(numNodes);
            edge_progress.label() << "\tReading " << numEdges << " edges";

            typename PackedMemoryArray<NodeList>::Iterator it;
            while( !nodeLists.empty())
            {
                it = nodeLists.chooseCell();
                for( typename std::vector<NodeDescriptor>::iterator t = it->targets.begin(); t != it->targets.end(); ++t)
                {
                    G.insertEdge( it->source, *t);
                }
                ++edge_progress;
                nodeLists.erase(it);
            }


            in.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << GraphReader<GraphType>::m_filename << "'\n";
            throw e;
        }

        std::cout << "Reading coordinates from " << m_coordinatesFilename << std::endl;

        try {
            in.open( m_coordinatesFilename.c_str());
            assert( in.good());

            std::stringstream coordinatestream;
            numNodes = G.getNumNodes();
            coordinatestream << "\tReading " << numNodes << " coordinates";
            ProgressBar coordinate_progress(numNodes,coordinatestream.str());
            for( source = 1; source <= numNodes; ++source)
            {
                in >> x >> y >> token;
                u = G.getNodeIterator( GraphReader<GraphType>::m_ids[source]);
                u->x = x;
                u->y = y;
                ++coordinate_progress;
            }

            in.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << m_coordinatesFilename << "'\n";
            throw e;
        }

    }
private:
    std::string m_coordinatesFilename;
};


template<typename GraphType>
class GraphWinReader : public GraphReader<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    GraphWinReader( const std::string& filename):GraphReader<GraphType>(filename)
    {
    }

    void read( GraphType& G)
    {
        std::ifstream in;
        in.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
        try {
            in.open( GraphReader<GraphType>::m_filename.c_str());
            assert( in.good());
            SizeType numNodes, numEdges, source, target;
            std::vector<NodeDescriptor> V;

            for( unsigned int i = 0; i < 4; i++)
            {
                in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
            }

            in >> numNodes;
            std::stringstream nodestream;
            nodestream << "Reading " << numNodes << " nodes";
            ProgressBar node_progress( numNodes,nodestream.str());
            V.push_back(NodeDescriptor());

            for( SizeType i = 0; i < numNodes; ++i)
            {
                V.push_back(G.insertNode());
                in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
                ++node_progress;
            }
            in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');

            in >> numEdges;
            std::stringstream edgestream;
            edgestream << "Reading " << numEdges << " edges";
            ProgressBar edge_progress(numEdges,edgestream.str());
            for( SizeType i = 0; i < numEdges; ++i)
            {
                in >> source;
                in >> target;
                std::cout << source << " " << target << "\n";
                G.insertEdge( V[source], V[target]);
                in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
                ++edge_progress;
            }
            std::cout << "\rReading edges...done!\n";
            in.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << GraphReader<GraphType>::m_filename << "'\n";
            throw e;
        }
    }
};


//--------------------------------------- WRITERS --------------------------------------//

template<typename GraphType>
class GraphWriter
{
public:
    GraphWriter( const std::string& filename):m_filename(filename)
    {
    }

    virtual void write( GraphType& G)
    {
    }
protected:
    std::string m_filename;
};

template<typename GraphType>
class GraphVizWriter : public GraphWriter<GraphType>
{
public:

    typedef typename GraphType::NodeIterator NodeIterator;
    typedef typename GraphType::EdgeIterator EdgeIterator;
    typedef typename GraphType::SizeType SizeType;

    GraphVizWriter( const std::string& filename):GraphWriter<GraphType>(filename)
    {
    }

    virtual void write( GraphType& G)
    {
        std::ofstream out;
        out.exceptions ( std::ofstream::failbit | std::ofstream::badbit );
        try {
            out.open( GraphWriter<GraphType>::m_filename.c_str());
            out << "digraph BFS {\n\tedge [len=3]\n\tnode  [fontname=\"Arial\"]\n";

            NodeIterator u,v,lastnode;
            EdgeIterator e,lastedge;

            //G.addNodeProperty( "dotId", false);

            NodeArray<SizeType, GraphType> dotId( &G,std::numeric_limits<SizeType>::max());

            SizeType i = 0;

            std::stringstream nodestream;
            nodestream << "Writing out " << G.getNumNodes() << " nodes";
            ProgressBar node_progress( G.getNumNodes(),nodestream.str());

            for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u, ++i)
            {
                out << i;
                //G.setProperty( u, "dotId", i);
                dotId[u] = i;

                //assert(G.getProperty( u, "dotId") == dotId[u]);

                out << "[label=\"" << G.getRelativePosition(u) << " ";
                u->print(out);
                out << "\"]\n";
                ++node_progress;
            }

            std::stringstream edgestream;
            edgestream << "Writing out " << G.getNumEdges() << " edges";
            ProgressBar edge_progress( G.getNumEdges(),edgestream.str());

            for( u = G.beginNodes(), lastnode = G.endNodes(); u != lastnode; ++u)
            {
                for( e = G.beginEdges(u), lastedge = G.endEdges(u); e != lastedge; ++e)
                {
                    v = G.target(e);
                    //out << G.getProperty( u, "dotId") << "->" << G.getProperty( v, "dotId");
                    out << dotId[u] << "->" << dotId[v];
                    out << "[label=\"" ;
                    e->print(out);
                    out << "\"]\n";
                    ++edge_progress;
                }
            }

            //G.removeNodeProperty( "dotId");

            out << "}";
            out.close();
        }
        catch (std::ofstream::failure e) {
            std::cout << "Exception opening/writing file '" << GraphWriter<GraphType>::m_filename << "'\n";
            throw e;
        }
    }
};

template<typename GraphType>
class GMLWriter : public GraphWriter<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    GMLWriter( const std::string& filename):GraphWriter<GraphType>(filename)
    {
    }

    void write( GraphType& G)
    {
        std::ofstream out;
        std::cout << "Writing GML to " << GraphWriter<GraphType>::m_filename << std::endl;
        V.clear();
        out.exceptions ( std::ofstream::failbit | std::ofstream::badbit );

        try {
            out.open( GraphWriter<GraphType>::m_filename.c_str());

            if (out.is_open())
            {
                out.exceptions ( std::ios_base::goodbit);

                std::stringstream node_stream, edge_stream;
                node_stream << "Writing " << G.getNumNodes() << " nodes";
                edge_stream << "Writing " << G.getNumEdges() << " edges";
                ProgressBar node_progress( G.getNumNodes(),node_stream.str());
                ProgressBar edge_progress( G.getNumEdges(),edge_stream.str());

                out << "graph [\n";

                NodeIterator u, lastNode;
                EdgeIterator e, lastEdge;

                nodeId = 0;
                for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
                {
                    writeNode( G, u, out);
                    ++node_progress;
                }

                for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
                {
                    for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
                    {
                        writeEdge( G, u, e, out);
                        ++edge_progress;
                    }
                }

                out << "]\n";

                out.close();
            }
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/writing file '" << GraphWriter<GraphType>::m_filename << "'\n";
            throw e;
        }
    }

    void writeNode(GraphType& G, NodeIterator& u, std::ofstream& out)
    {
        out << "node [" << std::endl;
        out << "id " << nodeId++ << "\n";
        u->writeProperties(out);
        out << "]" << std::endl;
    }

    void writeEdge(GraphType& G, NodeIterator& u, EdgeIterator& e, std::ofstream& out)
    {
        out << "edge [" << std::endl;
        out << "source " << G.getRelativePosition(u) << std::endl;
        NodeIterator v = G.target(e);
        out << "target " << G.getRelativePosition(v) << std::endl;
        e->writeProperties(out);
        out << "]" << std::endl;
    }

private:
    std::vector<NodeDescriptor> V;
    unsigned int nodeId;
};


template<typename GraphType>
class JSONWriter : public GraphWriter<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    JSONWriter( const std::string& filename):GraphWriter<GraphType>(filename)
    {
    }

    void write( GraphType& G)
    {
        std::ofstream out;
        std::cout << "Writing JSON to " << GraphWriter<GraphType>::m_filename << std::endl;
        V.clear();
        out.exceptions ( std::ofstream::failbit | std::ofstream::badbit );

        try {
            out.open( GraphWriter<GraphType>::m_filename.c_str());

            if (out.is_open())
            {
                out.exceptions ( std::ios_base::goodbit);
                std::stringstream node_stream, edge_stream;
                node_stream << "Writing " << G.getNumNodes() << " nodes";
                edge_stream << "Writing " << G.getNumEdges() << " edges";
                ProgressBar node_progress( G.getNumNodes(),node_stream.str());
                ProgressBar edge_progress( G.getNumEdges(),edge_stream.str());


                out << "{\n\"graph\": [\n";

                NodeIterator u, w, lastNode;
                EdgeIterator e, k, lastEdge;

                out << "\"nodes\": [\n";

                nodeId = 0;
                w = G.beginNodes();
                ++w;
                for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u, ++w)
                {
                    writeNode( G, u, out);
                    ++node_progress;
                    if( w != lastNode) out << ",";
                    out << "\n";
                }

                out << "],\n\"edges\": [\n";

                w = G.beginNodes();
                ++w;
                for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
                {
                    k = G.beginEdges(u);
                    ++k;
                    for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
                    {
                        writeEdge( G, u, e, out);
                        ++edge_progress;
                        if( w != lastNode) out << ",";
                        else
                        {
                            if( k != lastEdge) out << ",";
                        }
                        out << "\n";
                    }
                }

                out << "}\n";

                out.close();
            }
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/writing file '" << GraphWriter<GraphType>::m_filename << "'\n";
            throw e;
        }
    }

    void writeNode(GraphType& G, NodeIterator& u, std::ofstream& out)
    {
        out << "{" ;
        out << "\"id\":" << nodeId++ << "";
        u->writeJSON(out);
        out << "}";
    }

    void writeEdge(GraphType& G, NodeIterator& u, EdgeIterator& e, std::ofstream& out)
    {
        out << "{" ;
        out << "\"s\":" << G.getRelativePosition(u) << ",";
        NodeIterator v = G.target(e);
        out << "\"t\":" << G.getRelativePosition(v) << ",";
        e->writeJSON(out);
        out << "}";
    }

private:
    std::vector<NodeDescriptor> V;
    unsigned int nodeId;
};


template<typename GraphType>
class CSVWriter : public GraphWriter<GraphType>
{

public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::InEdgeIterator  InEdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;

    CSVWriter( const std::string& nodeFilename, const std::string& edgeFilename, std::vector<NodeDescriptor>& ids, bool MAXSCC = false):
    GraphWriter<GraphType>(std::string()),
    m_nodeFilename(nodeFilename), m_edgeFilename(edgeFilename), m_ids(ids), m_MAXSCC(MAXSCC)
    {}


    /**
     * @brief Defines the strongly connected components of the graph.
     * @param G The graph object.
     * @return The component id that corresponds to the largest strongly connected component.
     **/
    unsigned int markStronglyConnectedComponents( GraphType& G)
    {
        NodeIterator u, v, s, lastNode;
        EdgeIterator e;
        InEdgeIterator k, lastInEdge;
        std::stack<NodeIterator> S;
        std::stack<NodeIterator> orderedNodes;
        unsigned int component = 0, maxSize = 0;
        unsigned int componentSize = 0, maxSCC = 0;

        //initialize DFS
        for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
        {
            u->component = 0;
            u->isMarked = false;
        }

        //run Forward DFS
        for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
        {
            if( u->isMarked == false)
                runIterativeDFS( u, orderedNodes, G);
        }

        //initialize DFS
        for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
        {
            u->component = 0;
            u->isMarked = false;
        }

        //run Backward DFS
        while( !orderedNodes.empty())
        {
            u = orderedNodes.top();
            orderedNodes.pop();

            if( u->isMarked == true)
                continue;

            component++;
            S.push( u);
            u->isMarked = true;
            u->component = component;
            componentSize = 1;

            while( !S.empty())
            {
                s = S.top();
                S.pop();

                for( k = G.beginInEdges( s), lastInEdge = G.endInEdges( s); k != lastInEdge; ++k)
                {
                    v = G.source( k);

                    if( v->isMarked == false)
                    {
                        S.push( v);
                        v->isMarked = true;
                        v->component = component;
                        componentSize++;
                    }
                }
            }

            if( maxSize < componentSize)
            {
                maxSize = componentSize;
                maxSCC = component;
            }
        }

        std::cout << "\n\tStrongly connected components: " << component
                  << "\n\tLargest strongly connected component size: " << maxSize << "\n";

        return maxSCC;
    }


    /**
     * @brief Performs a DFS ordering of the nodes by decreasing finishing time
     * @param root The root node.
     * @param orderedNodes A stack containing the DFS-ordered nodes.
     **/
    void runIterativeDFS( const NodeIterator& root, std::stack<NodeIterator> &orderedNodes, GraphType& G)
    {
        std::stack<NodeIterator> stack;
        stack.push( root);
        root->isMarked = true;

        while( stack.empty() == false)
        {
            NodeIterator u = stack.top();

            unsigned int ssize = stack.size();

            for( EdgeIterator e = G.beginEdges( u), lastedge = G.endEdges( u); e != lastedge; ++e)
            {
                NodeIterator v = G.target( e);

                if( v->isMarked == false)
                {
                    v->isMarked = true;
                    stack.push( v);
                }
            }

            if( ssize == stack.size())
            {
                stack.pop();
                orderedNodes.push( u);
            }
        }
    }


    /**
     * @brief Extracts the largest strongly connected component of the graph.
     * @param G The graph object.
     **/
    void extractMaxStronglyConnectedComponent( GraphType& G)
    {
        NodeIterator u, lastNode;
        EdgeIterator e, lastEdge;
        InEdgeIterator k, lastInEdge;

        std::vector<NodeDescriptor> erasedNodes;
        std::vector<EdgeDescriptor> erasedEdges;

        std::cout << "\tExtract the largest strongly connected component";

        const unsigned int maxSCC = markStronglyConnectedComponents( G);
        std::vector<NodeDescriptor> newIds;
        newIds.push_back(0);

        //erase the nodes that are not in the largest strongly connected component
        for( unsigned int i=1, size=m_ids.size(); i<size; i++)
        {
            NodeIterator u = G.getNodeIterator( m_ids[i]);

            if( u->component != maxSCC)
            {
                //get the node
                erasedNodes.push_back( m_ids[i]);

                //its outgoing edges
                for( e = G.beginEdges(u), lastEdge = G.endEdges(u); e != lastEdge; ++e)
                    erasedEdges.push_back( G.getEdgeDescriptor( e));

                //its incoming edges
                for( k = G.beginInEdges(u), lastInEdge = G.endInEdges(u); k != lastInEdge; ++k)
                    erasedEdges.push_back( G.getEdgeDescriptor( k));
            }
            else
                newIds.push_back( m_ids[i]);
        }

        m_ids.swap( newIds);

        //erase duplicates
        sort( erasedEdges.begin(), erasedEdges.end() );
        erasedEdges.erase( unique( erasedEdges.begin(), erasedEdges.end() ), erasedEdges.end() );

        ProgressStream del_progress( erasedEdges.size() + erasedNodes.size());
        del_progress.label() << "\tRemoving " << erasedEdges.size() << " edges and " << erasedNodes.size() << " nodes";

        //erase the selected edges
        while( !erasedEdges.empty())
        {
            G.eraseEdge( erasedEdges.back());
            erasedEdges.pop_back();
            ++del_progress;
        }

        //erase the selected nodes
        while( !erasedNodes.empty())
        {
            G.eraseNode( erasedNodes.back());
            erasedNodes.pop_back();
            ++del_progress;
        }
    }


    void write( GraphType& G)
    {
        std::ofstream out;
        unsigned int startNodeID, endNodeID, nodeID;
        double x, y, weight;

        NodeIterator u, v;
        EdgeIterator e, endEdges;

        if( m_MAXSCC)
            extractMaxStronglyConnectedComponent( G);

        // set the id index of nodes
        for(unsigned int i=1; i < m_ids.size(); i++)
            G.getNodeIterator( m_ids[i])->id = i;

        std::cout << "Writing nodes to : " << m_nodeFilename << std::endl
                  << "    and edges to : " << m_edgeFilename << std::endl;

        out.exceptions ( std::ofstream::failbit | std::ofstream::badbit );

        //export nodes
        try
        {
            out.open( m_nodeFilename.c_str());

            //write the number of nodes in G
            out << G.getNumNodes() << "\n";

            //write the info of the nodes
            std::stringstream nodestream;
            nodestream << "Writing " << G.getNumNodes() << " nodes";
            ProgressBar node_progress( G.getNumNodes(), nodestream.str());

            //DIMACS{9|10},CSV pact : id=[1, numNodes] => m_ids[0] = nullNode
            for( nodeID = 1; nodeID < m_ids.size(); nodeID++)
            {
                u = G.getNodeIterator( m_ids[nodeID]);
                out << u->x << " " << u->y << "\n";
                ++node_progress;
            }

            out.close();
        }

        catch (std::ofstream::failure e)
        {
            std::cerr << "Exception writing file '" << m_nodeFilename << "'\n";
            throw e;
        }

        //export edges
        try
        {
            out.open( m_edgeFilename.c_str());

            //write the number of edges in G
            out << G.getNumEdges() << "\n";

            //write the info of the nodes
            std::stringstream edgestream;
            edgestream << "Writing " << G.getNumEdges() << " edges";
            ProgressBar edge_progress( G.getNumEdges(), edgestream.str());

            //DIMACS{9|10}, CSV pact : id=[1, numNodes] => m_ids[0] = nullNode
            for( startNodeID=1; startNodeID < m_ids.size(); startNodeID++)
            {
                u = G.getNodeIterator( m_ids[startNodeID]);

                for( e = G.beginEdges(u), endEdges = G.endEdges(u); e != endEdges; ++e)
                {
                    v = G.target(e);

                    out << u->id  << " " << v->id << " " << e->distance << " " << e->travelTime << "\n";
                    ++edge_progress;
                }
            }

            out.close();
        }

        catch (std::ofstream::failure e)
        {
            std::cerr << "Exception writing file '" << m_edgeFilename << "'\n";
            throw e;
        }

    }

 private:

    std::string m_edgeFilename;
    std::string m_nodeFilename;
    std::vector<NodeDescriptor>& m_ids;
    bool m_MAXSCC;
};


template<typename GraphType>
class DIMACS10Writer : public GraphWriter<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;
    typedef typename std::vector<NodeDescriptor>::iterator iterator;

    DIMACS10Writer( const std::string& filename, const std::string& coordinatesFilename):GraphWriter<GraphType>(filename),m_coordinatesFilename(coordinatesFilename)
    {
    }

    void write( GraphType& G)
    {
        std::ofstream out;
        std::cout << "Writing shuffled DIMACS10 to " << GraphWriter<GraphType>::m_filename << std::endl;
        NodeIterator u,v;
        EdgeIterator e,beginEdges,endEdges;
        std::ifstream in;
        out.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

        try {
            out.open( GraphWriter<GraphType>::m_filename.c_str());
            out << G.getNumNodes() << " " << G.getNumEdges()/2 << std::endl;

            std::stringstream nodestream;
            nodestream << "Writing " << G.getNumNodes() << " nodes";
            ProgressBar node_progress( G.getNumNodes(),nodestream.str());
            NodeIterator u, lastNode;
            for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
            {
                beginEdges = G.beginEdges(u);
                endEdges = G.endEdges(u);
                for( e = beginEdges ; e != endEdges; ++e)
                {
                    v = G.target(e);
                    if( e != beginEdges) out << " ";
                    out << G.getRelativePosition(v) + 1;
                }
                out << "\n";
                ++node_progress;
            }

            out.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << GraphWriter<GraphType>::m_filename  << "'\n";
            throw e;
        }

        try {
            std::cout << "Writing shuffled coordinates to " << m_coordinatesFilename << std::endl;
            out.open( m_coordinatesFilename.c_str());
            std::stringstream nodestream;
            nodestream << "\tWriting " << G.getNumNodes() << " coordinates";
            ProgressBar node_progress( G.getNumNodes(),nodestream.str());
            NodeIterator u, lastNode;
            for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
            {
                out << u->x << " " << u->y << " 0\n";
                ++node_progress;
            }

            out.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << m_coordinatesFilename << "'\n";
            throw e;
        }

    }
private:
    std::string m_coordinatesFilename;
};


template<typename GraphType>
class DIMACS10Shuffler : public GraphWriter<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;
    typedef typename std::vector<NodeDescriptor>::iterator iterator;

    DIMACS10Shuffler( const std::string& filename, const std::string& coordinatesFilename, const std::vector<NodeDescriptor>& ids):GraphWriter<GraphType>(filename),m_coordinatesFilename(coordinatesFilename),m_ids(ids)
    {
    }

    void write( GraphType& G)
    {
        std::ofstream out;
        std::cout << "Writing shuffled DIMACS10 to " << GraphWriter<GraphType>::m_filename << std::endl;
        NodeIterator u,v;
        EdgeIterator e,beginEdges,endEdges;
        iterator it ,end;
        std::ifstream in;
        out.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

        try {
            out.open( GraphWriter<GraphType>::m_filename.c_str());
            out << G.getNumNodes() << " " << G.getNumEdges()/2 << std::endl;

            std::stringstream nodestream;
            nodestream << "Writing " << G.getNumNodes() << " nodes";
            ProgressBar node_progress( G.getNumNodes(),nodestream.str());
            for( it = m_ids.begin(), end = m_ids.end(); it != end; ++it)
            {
                u = G.getNodeIterator(*it);
                beginEdges = G.beginEdges(u);
                endEdges = G.endEdges(u);
                for( e = beginEdges ; e != endEdges; ++e)
                {
                    v = G.target(e);
                    if( e != beginEdges) out << " ";
                    out << v->rank + 1;
                }
                out << "\n";
                ++node_progress;
            }

            out.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << GraphWriter<GraphType>::m_filename  << "'\n";
            throw e;
        }

        try {
            std::cout << "Writing shuffled coordinates to " << m_coordinatesFilename << std::endl;
            out.open( m_coordinatesFilename.c_str());
            std::stringstream nodestream;
            nodestream << "\tWriting " << G.getNumNodes() << " coordinates";
            ProgressBar node_progress( G.getNumNodes(),nodestream.str());
            for( it = m_ids.begin(), end = m_ids.end(); it != end; ++it)
            {
                u = G.getNodeIterator(*it);
                out << u->x << " " << u->y << " 0\n";
                ++node_progress;
            }

            out.close();
        }
        catch (std::ifstream::failure e) {
            std::cerr << "Exception opening/reading file '" << m_coordinatesFilename << "'\n";
            throw e;
        }

    }
private:
    std::string m_coordinatesFilename;
    std::vector<NodeDescriptor> m_ids;
};


template<typename GraphType>
class DDSGWriter : public GraphWriter<GraphType>
{
public:
    typedef typename GraphType::NodeIterator    NodeIterator;
    typedef typename GraphType::EdgeIterator    EdgeIterator;
    typedef typename GraphType::NodeDescriptor  NodeDescriptor;
    typedef typename GraphType::EdgeDescriptor  EdgeDescriptor;
    typedef typename GraphType::SizeType        SizeType;
    typedef typename std::vector<NodeDescriptor>::iterator iterator;

    DDSGWriter( const std::string& filename):GraphWriter<GraphType>(filename)
    {
    }

    void write( GraphType& G)
    {
        std::ofstream out;
        std::cout << "Writing DDSG to " << GraphWriter<GraphType>::m_filename << std::endl;
        NodeIterator u,v,lastNode;
        EdgeIterator e,beginEdges,endEdges;
        out.exceptions ( std::ofstream::failbit | std::ofstream::badbit );


        //restrictions : 1) the loaded graph must be undirected
        //               2) max node Id != max(unisgned int)

        try {
            out.open( GraphWriter<GraphType>::m_filename.c_str());
            out << "d" << std::endl
                << G.getNumNodes() << " " << G.getNumEdges() << std::endl;

            std::stringstream edgestream;
            edgestream << "Writing " << G.getNumEdges() << " edges";
            ProgressBar edge_progress( G.getNumEdges(),edgestream.str());

            for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
            {
                beginEdges = G.beginEdges(u);
                endEdges = G.endEdges(u);
                for( e = beginEdges ; e != endEdges; ++e)
                {
                    v = G.target(e);
                    out << G.getRelativePosition(u) << " " << G.getRelativePosition(v) << " " << e->weight << std::endl;
                    ++edge_progress;
                }
            }

            out.close();
        }
        catch (std::ofstream::failure e) {
            std::cerr << "Exception opening/reading file '" << GraphWriter<GraphType>::m_filename  << "'\n";
            throw e;
        }
    }
};

template<typename GraphType>
class MetisGraphWriter
{

 public:

    typedef typename GraphType::NodeIterator         NodeIterator;
    typedef typename GraphType::EdgeIterator         EdgeIterator;
    typedef typename GraphType::InEdgeIterator       InEdgeIterator;
    typedef typename std::vector<NodeIterator>       NodeContainer;

    struct FreeNodeSelectionOption
    {
        bool isEnabled( NodeIterator& v)
        {
            return true;
        }
    };

    MetisGraphWriter( const GraphType& graph, const std::string& graphFileName):
    G(graph), outputFileName(graphFileName)
    {}

    NodeContainer write()
    {
        return write<FreeNodeSelectionOption>();
    }

    template <typename NodeSelectionOption>
    NodeContainer write()
    {
        std::ofstream out;
        out.open( outputFileName.c_str());
        std::cout << "[+] Writing metis graph data. (output:" << outputFileName.c_str() <<")\n";
        if( out.is_open() == false)
        {
            std::cout << "[-] Error opening the file. Exiting...\n";
            exit(-1);
        }

        NodeIterator u, v, endNode;
        EdgeIterator e, endEdge;
        InEdgeIterator k, endInEdge;
        std::size_t numNodes = 0, numEdges = 0;

        NodeContainer chosenNodes( G.getNumNodes());
        std::unordered_map<std::size_t, std::size_t> mapIds( G.getNumNodes());
        NodeSelectionOption option;
        std::size_t i=0;
        for( v = G.beginNodes(), endNode = G.endNodes(); v != endNode; ++v)
        {
            if( option.isEnabled(v))
            {
                chosenNodes[i] = v;
                ++i;
                mapIds[v->getNodeId()] = i;
            }
        }

        chosenNodes.resize( i);
        numNodes = chosenNodes.size();

        ProgressStream undirected_progress( numNodes);
        undirected_progress.label() << "Counting the undirected edges for " << numNodes << " nodes";
        for( i=0; i<chosenNodes.size(); i++)
        {
            u = chosenNodes[i];

            for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                v = G.target( e);
                if( option.isEnabled(v))
                    numEdges++;
            }

            for( k = G.beginInEdges( u), endInEdge = G.endInEdges( u); k != endInEdge; ++k)
            {
                v = G.source( k);
                if( option.isEnabled(v))
                {
                    if( G.hasEdge(u,v) == true)
                        continue;
                    numEdges++;
                }
            }

            ++undirected_progress;
        }

        numEdges /= 2;

        ProgressStream write_progress( chosenNodes.size());
        write_progress.label() << "Writing metis graph ("  << numNodes << " nodes, " << numEdges << " undirected edges)";
        out << numNodes << " " << numEdges << "\n";

        std::vector<std::size_t> adjIds;
        for( i=0; i<chosenNodes.size(); i++)
        {
            u = chosenNodes[i];
            adjIds.clear();

            for( e = G.beginEdges( u), endEdge = G.endEdges( u); e != endEdge; ++e)
            {
                v = G.target( e);
                if( option.isEnabled(v))
                    adjIds.push_back( mapIds[v->id]);
            }

            for( k = G.beginInEdges( u), endInEdge = G.endInEdges( u); k != endInEdge; ++k)
            {
                v = G.source( k);
                if( option.isEnabled(v))
                {
                    if( G.hasEdge(u,v) == true)
                        continue;
                    adjIds.push_back( mapIds[v->id]);
                }
            }

            std::sort( adjIds.begin(), adjIds.end());
            adjIds.erase( std::unique( adjIds.begin(), adjIds.end()), adjIds.end());

            if( adjIds.size() > 0)
            {
                const std::size_t numIds = adjIds.size()-1;
                for( std::size_t i=0; i<numIds; i++)
                    out << adjIds[i] << " ";
                out << adjIds.back();
            }

            out << "\n";
            ++write_progress;
        }

        return chosenNodes;
    }

    //not tested
    void write( std::vector<NodeIterator>& nodes, std::vector<std::vector<NodeIterator>>& edges, std::unordered_map<std::size_t, NodeIterator>& id2node)
    {
        std::ofstream out;
        out.open( outputFileName.c_str());
        std::cout << "[+] Writing metis graph data. (output:" << outputFileName.c_str() <<")\n";
        if( out.is_open() == false)
        {
            std::cout << "[-] Error opening the file. Exiting...\n";
            exit(-1);
        }

        std::size_t numEdges = 0;
        std::size_t numNodes = nodes.size();
        std::map<NodeIterator, std::size_t> node2id;
        id2node.reserve( nodes.size());
        for( std::size_t i=0; i<nodes.size(); i++)
        {
            node2id[nodes[i]] = i;
            id2node[i] = nodes[i];
            numEdges += edges[i].size();
        }

        numEdges /= 2;
        ProgressStream write_progress( numNodes);
        write_progress.label() << "Writing metis graph ("  << numNodes << " nodes, " << numEdges << " undirected edges)";
        out << numNodes << " " << numEdges << "\n";

        for( std::size_t i=0; i<edges.size(); i++)
        {
            if( edges[i].size() > 0)
            {
                const std::size_t numIds = edges[i].size()-1;
                for( std::size_t j=0; j<numIds; j++)
                    out << node2id[edges[i][j]] << " ";
                out << node2id[edges[i].back()];
            }

            out << "\n";
            ++write_progress;
        }
    }

 private:

    const GraphType& G;
    const std::string& outputFileName;
};

#endif //GRAPHIO_H
