#ifndef IMPLICIT_HEAP
#define IMPLICIT_HEAP

#include <cstdint>
#include <cstdlib>

template <typename KeyType, typename DataType>
class Implicit_Heap
{
 public:

    #define BRANCHING_FACTOR 4

    #define PQ_MEM_WIDTH 32



    /**
     * Holds an inserted element, as well as the current index in the node array.
     * Acts as a handle to clients for the purpose of mutability.
     */
    struct implicit_node_t
    {
        //! Index for the item in the "tree" array
        uint32_t index;

        //! Pointer to a piece of client data
        DataType item;

        //! Key for the item
        KeyType key;
    } __attribute__ ((aligned(4)));

    typedef struct implicit_node_t implicit_node;
    typedef implicit_node pq_node_type;
    typedef implicit_node PQNodeType;

    /**
     * A mutable, meldable, array-based d-ary heap.  Maintains a single, complete
     * d-ary tree.  Imposes the standard heap invariant.
     */
    struct implicit_heap_t
    {
        //! The array of node pointers encoding the tree structure
        uint32_t index_data;

        implicit_node **nodes;
        implicit_node *data;
        //! The number of items held in the queue
        uint32_t size;
        //! Current capacity of the heap
        uint32_t capacity;

        void mm_create( uint32_t maxCapacity = 473254)
        {
            capacity = maxCapacity;
            data = (pq_node_type *) malloc( sizeof(implicit_node) * capacity );
            nodes = (implicit_node**) calloc( maxCapacity, sizeof( implicit_node* ) );
        }

        void mm_destroy()
        {
            free( data );
            free( nodes );
        }

        void mm_clear()
        {
            index_data = 0;
            size = 0;
        }

    } __attribute__ ((aligned(4)));

    typedef struct implicit_heap_t implicit_heap;
    typedef implicit_heap pq_type;

    Implicit_Heap()
    {
        create();
    }

    ~Implicit_Heap()
    {
        destroy();
    }

    void reserve( uint32_t maxCapacity)
    {
        clear();
        queue.mm_destroy();
        queue.mm_create( maxCapacity);
    }

    /**
     * Deletes all nodes, leaving the queue empty.
     */
    void clear()
    {
        queue.mm_clear();
    }

    /**
     * Returns the key associated with the queried node.
     *
     * @param node  Node to query
     * @return      Node's key
     */
    const KeyType& getKey( const PQNodeType* node)
    {
        return node->key;
    }

    /**
     * Returns the item associated with the queried node.
     *
     * @param node  Node to query
     * @return      Node's item
     */
    DataType& getData( const PQNodeType* node)
    {
        return node->item;
    }

    /**
     * Returns the current size of the queue.
     *
     * @return      Size of queue
     */
    uint32_t size()
    {
        return queue.size;
    }

    /**
     * Takes an item-key pair to insert it into the queue and creates a new
     * corresponding node.  Merges the new node with the root of the queue.
     *
     * @param key   Key to use for node priority
     * @param item  Item to insert
     * @return      Pointer to corresponding node
     */
    PQNodeType* insert( KeyType key, DataType item)
    {
        implicit_node *node = (implicit_node *) &(queue.data[queue.index_data]);
        queue.index_data++;

        //pq_alloc_node( queue.map);
        node->item = item;
        node->key = key;
        node->index = queue.size++;


        queue.nodes[node->index] = node;
        heapify_up( node );

        return node;
    }

    /**
     * Returns the minimum item from the queue without modifying any data.
     *
     * @return      Node with minimum key
     */
    PQNodeType* findMin()
    {
        if( empty( queue ) )
            return NULL;
        return queue.nodes[0];
    }

    const KeyType& minKey() const
    {
        return queue.nodes[0]->key;
    }

    const DataType& minItem() const
    {
        return queue.nodes[0]->item;
    }

    /**
     * Deletes the minimum item from the queue and returns it, restructuring
     * the queue along the way to maintain the heap property.  Relies on the
     * @ref <pq_delete> method to delete the root of the tree.
     *
     * @return      Minimum key, corresponding to item deleted
     */
    KeyType popMin()
    {
        /*std::cout << "\nkey:" << queue.root->key << " ";
        std::cout << queue.size;
        {int a; std::cin>>a;}*/
        return remove( queue.nodes[0] );
    }

    /**
     * Deletes an arbitrary item from the queue and modifies queue structure
     * to preserve the heap invariant.  Requires that the location of the
     * item's corresponding node is known.  Removes the node from its list
     * of siblings, then merges all its children into a new tree and
     * subsequently merges that tree with the root.
     *
     * @param node  Pointer to node corresponding to the item to delete
     * @return      Key of item deleted
     */
    KeyType remove( PQNodeType* node )
    {
        KeyType key = node->key;
        implicit_node *last_node = queue.nodes[queue.size - 1];
        push( last_node->index, node->index );

        queue.size--;

        if ( node != last_node )
            heapify_down( last_node );

        return key;
    }


    /**
     * If the item in the queue is modified in such a way to decrease the
     * key, then this function will update the queue to preserve queue
     * properties given a pointer to the corresponding node.  Cuts the node
     * from its list of siblings and merges it with the root.
     *
     * @param node      Node to change
     * @param new_key   New key to use for the given node
     */
    void decrease( PQNodeType* node, KeyType new_key )
    {
        node->key = new_key;
        heapify_up( node );
    }

    /**
     * Determines whether the queue is empty, or if it holds some items.
     *
     * @return      True if queue holds nothing, false otherwise
     */
    bool empty() const
    {
        return ( queue.size == 0 );
    }

 private:

     void push( uint32_t src, uint32_t dst )
    {
        queue.nodes[dst] = queue.nodes[src];
        queue.nodes[dst]->index = dst;
    }

    /**
     * Places a node in a certain location in the tree, updating both the
     * queue structure and the node record.
     *
     * @param queue Queue to which the node belongs
     * @param node  Pointer to node to be dumped
     * @param dst   Index of location to dump node
     */
     void dump( implicit_node *node, uint32_t dst )
    {
        queue.nodes[dst] = node;
        node->index = dst;
    }

    /**
     * Takes a node that is potentially at a higher position in the tree
     * than it should be, and pulls it up to the correct location.
     *
     * @param queue Queue to which node belongs
     * @param node  Potentially violating node
     */
     uint32_t heapify_down( implicit_node *node )
    {
        if ( node == NULL )
            return -1;

        uint32_t sentinel, i, min;
        uint32_t base = node->index;
        while( base * BRANCHING_FACTOR + 1 < queue.size )
        {
            i = base * BRANCHING_FACTOR + 1;
            sentinel = i + BRANCHING_FACTOR;
            if( sentinel > queue.size )
                sentinel = queue.size;

            min = i++;
            for( i = i; i < sentinel; i++ )
            {
                if( queue.nodes[i]->key < queue.nodes[min]->key )
                    min = i;
            }

            if ( queue.nodes[min]->key < node->key )
                push( min, base );
            else
                break;

            base = min;
        }

        dump( node, base );

        return node->index;
    }

    /**
     * Takes a node that is potentially at a lower position in the tree
     * than it should be, and pulls it up to the correct location.
     *
     * @param queue Queue to which node belongs
     * @param node  Potentially violating node
     */
     uint32_t heapify_up( implicit_node *node )
    {
        if ( node == NULL )
            return -1;

        uint32_t i;
        for( i = node->index; i > 0; i = (i-1)/BRANCHING_FACTOR )
        {
            if ( node->key < queue.nodes[(i-1)/BRANCHING_FACTOR]->key )
                push( (i-1)/BRANCHING_FACTOR, i );
            else
                break;
        }
        dump( node, i );

        return node->index;
    }

    implicit_heap queue;

    /**
     * Creates a new, empty queue.
     *
     * @param map   Memory map to use for node allocation
     * @return      Pointer to the new queue
     */
    void create()
    {
        queue.mm_create();
    }

    /**
     * Frees all the memory used by the queue.
     */
    void destroy()
    {
        queue.mm_destroy();
    }
};

#endif