template <typename KeyType, typename DataType>
class Pairing_Heap
{
 public:

/*
    #define PQ_MEM_WIDTH 32

typedef struct mem_map_t
{
    //! number of different node types
    uint32_t types;
    //! sizes of single nodes
    uint32_t *sizes;

    uint8_t ***data;
    uint8_t ****free;

    uint32_t *chunk_data;
    uint32_t *chunk_free;

    uint32_t *index_data;
    uint32_t *index_free;
} mem_map;

     const uint32_t mm_sizes[PQ_MEM_WIDTH] =
    {
        0x00000001, 0x00000002, 0x00000004, 0x00000008,
        0x00000010, 0x00000020, 0x00000040, 0x00000080,
        0x00000100, 0x00000200, 0x00000400, 0x00000800,
        0x00001000, 0x00002000, 0x00004000, 0x00008000,
        0x00010000, 0x00020000, 0x00040000, 0x00080000,
        0x00100000, 0x00200000, 0x00400000, 0x00800000,
        0x01000000, 0x02000000, 0x04000000, 0x08000000,
        0x10000000, 0x20000000, 0x40000000, 0x80000000
    };

    mem_map* mm_create()
    {
            uint32_t types = 1;
            uint32_t sizes[1] =
            {
                sizeof( pq_node_type )
            };

            uint32_t capacities[1] =
            {
                473254
            };

        int i;

        mem_map *map = (mem_map*) malloc( sizeof( mem_map ) );
        map->types = types;
        map->sizes = (uint32_t*) malloc( types * sizeof( uint32_t ) );
        map->data = (uint8_t***) malloc( types * sizeof( uint8_t* ) );
        map->free = (uint8_t****) malloc( types * sizeof( uint8_t** ) );
        map->chunk_data = (uint32_t*) calloc( types, sizeof( uint32_t ) );
        map->chunk_free = (uint32_t*) calloc( types, sizeof( uint32_t ) );
        map->index_data = (uint32_t*) calloc( types, sizeof( uint32_t ) );
        map->index_free = (uint32_t*) calloc( types, sizeof( uint32_t ) );


        for( i = 0; i < types; i++ )
        {
            map->sizes[i] = sizes[i];

            map->data[i] = (uint8_t**) calloc( PQ_MEM_WIDTH, sizeof( uint8_t* ) );
            map->free[i] = (uint8_t***) calloc( PQ_MEM_WIDTH, sizeof( uint8_t** ) );

            map->data[i][0] = (uint8_t*)  malloc( map->sizes[i] );
            map->free[i][0] = (uint8_t**) malloc( sizeof( uint8_t* ) );
        }

        return map;
    }

    void mm_destroy( mem_map *map )
    {
        int i, j;
        for( i = 0; i < map->types; i++ )
        {
            for( j = 0; j < PQ_MEM_WIDTH; j++ )
            {
                if( map->data[i][j] != NULL )
                    free( map->data[i][j] );
                if( map->free[i][j] != NULL )
                    free( map->free[i][j] );
            }

            free( map->data[i] );
            free( map->free[i] );
        }

        free( map->data );
        free( map->free );
        free( map->sizes );
        free( map->chunk_data );
        free( map->chunk_free );
        free( map->index_data );
        free( map->index_free );

        free( map );
    }

    void mm_clear( mem_map *map )
    {
        int i;
        for( i = 0; i < map->types; i++ )
        {
            map->chunk_data[i] = 0;
            map->chunk_free[i] = 0;
            map->index_data[i] = 0;
            map->index_free[i] = 0;
        }
    }

    void* pq_alloc_node( mem_map *map, uint32_t type )
    {
        void *node;
        if ( map->chunk_free[type] == 0 && map->index_free[type] == 0 )
        {
            if( map->index_data[type] == mm_sizes[map->chunk_data[type]] )
                mm_grow_data( map, type );

            node = ( map->data[type][map->chunk_data[type]] + ( map->sizes[type] *
                (map->index_data[type])++ ) );
        }
        else
        {
            if( map->index_free[type] == 0 )
                map->index_free[type] = mm_sizes[--(map->chunk_free[type])];

            node =
                map->free[type][map->chunk_free[type]][--(map->index_free[type])];
        }

        memset( node, 0, map->sizes[type] );

        return node;
    }

    void pq_free_node( mem_map *map, uint32_t type, void *node )
    {
        if( map->index_free[type] == mm_sizes[map->chunk_free[type]] )
            mm_grow_free( map, type );

        map->free[type][map->chunk_free[type]][(map->index_free[type])++] = (uint8_t*) node;
    }

    //==============================================================================
    // STATIC METHODS
    //==============================================================================

     void mm_grow_data( mem_map *map, uint32_t type )
    {
        uint32_t chunk = ++(map->chunk_data[type]);
        map->index_data[type] = 0;

        if( map->data[type][chunk] == NULL )
            map->data[type][chunk] = (uint8_t*) malloc( map->sizes[type] * mm_sizes[chunk] );
    }

 void mm_grow_free( mem_map *map, uint32_t type )
{
    uint32_t chunk = ++(map->chunk_free[type]);
    map->index_free[type] = 0;

    if( map->free[type][chunk] == NULL )
        map->free[type][chunk] = (uint8_t**)  malloc( map->sizes[type] * mm_sizes[chunk] );
}*///lazy

    /*#define PQ_MEM_WIDTH 32


    typedef struct mem_map_t
    {
        //! number of different node types
        uint32_t types;
        //! sizes of single nodes
        uint32_t *sizes;
        //! number of each type of node
        uint32_t *capacities;

        uint8_t **data;
        uint8_t ***free;

        uint32_t *index_data;
        uint32_t *index_free;
    } mem_map;


    mem_map* mm_create()
    {
        uint32_t types = 1;
        uint32_t sizes[1] =
        {
            sizeof( pq_node_type )
        };

        uint32_t capacities[1] =
        {
            473254
        };

        int i;

        mem_map *map = (mem_map *) malloc( sizeof( mem_map ) );
        map->types = types;
        map->sizes =  (uint32_t *)malloc( types * sizeof( uint32_t ) );
        map->capacities = (uint32_t *) malloc( types * sizeof( uint32_t ) );
        map->data = (uint8_t **) malloc( types * sizeof( uint8_t* ) );
        map->free = (uint8_t ***) malloc( types * sizeof( uint8_t** ) );
        map->index_data = (uint32_t *) calloc( types, sizeof( uint32_t ) );
        map->index_free = (uint32_t *) calloc( types, sizeof( uint32_t ) );


        for( i = 0; i < types; i++ )
        {
            map->sizes[i] = sizes[i];
            map->capacities[i] = capacities[i];

            map->data[i] = (uint8_t *) calloc( PQ_MEM_WIDTH, sizeof( uint8_t* ) );
            map->free[i] = (uint8_t **) calloc( PQ_MEM_WIDTH, sizeof( uint8_t** ) );

            map->data[i] =(uint8_t *) malloc( map->sizes[i] * map->capacities[i] );
            map->free[i] = (uint8_t **) malloc( sizeof( uint8_t* ) * map->capacities[i] );
        }

        return map;
    }

    void mm_destroy( mem_map *map )
    {
        int i;
        for( i = 0; i < map->types; i++ )
        {
            free( map->data[i] );
            free( map->free[i] );
        }

        free( map->data );
        free( map->free );
        free( map->capacities );
        free( map->sizes );

        free( map );
    }

    void mm_clear( mem_map *map )
    {
        int i;
        for( i = 0; i < map->types; i++ )
        {
            map->index_data[i] = 0;
            map->index_free[i] = 0;
        }
    }

    void* pq_alloc_node( mem_map *map, uint32_t type )
    {
        void *node;
        if ( map->index_free[type] == 0 )
            node = ( map->data[type] + ( map->sizes[type] *
                (map->index_data[type])++ ) );
        else
            node = map->free[type][--(map->index_free[type])];

        memset( node, 0, map->sizes[type] );

        return node;
    }

    void pq_free_node( mem_map *map, uint32_t type, void *node )
    {
        map->free[type][(map->index_free[type])++] = (uint8_t *) node;
    }*///eager


    #define PQ_MEM_WIDTH 32

    /**
     * Dummy API for node allocation.  Just makes simple calls to associated system
     * functions.
     */

    typedef struct mem_map_t
    {
        //! number of different node types
        uint32_t types;
        //! sizes of single nodes
        uint32_t *sizes;
    } mem_map;


    mem_map* mm_create( )
    {
        uint32_t types = 1;
        uint32_t sizes[1] =
        {
            sizeof( pq_node_type )
        };

        uint32_t capacities[1] =
        {
            473254
        };


        mem_map *map = (mem_map *) malloc( sizeof( mem_map ) );
        map->types = types;
        map->sizes = (uint32_t*) malloc( types * sizeof( uint32_t ) );
        map->sizes[0] = sizes[0];

        return map;
    }

    void mm_destroy( mem_map *map )
    {
        free( map->sizes );
        free( map );
    }

    void mm_clear( mem_map *map )
    {
        return;
    }

    void* pq_alloc_node( mem_map *map, uint32_t type )
    {
        void *node = calloc( 1, map->sizes[type] );

        return node;
    }

    void pq_free_node( mem_map *map, uint32_t type, void *node )
    {
        free( node );
    }


    /**
     * Holds an inserted element, as well as pointers to maintain tree
     * structure.  Acts as a handle to clients for the purpose of
     * mutability.  Each node is contained in a doubly linked list of
     * siblings and has a pointer to it's first child.  If a node is the
     * first of its siblings, then its prev pointer points to their
     * collective parent.  The last child is marked by a null next pointer.
     */
    struct pairing_node_t
    {
        //! First child of this node
        struct pairing_node_t *child;
        //! Next node in the list of this node's siblings
        struct pairing_node_t *next;
        //! Previous node in the list of this node's siblings
        struct pairing_node_t *prev;

        //! Pointer to a piece of client data
        DataType item;
        //! Key for the item
        KeyType key;
    } __attribute__ ((aligned(4)));

    typedef struct pairing_node_t pairing_node;
    typedef pairing_node pq_node_type;
    typedef pairing_node PQNodeType;


    /**
     * A mutable, meldable, two-pass Pairing heap.  Maintains a single multiary tree
     * with no structural constraints other than the standard heap invariant.
     * Handles most operations through cutting and pairwise merging.  Primarily uses
     * iteration for merging rather than the standard recursion methods (due to
     * concerns for stackframe overhead).
     */
    struct pairing_heap_t
    {
        //! Memory map to use for node allocation
        mem_map *map;
        //! The number of items held in the queue
        uint32_t size;
        //! Pointer to the minimum node in the queue
        pairing_node *root;
    } __attribute__ ((aligned(4)));


    typedef struct pairing_heap_t pairing_heap;
    typedef pairing_heap pq_type;

    Pairing_Heap()
    {
        create();
    }

    ~Pairing_Heap()
    {
        destroy();
    }

    void reserve( uint32_t size)
    {
        return;
    }

    /**
     * Deletes all nodes, leaving the queue empty.
     */
    void clear()
    {
        mm_clear(queue->map);
        queue->root = NULL;
        queue->size = 0;
    }

    /**
     * Returns the key associated with the queried node.
     *
     * @param node  Node to query
     * @return      Node's key
     */
    const KeyType& getKey( const PQNodeType* node) const
    {
        return node->key;
    }

    /**
     * Returns the item associated with the queried node.
     *
     * @param node  Node to query
     * @return      Node's item
     */
    DataType& getData( const PQNodeType* node) const
    {
        return node->item;
    }

    /**
     * Returns the current size of the queue.
     *
     * @return      Size of queue
     */
    uint32_t size() const
    {
        return queue->size;
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
        PQNodeType* wrapper = (PQNodeType*) pq_alloc_node( queue->map, 0 );
        wrapper->item = item;
        wrapper->key = key;
        queue->size++;

        queue->root = merge( queue->root, wrapper);

        return wrapper;
    }

    /**
     * Returns the minimum item from the queue without modifying any data.
     *
     * @return      Node with minimum key
     */
    PQNodeType* findMin() const
    {
        if( empty())
            return NULL;
        return queue->root;
    }

    const KeyType& minKey() const
    {
        return queue->root->key;
    }

    const DataType& minItem() const
    {
        return queue->root->item;
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
        /*std::cout << "\nkey:" << queue->root->key << " ";
        std::cout << queue->size;
        {int a; std::cin>>a;}*/
        return remove( queue->root );
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

        if ( node == queue->root )
            queue->root = collapse( node->child );
        else
        {
            if ( node->prev->child == node )
                node->prev->child = node->next;
            else
                node->prev->next = node->next;

            if ( node->next != NULL )
                node->next->prev = node->prev;

            queue->root = merge( queue->root, collapse( node->child ) );
        }

        pq_free_node( queue->map, 0, node );
        queue->size--;

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
        if ( node == queue->root )
            return;

        if ( node->prev->child == node )
            node->prev->child = node->next;
        else
            node->prev->next = node->next;

        if ( node->next != NULL )
            node->next->prev = node->prev;

        queue->root = merge( queue->root, node );
    }

    /**
     * Determines whether the queue is empty, or if it holds some items.
     *
     * @return      True if queue holds nothing, false otherwise
     */
    bool empty() const
    {
        return ( queue->size == 0 );
    }

 private:

    /**
     * Merges two nodes together, making the item of greater key the child
     * of the other.
     *
     * @param a     First node
     * @param b     Second node
     * @return      Resulting tree root
     */
    PQNodeType* merge( PQNodeType *a, PQNodeType *b )
    {
        PQNodeType *parent, *child;

        if ( a == NULL )
            return b;
        else if ( b == NULL )
            return a;
        else if ( a == b )
            return a;

        if ( b->key < a->key )
        {
            parent = b;
            child = a;
        }
        else
        {
            parent = a;
            child = b;
        }

        child->next = parent->child;
        if ( parent->child != NULL )
            parent->child->prev = child;
        child->prev = parent;
        parent->child = child;

        parent->next = NULL;
        parent->prev = NULL;

        return parent;
    }

    /**
     * Performs an iterative pairwise merging of a list of nodes until a
     * single tree remains.  Implements the two-pass method without using
     * explicit recursion (to prevent stack overflow with large lists).
     * Performs the first pass in place while maintaining only the minimal list
     * structure needed to iterate back through during the second pass.
     *
     * @param node  Head of the list to collapse
     * @return      Root of the collapsed tree
     */
    PQNodeType* collapse( PQNodeType *node )
    {
        PQNodeType *tail, *a, *b, *next, *result;

        if ( node == NULL )
            return NULL;

        next = node;
        tail = NULL;
        while ( next != NULL )
        {
            a = next;
            b = a->next;
            if ( b != NULL )
            {
                next = b->next;
                result = merge( a, b );
                // tack the result onto the end of the temporary list
                result->prev = tail;
                tail = result;
            }
            else
            {
                a->prev = tail;
                tail = a;
                break;
            }
        }

        result = NULL;
        while ( tail != NULL )
        {
            // trace back through to merge the list
            next = tail->prev;
            result = merge( result, tail );
            tail = next;
        }

        return result;
    }

    pairing_heap *queue;

    /**
     * Creates a new, empty queue.
     *
     * @param map   Memory map to use for node allocation
     * @return      Pointer to the new queue
     */
    void create()
    {
        queue = (pairing_heap*) calloc( 1, sizeof( pairing_heap ) );
        queue->map = (mem_map*) mm_create();
    }

    /**
     * Frees all the memory used by the queue.
     */
    void destroy()
    {
        clear();
        mm_destroy(queue->map);
        free(queue);
    }
};