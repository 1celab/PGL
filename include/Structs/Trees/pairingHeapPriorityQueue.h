#ifndef PAIRING_HEAP_H
#define PAIRING_HEAP_H

template <typename KeyType, typename DataType>
class Pairing_Heap
{
 public:

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
        pairing_node_t()
        {
            clear();
        }

        void clear()
        {
            child = 0;
            next = 0;
            prev = 0;
        }

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
    };

    typedef struct pairing_node_t pairing_node;
    typedef pairing_node pq_node_type;
    typedef pairing_node PQNodeType;

    Pairing_Heap()
    {
        clear();
    }

    ~Pairing_Heap()
    {}

    void reserve( uint32_t size)
    {
        pool.resize(size);
    }

    /**
     * Deletes all nodes, leaving the queue empty.
     */
    void clear()
    {
        //pool.clear();
        poolRoot = NULL;
        numItems = 0;
        poolSize = 0;
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
        return numItems;
    }

    /**
     * Takes an item-key pair to insert it into the queue and creates a new
     * corresponding node.  Merges the new node with the root of the queue.
     *
     * @param key   Key to use for node priority
     * @param item  Item to insert
     * @return      Pointer to corresponding node
     */
    PQNodeType* insert( const KeyType& key, const DataType& item)
    {
        pool[poolSize].clear();
        pool[poolSize].item = item;
        pool[poolSize].key = key;
        poolRoot = merge( poolRoot, &(pool[poolSize]));
        numItems++;
        poolSize++;

        return &(pool[poolSize-1]);
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
        return poolRoot;
    }

    const KeyType& minKey() const
    {
        return poolRoot->key;
    }

    const DataType& minItem() const
    {
        return poolRoot->item;
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
        /*std::cout << "\nkey:" << poolRoot->key << " ";
        std::cout << queue->size;
        {int a; std::cin>>a;}*/
        return remove( poolRoot );
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

        if ( node == poolRoot )
            poolRoot = collapse( node->child );
        else
        {
            if ( node->prev->child == node )
                node->prev->child = node->next;
            else
                node->prev->next = node->next;

            if ( node->next != NULL )
                node->next->prev = node->prev;

            poolRoot = merge( poolRoot, collapse( node->child ) );
        }

        numItems--;

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
        if ( node == poolRoot )
            return;

        if ( node->prev->child == node )
            node->prev->child = node->next;
        else
            node->prev->next = node->next;

        if ( node->next != NULL )
            node->next->prev = node->prev;

        poolRoot = merge( poolRoot, node );
    }

    /**
     * Determines whether the queue is empty, or if it holds some items.
     *
     * @return      True if queue holds nothing, false otherwise
     */
    bool empty() const
    {
        return ( numItems == 0 );
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

    std::vector<pairing_node> pool;
    pairing_node* poolRoot;
    unsigned int poolSize;
    unsigned int numItems;
};

#endif
