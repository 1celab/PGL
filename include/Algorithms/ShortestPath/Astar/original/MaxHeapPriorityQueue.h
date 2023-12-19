#ifndef __MAXHEAP__
#define __MAXHEAP__

#include <iostream>
#include <vector>
#include <assert.h> 


/**
 * @class MaxHeapPQ
 *
 * @brief Max Heap Vector
 *
 * @author Andreas Paraskevopoulos
 *
 */

template <typename DataType, typename KeyType>
class Item
{

 public:

    DataType info;
    KeyType  key;

    Item()
    {}

    Item(KeyType key, DataType info)
    {
        this->info = info;
        this->key  = key;
    }

    bool operator> (Item& other) const
    {
        return key > other.key;
    }

    bool operator< (Item& other) const
    {
        return key < other.key;
    }

};


template <typename DataType, typename KeyType>
class MaxHeapPQ
{

  protected:

    typedef Item<DataType, KeyType> PQItem;

    std::vector<PQItem> heap;

    unsigned int lastItem; 

  public:

    // MaxHeap contructor
    MaxHeapPQ(unsigned int heapSize)
    {
        lastItem = 0;
        heap.resize(heapSize+1);

        //null item
        heap[0] = PQItem();
    }

    MaxHeapPQ()
    {
        lastItem = 0;
        heap.push_back(PQItem());
    }

    // MaxHeap destructor
    ~MaxHeapPQ()
    {
        heap.clear();
    }


    // resize
    void resize(unsigned int heapSize)
    {
        reset();
        heap.resize(heapSize+1);
    }


    //push item
    void push(KeyType key, DataType info)
    {
        PQItem item(key, info);
        ++lastItem;
        heap[lastItem] = item;
        shiftUp(lastItem);
    }


    // pop item
    void pop()
    {
        if(lastItem > 0)
        {
            heap[1] = heap[lastItem];  // Replace with the last element
            shiftDown(1);
            //heap.pop_back();
            --lastItem;
        }
    }


    // reset
    void reset()
    {
        lastItem = 0;
    }


    // size of heap
    unsigned int size() const
    {
        return lastItem;
    }


    // get the data of max item
    DataType dataOfMax() const
    {
        return heap[1].info;
    }


    // get the priority of max item
    KeyType keyOfMax() const
    {
        return heap[1].key;
    }

 protected:

    // shiftUp
    void shiftUp(unsigned int currentNode)
    {
        unsigned int parentNode = getParentOf(currentNode);
        PQItem currentNodeItem = heap[currentNode];

        //while Current node is not the RootNode
        while (currentNode > 1)  
        {
            if (heap[parentNode] < currentNodeItem)
            {
                heap[currentNode] = heap[parentNode];
                currentNode = parentNode;
                parentNode = getParentOf(currentNode);
            }
            else
                break;
        }

        heap[currentNode] = currentNodeItem;
    }


    // shiftDown
    void shiftDown(unsigned int currentNode)
    {
        unsigned int childNode = getLeftChildOf(currentNode);
        PQItem currentNodeItem = heap[currentNode];    // Used to compare values

        const unsigned int endNode = lastItem - 1;

        while (childNode < endNode)
        {
            //compare the key of left child with the key of the right child
            if (heap[childNode] < heap[childNode+1])  
            {
                    //if the right child has larger key
                    ++childNode;
            }

            if (currentNodeItem < heap[childNode])
            {   //switch the current node and its child node
                heap[currentNode]  = heap[childNode];
                currentNode        = childNode;
                childNode          = getLeftChildOf(currentNode);
            }
            else
            {
                heap[currentNode] = currentNodeItem;
                return;
            }
        }

        //if left child is the last node
        if (childNode == endNode && currentNodeItem < heap[childNode])  
        {
            //switch the current node and its child node
            heap[currentNode] = heap[childNode];
            currentNode = childNode;
        }

        heap[currentNode] = currentNodeItem;
    }


    //get parent of node
    inline unsigned int getParentOf(const unsigned int& childNode) const
    {
        if (childNode == 1)
            return 1;

        return (childNode >> 1);
    }


    //get left child of node
    inline unsigned int getLeftChildOf(const unsigned int& parentNode) const
    {
        return (parentNode << 1);
    }

};

#endif /*__HeapTreeClassH__*/
