#ifndef PRIORITY_QUEUE_NODEC_H
#define PRIORITY_QUEUE_NODEC_H

#include <Structs/Trees/completeBinaryTree.h>
#include <limits>

typedef unsigned int PQSizeType;

template <typename KeyType, typename DataType>
class HeapItemNoDec
{

 public:

    typedef PQSizeType* DescriptorType;

    HeapItemNoDec( unsigned int init = 0):key(std::numeric_limits<KeyType>::max()),data()
    {
    }

    HeapItemNoDec( const KeyType& k, const DataType& d, const DescriptorType& p):key(k),data(d)
    {
    }

    const KeyType& getKey() const
    {
        return key;
    }

    const DataType& getData() const
    {
        return data;
    }

    HeapItemNoDec operator = ( const HeapItemNoDec& other)
    {
        if( this != &other)
        {
            key = other.key;
            data = other.data;
        }
        return *this;
    }

    void swapWith( HeapItemNoDec& other)
    {
        std::swap(this->key, other.key);
        std::swap(this->data, other.data);
    }

    KeyType key;
    DataType data;
    //HeapItem* parent;
    //HeapItem* leftChild;
    //HeapItem* rightChild;
};


template <typename KeyType, typename DataType, template <typename datatype> class StorageType>
class StaticNoDecPriorityQueue
{

public:
    typedef PQSizeType SizeType;
    typedef PQSizeType* DescriptorType;
    typedef CompleteBinaryTree< HeapItemNoDec<KeyType,DataType>, StorageType> TreeType;
    typedef HeapItemNoDec<KeyType,DataType> PQItem;
    typedef typename TreeType::Node Node;

	class PQVisitor
	{
	public:
		PQVisitor()
		{
		}

		virtual void visit( Node& u)
		{
		}
	};

    StaticNoDecPriorityQueue():m_numItems(0)
    {
        m_auxNode = m_T.getRootNode();
        m_lastNode = m_T.getRootNode();
    }

    ~StaticNoDecPriorityQueue()
    {
    }

    void reserve( unsigned int numNodes)
    {
        m_T.reserve( numNodes);

        m_auxNode = m_T.getRootNode();
        m_lastNode = m_T.getRootNode();
    }

    const PQItem* getData() const
    {
        return m_T.getData();
    }

    void clear()
    {
        m_numItems = 0;
    }

    bool empty() const
    {
        return m_numItems == 0;
    }

    /**
     * @brief Insert a key-value pair to the priority queue
     * @param key The key of the new element
     * @param data The assorted data for the new element
     */
    void insert( const KeyType& key, const DataType& data)
    {
        increaseSize();
        m_auxNode.setAtBfsIndex( lastItemBfsIndex());
        m_auxNode->key = key;
        m_auxNode->data = data;

        upheap(m_auxNode);
    }

    const PQItem& min() const
    {
        Node auxNode = m_auxNode;
        auxNode.setAtRoot();
        return *auxNode;
    }

    const KeyType& minKey() const
    {
        return min().key;
    }

    const DataType& minItem() const
    {
        return min().data;
    }


    void popMin()
    {
        assert( m_numItems > 0);
        m_auxNode.setAtRoot();

        certainDownheap( m_auxNode);
        m_lastNode.setAtBfsIndex( lastItemBfsIndex() );
        if( m_auxNode != m_lastNode)
        {
            swap( m_auxNode, m_lastNode);
            upheap( m_auxNode);
        }

        decreaseSize();
    }

    /*void printGraphviz( const std::string& filename)
    {
        std::ofstream out( filename.c_str());
        m_T.printGraphviz(out);
        out.close();
    }*/

    void printGraphviz( const std::string& filename)
    {
        std::ofstream out( filename.c_str());
        out << "digraph BFS {\n\tedge [len=3]\n\tnode  [fontname=\"Arial\"]\n";

        std::stack<Node> S;
        m_auxNode.setAtRoot();
        S.push(m_auxNode);
        while( !S.empty())
        {
            m_auxNode = S.top();
            S.pop();
            out << m_auxNode.getPoolIndex();
            out << "[shape=record,label=\"{";
		    out << m_auxNode.getPoolIndex() ;//<< "|";
            //out <<  m_auxNode->key << "|";
	        out << "}\"]\n";
            out << "\t";
            if( !m_auxNode.isLeaf())
            {
                out <<  m_auxNode.getPoolIndex();
                out << " -> ";
                m_auxNode.goLeft();
                out <<  m_auxNode.getPoolIndex();
                out << "\n";
                S.push(m_auxNode);
                m_auxNode.goUp();
                out << "\t";
                out <<  m_auxNode.getPoolIndex();
                out << " -> ";
                m_auxNode.goRight();
                out <<  m_auxNode.getPoolIndex();
                S.push(m_auxNode);
            }
            out << "\n";
        }
        out << "}";
        out.close();
    }

    const SizeType& size()
    {
        return m_numItems;
    }

	void visit( PQVisitor* vis)
    {
        std::stack<Node> S;
        m_auxNode.setAtRoot();
        S.push(m_auxNode);
        while( !S.empty())
        {
            m_auxNode = S.top();
            S.pop();
			vis->visit( m_auxNode);

            if( !m_auxNode.isLeaf())
            {
                m_auxNode.goLeft();
                S.push(m_auxNode);
                m_auxNode.goUp();
                m_auxNode.goRight();
                S.push(m_auxNode);
            }
        }
    }

    TreeType m_T;

private:

    SizeType m_numItems;
    enum pos { PARENT, LEFT, RIGHT} m_minNodePos;
    KeyType m_minKey;
    Node m_auxNode, m_parentNode, m_lastNode, m_left, m_right;
    PQItem m_tempItem;

    void certainDownheap( Node& u)
    {
        while( !u.isLeaf())
        {
            m_left = u;
            m_left.goLeft();
            m_right = u;
            m_right.goRight();
            if( !isInHeap( m_left)) return;

            if( !isInHeap( m_right))
            {
                swap( u, m_left);
                u = m_left;
                return;
            }

            if( m_left->key < m_right->key)
            {
                swap( u, m_left);
                u = m_left;
            }
            else
            {
                swap( u, m_right);
                u = m_right;
            }
        }
    }

    void decreaseSize()
    {
        --m_numItems;
    }

    void downheap( Node& u)
    {
        while( !u.isLeaf())
        {
            m_minKey = u->key;
            m_minNodePos = PARENT;
            m_left = u;
            m_left.goLeft();
            if( isInHeap(m_left) && ( m_left->key < m_minKey) )
            {
                m_minKey = m_left->key;
                m_minNodePos = LEFT;
            }

            m_right = u;
            m_right.goRight();
            if( isInHeap(m_right) && ( m_right->key < m_minKey) )
            {
                m_minKey = m_right->key;
                m_minNodePos = RIGHT;
            }

            if( m_minNodePos == PARENT) return;

            if( m_minNodePos == LEFT)
            {
                swap( u, m_left);
                u = m_left;
            }
            else
            {
                swap( u, m_right);
                u = m_right;
            }
        }
    }

    void increaseSize()
    {
        ++m_numItems;
    }

    bool isInHeap( const Node& u) const
    {
        return u.getBfsIndex() <= lastItemBfsIndex();
    }

    const SizeType& lastItemBfsIndex() const
    {
        return m_numItems;
    }

    void swap( Node& u, Node& v)
    {
        //m_tempItem = *v;
        //*v = *u;
        //*u = m_tempItem;

        //std::swap(*u,*v);

        u->swapWith(*v);

    }

    void upheap( Node& u)
    {
        m_parentNode = u;
        while( !u.isRoot())
        {
            m_parentNode.goUp();
            if( m_parentNode->key > u->key)
            {
                swap( u, m_parentNode);
                u = m_parentNode;
            }
            else
            {
                return;
            }
        }
    }

};


#endif //PRIORITY_QUEUE_H
