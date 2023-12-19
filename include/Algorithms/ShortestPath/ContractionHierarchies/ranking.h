#include <Utilities/geographic.h>


template< typename GraphType>
void rankNodes(GraphType& G)
{
    typedef typename GraphType::NodeIterator    NodeIterator;

    NodeIterator u,lastNode;
    unsigned int minX,minY,maxX,maxY,centerX,centerY,maxDistance;
    for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
    {
        if( u->x < minX)
        {
            minX = u->x;
        }
        if( u->y < minY)
        {
            minY = u->y;
        }
        if( u->x > maxX)
        {
            maxX = u->x;
        }
        if( u->y > maxY)
        {
            maxY = u->y;
        }
    }

    centerX = minX + (maxX - minX)/2;
    centerY = minY + (maxY - minY)/2;

    maxDistance = euclideanDistance( maxX, maxY, centerX, centerY);

    for( u = G.beginNodes(), lastNode = G.endNodes(); u != lastNode; ++u)
    {
        u->rank = maxDistance - euclideanDistance( u->x, u->y, centerX, centerY);
    }
}
