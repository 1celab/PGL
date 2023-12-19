#ifndef GEOGRAPHIC_H
#define GEOGRAPHIC_H

#include <cmath>
#include <math.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

#if !defined(PI)
    #define PI 3.14159265358979323846
#endif

class GeoPoint2D
{
  public:

    GeoPoint2D()
    {
        setLatitude( std::numeric_limits<double>::infinity());
        setLongitude( std::numeric_limits<double>::infinity());
    }

    GeoPoint2D( const double latitude, const double longitude)
    {
        setLatitude( latitude);
        setLongitude( longitude);
    }

    GeoPoint2D( const GeoPoint2D& point)
    {
        *this = point;
    }

    bool hasValidCoords()
    {
        return ( std::isinf(lon) == false && std::isinf(lat) == false);
    }

    void setPoint( const GeoPoint2D& point)
    {
        *this = point;
    }

    const GeoPoint2D& getPoint()
    {
        return *this;
    }

    void setLatitude( const double latitude)
    {
        lat = latitude;
    }

    void setLongitude( const double longitude)
    {
        lon = longitude;
    }

    const double& getLatitude() const
    {
      return lat;
    }

    const double& getLongitude() const
    {
        return lon;
    }

    void readPoint( std::istream& in)
    {
        in >> (*this);
    }

    void writePoint( std::ostream& out)
    {
        out << getLatitude() << " " << getLongitude();
    }

    friend std::istream& operator>>( std::istream& in, GeoPoint2D& point)
    {
        std::string str;
        in >> str;
        point.lat = std::stod( str);
        in >> str;
        point.lon = std::stod( str);
        return in;
    }

    friend std::ostream& operator<<( std::ostream& out, const GeoPoint2D& point)
    {
        out << "(" << point.lat << ", " << point.lon << ")";
        return out;
    }

    GeoPoint2D& operator=( const GeoPoint2D& point)
    {
        setLatitude( point.getLatitude());
        setLongitude( point.getLongitude());
        return *this;
    }

    bool operator==(const GeoPoint2D& rhs)
    {
       return( ( getLatitude() == rhs.getLatitude()) && ( getLongitude() == rhs.getLongitude()));
    }

    bool operator!=(const GeoPoint2D& rhs)
    {
       return !(*this == rhs);
    }

    bool operator<(const GeoPoint2D& rhs) const
    {
       if( getLatitude() < rhs.getLatitude())
           return true;
       else if( getLatitude() == rhs.getLatitude() && getLongitude() < rhs.getLongitude())
	   return true;
       return false;
    }

    /**
    *@brief calculate the distance in meters between two points
    *@param Latitude and Longitude of the points.
    */
    double distance( const GeoPoint2D p2) const
    {
        return distance( *this, p2);
    }

    static GeoPoint2D getPoint( const GeoPoint2D& p, const double angle, const double d)
    {
        const double R = 6378.1; //radius of the Earth in km
        double brng = angle*(PI/180.0); //bearing is degrees converted to radians.

        double dlat1 = p.getLatitude()*(PI/180.0);
        double dlon1 = p.getLongitude()*(PI/180.0);

        double dlat2 = asin( sin(dlat1)*cos(d/R) +
             cos(dlat1)*sin(d/R)*cos(brng));

        double dlon2 = dlon1 + atan2(sin(brng)*sin(d/R)*cos(dlat1),
                       cos(d/R)-sin(dlat1)*sin(dlat2));

        double lat2 = dlat2*(180.0/PI);
        double lon2 = dlon2*(180.0/PI);
	return GeoPoint2D( lat2, lon2);
    }

    static double distance( const GeoPoint2D& p1, const GeoPoint2D& p2)
    {
        const double& lat1 = p1.getLatitude();
        const double& lon1 = p1.getLongitude();
        const double& lat2 = p2.getLatitude();
        const double& lon2 = p2.getLongitude();

        double dlat1 = lat1*(PI/180.0);
        double dlon1 = lon1*(PI/180.0);
        double dlat2 = lat2*(PI/180.0);
        double dlon2 = lon2*(PI/180.0);

        double dlon = dlon1-dlon2;
        double dlat = dlat1-dlat2;

        double aHarv = pow(sin(dlat/2.0),2.0)+cos(dlat1)*cos(dlat2)*pow(sin(dlon/2.0),2.0);
        double cHarv = 2.0*atan2(sqrt(aHarv),sqrt(1.0-aHarv));

        //earth's radius from wikipedia varies between 6,356.750 km — 6,378.135 km (˜3,949.901 — 3,963.189 miles)
        //The IUGG value for the equatorial radius of the Earth is 6378.137 km (3963.19 mile)
        double distance = cHarv*6378137.0;

        return distance;
    }

 private:

    double lat, lon;
};

BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET( GeoPoint2D, double,
                                          //bg::cs::spherical_equatorial<bg::degree>,
                                          bg::cs::geographic<bg::degree>,
                                          getLongitude, getLatitude, setLongitude, setLatitude)

struct GeoPoint2DSet
{
    typedef std::pair<GeoPoint2D, std::size_t> PointItem;
    typedef std::vector<PointItem> PointContainer;
    bgi::rtree<PointItem, bgi::quadratic<16>> rTree;

    GeoPoint2DSet()
    {}

    GeoPoint2DSet( const PointContainer& points):
    rTree( points)
    {}

    void clear()
    {
        rTree.clear();
    }

    void insert( const PointContainer& points)
    {
        //rTree.clear();
        rTree.insert( points.begin(), points.end());
    }

    PointItem searchNearestNeighbor( const GeoPoint2D& targetPoint) const
    {
        PointContainer nearestNeighborPoint;
        rTree.query( bgi::nearest( targetPoint, 1), std::back_inserter(nearestNeighborPoint));
        return nearestNeighborPoint.back();
    }

    void searchNearestNeighbors( const GeoPoint2D& targetPoint, const std::size_t numNearestNeighborPoints, PointContainer& nearestNeighborPoints) const
    {
        nearestNeighborPoints.clear();
        nearestNeighborPoints.reserve( numNearestNeighborPoints);
        rTree.query( bgi::nearest( targetPoint, numNearestNeighborPoints), std::back_inserter(nearestNeighborPoints));
    }

    void remove( const PointItem& targetPointItem)
    {
        rTree.remove( targetPointItem);
    }
};

double deg2rad( double deg)
{
    return ( deg * PI / 180);
}

double rad2deg( double rad) {
    return ( rad * 180 / PI);
}

//TODO REMOVE !!!!!!!!!!
double haversineDistanceInMeters(double lat1, double lon1, double lat2, double lon2)
{
    double theta, dist;
    theta = lon1 - lon2;
    dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    dist = dist * 1609.344;
    return (dist);
}

//TODO REMOVE !!!!!!!!!!
double greatCircle(double lat1, double long1, double lat2, double long2)
{
    //main code inside the class
    double dlat1=lat1*(PI/180.0);

    double dlon1=long1*(PI/180.0);
    double dlat2=lat2*(PI/180.0);
    double dlon2=long2*(PI/180.0);

    double dLong=dlon1-dlon2;
    double dLat=dlat1-dlat2;

    double aHarv= pow(sin(dLat/2.0),2.0)+cos(dlat1)*cos(dlat2)*pow(sin(dLong/2.0),2.0);
    double cHarv=2.0*atan2(sqrt(aHarv),sqrt(1.0-aHarv));

    //earth's radius from wikipedia varies between 6,356.750 km — 6,378.135 km (˜3,949.901 — 3,963.189 miles)
    //The IUGG value for the equatorial radius of the Earth is 6378.137 km (3963.19 mile)
    double distance = cHarv * 6378137.0;

    return distance;
}

//TODO REMOVE !!!!!!!!!!
/**
*@brief calculate the distance in m between two points
*@param Latitude and Longitude of the points.
*/
double distance( const double lon1, const double lat1, const double lon2, const double lat2)
{
    return greatCircle( lat1, lon1, lat2, lon2);
}

double euclideanDistance( double x1, double y1, double x2, double y2)
{
    return sqrt(pow(abs( x1 - x2),2) + pow(abs( y1 - y2),2));
}

std::pair<double,double> ToGeographic(std::pair<double,double> mercator)
{
    std::pair<double,double> geographic;
    std::cout << mercator.first << " " << mercator.second << std::endl;
    if (abs(mercator.first) < 180 && abs(mercator.second) < 90)
        return std::pair<double,double>(0,0);

    if ( ( abs(mercator.first) > 20037508.3427892) || (abs(mercator.second) > 20037508.3427892) )
        return std::pair<double,double>(0,0);

    double x = mercator.first;
    double y = mercator.second;
    double num3 = x / 6378137.0;
    double num4 = num3 * 57.295779513082323;
    double num5 = floor((double)((num4 + 180.0) / 360.0));
    double num6 = num4 - (num5 * 360.0);
    double num7 = 1.5707963267948966 - (2.0 * atan(exp((-1.0 * y) / 6378137.0)));
    geographic.first = num6;
    geographic.second = num7 * 57.295779513082323;
    return geographic;
}

std::pair<double,double> ToWebMercator(std::pair<double,double> geographic)
{
    std::pair<double,double> mercator;
    std::cout << abs(geographic.first) << " " << abs(geographic.second) << std::endl;
    if ((abs(geographic.first) > 180 || abs(geographic.second) > 90))
        return std::pair<double,double>(0,0);

    double num = geographic.first * 0.017453292519943295;
    double x = 6378137.0 * num;
    double a = geographic.second * 0.017453292519943295;

    mercator.first = x;
    mercator.second = 3189068.5 * log((1.0 + sin(a)) / (1.0 - sin(a)));
    return mercator;
}

#endif //GEOGRAPHIC_H
