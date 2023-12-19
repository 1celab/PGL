#ifndef TIME_EXPANDED_DEFINITIONS_H
#define TIME_EXPANDED_DEFINITIONS_H

#include <climits>

typedef unsigned int StationID;
typedef unsigned int StopID;
typedef unsigned int RouteID;
typedef unsigned int TripID;
typedef unsigned int NodeID;
typedef unsigned int VehicleID;
typedef int VehicleTypeID;
typedef unsigned int TrainID;
typedef unsigned int BusID;
typedef unsigned int EventID;
typedef int Distance;
typedef unsigned int Time;
typedef unsigned int PQRange;
typedef unsigned int TTL;

typedef unsigned int SwitchNodeID;

const Distance INF = std::numeric_limits<Distance>::max();

#endif // TIME_EXPANDED_DEFINITIONS_H
