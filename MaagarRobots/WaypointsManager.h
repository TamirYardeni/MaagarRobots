/*
 * WaypointsManager.h
 *
 *  Created on: Jul 13, 2017
 *      Author: user
 */

#ifndef SRC_WAYPOINTSMANAGER_H_
#define SRC_WAYPOINTSMANAGER_H_

#include <cmath>
#include <list>
#include "HamsterAPIClientCPP/Hamster.h"
#include "opencv/cv.h"
#include "Node.h"
#include <stack>
#include "PathPlanner.h"

using namespace std;
using namespace HamsterAPI;

#define MAX_NODES_WAYPOINT 10

class WaypointsManager {
private:
	OccupancyGrid* m_coarseGrid;
	bool raytraceWillCollide(cv::Point2i start, cv::Point2i end);
	int getNextWaypoint(Path* path, int startNodeIndex,int maxNodesAhead);
public:
	WaypointsManager(OccupancyGrid* grid);
	~WaypointsManager();

	list<cv::Point2i> computeWaypoints(Path* path);
};

#endif /* SRC_WAYPOINTSMANAGER_H_ */
