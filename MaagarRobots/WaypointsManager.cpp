/*
 * WaypointsManager.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: user
 */

#include "WaypointsManager.h"

WaypointsManager::WaypointsManager(OccupancyGrid* grid) {
	m_coarseGrid = grid;
}

WaypointsManager::~WaypointsManager() {
}

bool WaypointsManager::raytraceWillCollide(cv::Point2i start, cv::Point2i end)
{
    int dx = abs(end.x - start.x);
    int dy = abs(end.y - start.y);
    int x = start.x;
    int y = start.y;
    int n = 1 + dx + dy;
    int x_inc = (end.x > start.x) ? 1 : -1;
    int y_inc = (end.y > start.y) ? 1 : -1;
    int delta = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n)
    {
    	// Continue raytrace
        if (delta > 0)
        {
            x += x_inc;
            delta -= dy;
        }
        else
        {
            y += y_inc;
            delta += dx;
        }

        // Check if it collides with something (as row,col)
		if(CELL_FREE != m_coarseGrid->getCell(y,x))
		{
			return true;
		}
    }

    return false;
}

int WaypointsManager::getNextWaypoint(Path* path, int startNodeIndex,int maxNodesAhead) {
	int nextWaypointIndex = -1;
	Node* targetNode = NULL;
	Node* startNode = (*path)[startNodeIndex];
	cv::Point2i start(startNode->col,startNode->row);

	for (;maxNodesAhead > 0;--maxNodesAhead)
	{
		int targetNodeIndex = startNodeIndex + maxNodesAhead;

		targetNode = (*path)[targetNodeIndex];
		cv::Point2i end(targetNode->col,targetNode->row);

		if(!raytraceWillCollide(start,end))
		{
			// Set waypoint to next node, we know its ok
			nextWaypointIndex = targetNodeIndex;
			break;
		}
	}

	return nextWaypointIndex;
}

list<cv::Point2i> WaypointsManager::computeWaypoints(Path* path) {
	int startNodeIndex = 0;
	int nextWaypointIndex = 0;
	list<cv::Point2i> waypoints;
	bool isDone = false;
	int pathGoalIndex = ((int)path->size()) - 1;
	int maxNodesAhead;

	do {
		maxNodesAhead = std::min(MAX_NODES_WAYPOINT,pathGoalIndex - startNodeIndex);
		nextWaypointIndex = getNextWaypoint(path,startNodeIndex,maxNodesAhead);

		if (nextWaypointIndex < 1)
		{
			cout << "[ERROR] getNextWaypoint did not find a next waypoint (start = " << startNodeIndex << ")" << endl;
			waypoints.clear();
			return waypoints;
		}
		else
		{
			Node* nextWaypoint = (*path)[nextWaypointIndex];
			waypoints.push_back(cv::Point2i(nextWaypoint->col,nextWaypoint->row));

			if (nextWaypointIndex >= pathGoalIndex)
			{
				isDone = true;
			}
		}

		startNodeIndex = nextWaypointIndex;
	}
	while(!isDone);

	return waypoints;
}
