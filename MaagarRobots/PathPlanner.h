/*
 * PathPlanner.h
 *
 *  Created on: Jun 23, 2017
 *      Author: user
 */

#ifndef SRC_PATHPLANNER_H_
#define SRC_PATHPLANNER_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include <vector>
#include <queue>
#include <math.h>
#include <algorithm>
#include "opencv/cv.h"
#include "Node.h"

using namespace HamsterAPI;
using namespace std;

typedef vector<Node* > Path;

class PathPlanner {
private:
	OccupancyGrid *grid;
	int startRow, startCol;
	vector<vector<Node*> > mat;

	void buildGraph(Node *goal);
	vector<Node *> getAdjacentNodes(const Node *node);
	float getEuclideanDistance(const Node *successor, const Node *goal);
	bool isSameNode(const Node *node1, const Node *node2);
	Node* findNodeOnList(deque<Node*> nodes_, Node *node_);
	Path* findPath(Node *goal);
	Path* reversePath(Path *path);

public:
	PathPlanner(OccupancyGrid *grid, int startRow, int startCol);
	Path* getPath(cv::Point2i goal);
	virtual ~PathPlanner();
};

#endif /* SRC_PATHPLANNER_H_ */
