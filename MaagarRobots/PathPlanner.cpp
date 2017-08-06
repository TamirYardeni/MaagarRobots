/*
 * PathPlanner.cpp
 *
 *  Created on: Jun 23, 2017
 *      Author: user
 */

#include "PathPlanner.h"

PathPlanner::PathPlanner(OccupancyGrid *grid, int startRow, int startCol) : startRow(startRow), startCol(startCol) {
	this->grid = grid;
}

struct NodeComparer {
	bool operator() (const Node *n1, const Node *n2) const {
		return n1->f > n2->f;
	}
};

float PathPlanner::getEuclideanDistance(const Node *successor, const Node *goal) {
	pair<int, int> delta (abs(successor->row - goal->row), abs(successor->col - goal->col));
	//return grid->getResolution() * sqrt(pow(delta.first, 2) + pow(delta.second, 2));
	return sqrt(pow(delta.first, 2) + pow(delta.second, 2));
};

bool PathPlanner::isSameNode(const Node *node1, const Node *node2) {
	return node1->row == node2->row && node1->col == node2->col;
}

Path* PathPlanner::getPath(cv::Point2i goal) {
	Node *goalNode = new Node();
	goalNode->col = goal.x;
	goalNode->row = goal.y;
	goalNode->g = 1000;
	goalNode->h = 0;
	goalNode->f = 0;
	goalNode->finished = false;
	goalNode->parent = 0;

	buildGraph(goalNode);

	Path* path = findPath(goalNode);
	return path;
}

void PathPlanner::buildGraph(Node *goal) {
	int rows = grid->getHeight();
	int cols = grid->getWidth();

	int high_value = rows * cols;

	mat.clear();

	mat.resize(rows); // number of vectors = rows

	for (int i = 0; i < rows; i++) {
		mat[i].resize(cols); // inside each vector = columns

		for (int j = 0; j < cols; j++) {
			Cell c = grid->getCell(i, j);
			if (c == CELL_FREE) {
				Node *node = new Node();
				node->row = i;
				node->col = j;
				node->h = getEuclideanDistance(node, goal);
				node->g = high_value;
				node->f = high_value;
				node->finished = false;
				node->parent = 0;

				mat[i][j] = node;
			}
			else {
				mat[i][j] = NULL;
			}
		}
	}

	mat[goal->row][goal->col] = goal;
}

vector<Node *> PathPlanner::getAdjacentNodes(const Node *node) {
	uint row = node->row;
	uint col = node->col;

	vector<Node *> successors;

	if (row < grid->getHeight() && mat[row][col + 1]) {
		successors.push_back(mat[row][col + 1]);
	}

	if (row > 0 && mat[row][col - 1]) {
			successors.push_back(mat[row][col - 1]);
	}

	if (col < grid->getWidth() && mat[row + 1][col]) {
			successors.push_back(mat[row + 1][col]);
	}

	if (col > 0 && mat[row - 1][col]) {
			successors.push_back(mat[row - 1][col]);
	}

	return successors;
}

Path* PathPlanner::findPath(Node *goal) {
	Path* path = new Path();
	Node *startNode = mat[startRow][startCol];
	startNode->g = 0;
	startNode->finished=true;
	bool bReachedGoal = false;

	priority_queue<Node *, vector<Node *>, NodeComparer> openList;

	openList.push(startNode);

	while (!openList.empty()) {
		Node *currNode = openList.top();
        openList.pop();

		if (isSameNode(currNode, goal)) {
			bReachedGoal = true;
			break;
		}

		vector<Node *> adjacentNodes = getAdjacentNodes(currNode);

 		for (uint k = 0; k < adjacentNodes.size(); k++) {
			Node *adjacentNode = adjacentNodes[k];

			if (adjacentNode->finished) {
				continue;
			}

			openList.push(adjacentNode);

			if (adjacentNode->g > currNode->g + 1) {
				adjacentNode->g = currNode->g + 1;
				adjacentNode->f = adjacentNode->g + adjacentNode->h;
				adjacentNode->parent = currNode;
			}
		}

		currNode->finished = true;
	}

	if (bReachedGoal)
	{
		path->push_back(goal);
		Node *next = goal->parent;

		while (next) {
			path->push_back(next);
			next = next->parent;
		}
	}

	std::reverse(path->begin(), path->end());

	return path;
}

PathPlanner::~PathPlanner() {

}

