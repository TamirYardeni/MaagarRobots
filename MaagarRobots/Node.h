/*
 * Node.h
 *
 *  Created on: Jun 23, 2017
 *      Author: user
 */

#ifndef SRC_NODE_H_
#define SRC_NODE_H_

struct Node {
	int row;
	int col;
	float f;
	float g;
	float h;
	bool finished;
	Node *parent;
};

#endif /* SRC_NODE_H_ */
