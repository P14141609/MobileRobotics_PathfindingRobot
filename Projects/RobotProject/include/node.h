#ifndef NODE_H
#define NODE_H

#include "utils.h"

class Node
{
	public:
		// Default Constructor
		Node() {}

		// In-world coords
		ArPose position;

		// Previous Node in the path
		std::shared_ptr<Node> parent;

		// Whether the Node is accessible for pathing
		bool bAccessible = true;

		// Position within a vector of Nodes
		unsigned int index;

		// Total Movement Cost from start Node to this Node
		double g = 0;
		// Distance to target Node
		double h = 0;
		// F Value: Sum of G + H
		double f = 0;
};

#endif