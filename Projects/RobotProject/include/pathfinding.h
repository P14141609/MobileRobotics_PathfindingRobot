#ifndef PATHFINDING_H
#define PATHFINDING_H

// SFML
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
//
#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include <queue>
#include <memory>
#include "baseAction.h"
#include "node.h"
#include "utils.h"

class Pathfinding
{
	public:

		Pathfinding(const ArMap kMap, ArRobot* kRobot, const ArPose kStartPose, const ArPose kGoalPose); // Constructor
		virtual ~Pathfinding() {}  // Deconstructor
	
		// Creates nodes for pathfinding
		void createNodes();

		// Creates a path to target node
		void createPathTo(std::shared_ptr<Node> targetNode);

		// Returns the Node that is closest to the position
		std::shared_ptr<Node> closestNode(const Vertex kPosition);

		// Draws pathfinding info to a display
		void draw(sf::RenderTarget& target);

		Vertex getViewSize() { return Vertex(m_mapSize.x + m_displayBezel*2, m_mapSize.y + m_displayBezel * 2); }
		std::vector<std::shared_ptr<Node>> getNodes() { return m_pNodes; }
		ArPose getGoalPose() { return m_goalPose; }
		std::queue<Vertex> getPath() { return m_path; }

	private:
	
		ArRobot* m_pRobot;

		ArPose m_startPose;
		ArPose m_goalPose;

		// Stores the diameter of the Nodes
		double m_dNodeDiameter;

		void calcAccessibility();

		double calcG(std::shared_ptr<Node> currentNode, std::shared_ptr<Node> targetNode);

		bool nodeNearLine(const ArLineSegment kLine, const Vertex kNodePos, const double kDistance);

		// Stores two corners of the map
		Vertex m_mapUpperBounds = Vertex(INT32_MIN, INT32_MIN);
		Vertex m_mapLowerBounds = Vertex(INT32_MAX, INT32_MAX);
		// Stores size of the map
		Vertex m_mapSize;

		std::vector<std::shared_ptr<Node>> m_pNodes;
		std::queue<Vertex> m_path;

		bool m_bNodesInit = false;

		float m_displayBezel = 150.0f;

		ArMap m_map;

		ArPose absolutePose(const ArPose kPose) { return m_startPose - kPose; }

		bool nodeInVector(std::shared_ptr<Node> nodeToFind, std::vector<std::shared_ptr<Node>> vector);
		
		// Returns a distance from one node to another
		int dist_NodeToNode(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode);

		// Returns a vector of adjacent nodes
		std::vector<std::shared_ptr<Node>> getAdjacentNodes(std::shared_ptr<Node> node);

		// Returns a Node based on position
		std::shared_ptr<Node> nodeFromPos(const Vertex kPosition);
		
		// Queues up the path of nodes needed to reach the destination
		void queuePath(std::shared_ptr<Node> node);
};

#endif