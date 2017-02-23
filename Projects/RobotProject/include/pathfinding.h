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
		std::shared_ptr<Node> closestNode(const ArPose kPosition);

		// Draws pathfinding info to a display
		void draw(sf::RenderTarget& target);

		void popPath() { m_path.pop(); }

		sf::Vector2f getViewSize() { return m_mapSize + m_displayBezel*2.0f; }

		ArPose getGoalPose() { return m_goalPose; }
		ArPose getStartPose() { return m_startPose; }

		std::vector<std::shared_ptr<Node>> getNodes() { return m_pNodes; }
		std::queue<ArPose> getPath() { return m_path; }

	private:
	
		ArRobot* m_pRobot;

		ArPose m_startPose;
		ArPose m_goalPose;

		// Stores the diameter of the Nodes
		double m_dNodeDiameter;

		void calcAccessibility();

		double calcG(std::shared_ptr<Node> currentNode, std::shared_ptr<Node> targetNode);

		bool nodeNearLine(const ArLineSegment kLine, const ArPose kNodePos, const double kDistance);

		// Stores two corners of the map
		ArPose m_mapUpperBounds = ArPose(INT32_MIN, INT32_MIN);
		ArPose m_mapLowerBounds = ArPose(INT32_MAX, INT32_MAX);
		// Stores size of the map
		sf::Vector2f m_mapSize;

		std::vector<std::shared_ptr<Node>> m_pNodes;
		std::queue<ArPose> m_path;

		bool m_bNodesInit = false;

		sf::Vector2f m_displayBezel;

		ArMap m_map;

		bool nodeInVector(std::shared_ptr<Node> nodeToFind, std::vector<std::shared_ptr<Node>> vector);
		
		// Returns a distance from one node to another
		int manhattanDist(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode);

		// Returns a vector of adjacent nodes
		std::vector<std::shared_ptr<Node>> getAdjacentNodes(std::shared_ptr<Node> node);

		// Returns a Node based on position
		std::shared_ptr<Node> nodeFromPose(const ArPose kPosition);
		
		// Queues up the path of nodes needed to reach the destination
		void queuePath(std::shared_ptr<Node> node);
};

#endif