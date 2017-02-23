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

		// Constructor
		Pathfinding(const ArMap kMap, ArRobot* kRobot, const ArPose kStartPose, const ArPose kGoalPose);
		// Deconstructor
		virtual ~Pathfinding() {}
	
		// Creates Nodes for pathfinding
		void createNodes();

		// Creates a path to target Node
		void createPathTo(std::shared_ptr<Node> targetNode);

		// Returns the Node that is closest to the position
		std::shared_ptr<Node> closestNode(const ArPose kPosition);

		// Draws pathfinding representation to a display
		void draw(sf::RenderTarget& target);

		// Pops the Node off the front of the path queue
		void popPath() { m_path.pop(); }

		// Returns the size of the View
		sf::Vector2f getViewSize() { return m_mapSize + m_displayBezel*2.0f; }

		// Returns the goal pose
		ArPose getGoalPose() { return m_goalPose; }
		// Returns the start pose
		ArPose getStartPose() { return m_startPose; }

		// Returns the vector of Nodes
		std::vector<std::shared_ptr<Node>> getNodes() { return m_pNodes; }

		// Returns the path queue
		std::queue<ArPose> getPath() { return m_path; }

	private:
	
		// Pointer to Robot
		ArRobot* m_pRobot;

		// Start pose
		ArPose m_startPose;
		// Goal pose
		ArPose m_goalPose;

		// Diameter of the Nodes
		double m_dNodeDiameter;

		// Two corners of the map in world coords
		ArPose m_mapUpperBounds = ArPose(INT32_MIN, INT32_MIN);
		ArPose m_mapLowerBounds = ArPose(INT32_MAX, INT32_MAX);
		// Size of the map
		sf::Vector2f m_mapSize;

		// Vector of Nodes
		std::vector<std::shared_ptr<Node>> m_pNodes;
		// Queue of Poses forming a path
		std::queue<ArPose> m_path;

		// Whether Nodes have been initialised
		bool m_bNodesInit = false;

		// Display Bezel
		sf::Vector2f m_displayBezel;

		// Map Data
		ArMap m_map;

		// Calculates which Nodes are accessible
		void calcAccessibility();

		// Calculates G value of a Node
		double calcG(std::shared_ptr<Node> currentNode, std::shared_ptr<Node> targetNode);

		// Returns whether a Node is within a distance of a Line
		bool nodeNearLine(const ArLineSegment kLine, const ArPose kNodePos, const double kDistance);

		// Returns whether a Node is within a vector of Nodes
		bool nodeInVector(std::shared_ptr<Node> nodeToFind, std::vector<std::shared_ptr<Node>> vector);
		
		// Returns the distance from one Node to another
		int manhattanDist(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode);

		// Returns a vector of accessible adjacent Nodes
		std::vector<std::shared_ptr<Node>> getAdjacentNodes(std::shared_ptr<Node> node);

		// Returns a Node at a given pose
		std::shared_ptr<Node> nodeFromPose(const ArPose kPosition);
		
		// Forms a queue of Nodes to the given Node
		void queuePath(std::shared_ptr<Node> node);
};

#endif