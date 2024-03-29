﻿
#include "pathfinding.h"

// Constructor
Pathfinding::Pathfinding(const ArMap kMap, ArRobot* kRobot, const ArPose kStartPose, const ArPose kGoalPose)
{
	m_map = kMap;

	m_pRobot = kRobot;

	m_startPose = kStartPose;
	m_goalPose = kGoalPose;

	m_dNodeDiameter = 150; // 0.15m // Robot Diameter 0.5m
}

// Creates Nodes for pathfinding
void Pathfinding::createNodes()
{
	std::cerr << "\n Generating nodes for A* pathfinding... \n";

	// For every line segment in the map
	for (unsigned int i = 0; i < m_map.getLines()->size(); i++)
	{
		// Gets the line at index i
		ArLineSegment line = m_map.getLines()->at(i);

		// If the x/y is larger than the current then replace it
		m_mapUpperBounds.setX(Utils::maxDouble(m_mapUpperBounds.getX(), line.getX1(), line.getX2()));
		m_mapUpperBounds.setY(Utils::maxDouble(m_mapUpperBounds.getY(), line.getY1(), line.getY2()));

		// If the x/y is smaller than the current then replace it
		m_mapLowerBounds.setX(Utils::minDouble(m_mapLowerBounds.getX(), line.getX1(), line.getX2()));
		m_mapLowerBounds.setY(Utils::minDouble(m_mapLowerBounds.getY(), line.getY1(), line.getY2()));
	}
	
	// Creates a vertex of the world size
	m_mapSize = sf::Vector2f(m_mapUpperBounds.getX() - m_mapLowerBounds.getX(), m_mapUpperBounds.getY() - m_mapLowerBounds.getY());
	
	// Divides the map size by the diameter of the robot and rounds down to nearest whole number
	sf::Vector2f dividedMapSize = m_mapSize / (float)m_dNodeDiameter;

	// For the number of nodes needed for the width
	for (unsigned int x = 1; x <= dividedMapSize.x; x++)
	{
		// For the number of nodes needed for the height
		for (unsigned int y = 1; y <= dividedMapSize.y; y++)
		{
			// Declares new Node
			std::shared_ptr<Node> newNode = std::shared_ptr<Node>(new Node());
			
			// Sets the new Node's world position
			newNode->position.setX(m_mapLowerBounds.getX() + (m_dNodeDiameter * x) - m_dNodeDiameter*0.5);
			newNode->position.setY(m_mapLowerBounds.getY() + (m_dNodeDiameter * y) - m_dNodeDiameter*0.5);
			
			// Sets new Node's index member
			newNode->index = m_pNodes.size();

			// Pushes the new Node onto vector member of Nodes
			m_pNodes.push_back(newNode);
		}
	}

	// Flags NodesInitialised boolean member as true
	m_bNodesInit = true;

	std::cerr << "\n Generating nodes for A* pathfinding... Finished.\n";

	// Calculates the accessibility of the Nodes just created
	calcAccessibility();
}

// Calculates which Nodes are accessible
void Pathfinding::calcAccessibility()
{
	std::cerr << "\n Calculating node accessibility... \n";
	
	// For every line segment in the map
	for (unsigned int i = 0; i < m_map.getLines()->size(); i++)
	{
		// Gets the line at index i
		ArLineSegment line = m_map.getLines()->at(i);
	
		// For all Nodes
		for (std::shared_ptr<Node> node : m_pNodes)
		{
			// If Node is accessible
			if (node->bAccessible)
			{
				// If Node is near line: distance(RobotRadius*1.5) for leeway
				if (nodeNearLine(line, node->position, m_pRobot->getRobotRadius()*1.5))
				{
					// Node is too close to line: set inaccessible
					node->bAccessible = false;
				}
			}
		}
	}

	std::cerr << "\n Calculating node accessibility... Finished.\n";
}

// Creates a path to target Node
void Pathfinding::createPathTo(std::shared_ptr<Node> targetNode)
{
	std::cerr << "\n Generating path... \n";

	// If Nodes initialised
	if (m_bNodesInit)
	{
		///////////////////// Calculating Node Heuristics /////////////////////

		// For every Node, set h as dist to target Node
		for (std::shared_ptr<Node> node : m_pNodes)
		{
			// Sets the Node's heuristic as the manhattan distance from the Node to the target
			node->h = manhattanDist(node, targetNode);
		}

		///////////////////// Creating Open and Closed Lists /////////////////////

		// Declares vectors to store nodes to check and checked nodes
		std::vector<std::shared_ptr<Node>> openNodes;
		std::vector<std::shared_ptr<Node>> closedNodes;

		// If Node closest to your current location exists
		if (closestNode(m_pRobot->getPose()) != nullptr)
		{
			// Add Node at your current location to closed list
			closedNodes.push_back(closestNode(m_pRobot->getPose()));
		}
		// There is no Node 
		else
		{
			std::cerr << "\n Generating path... Error - No available node.\n";
			system("pause");
			return;
		}

		// Bool for whether a path was been found
		bool bPathFound = false;

		// While a path has not been found
		while (!bPathFound)
		{
			// Current Node being tested
			std::shared_ptr<Node> currentNode = closedNodes.back();

			///////////////////// Calculating Open Adjacent Nodes /////////////////////

			// Get all adjacent nodes
			std::vector<std::shared_ptr<Node>> adjNodes = getAdjacentNodes(currentNode);

			// For all adjacent nodes
			for (std::shared_ptr<Node> adjNode : adjNodes)
			{
				// If adjNode is inaccessible
				if (!adjNode->bAccessible) {}

				// Else If adjNode is the target destination
				else if (adjNode == targetNode)
				{
					// Sets adj parent to the closed node
					adjNode->parent = currentNode;

					// Create G and F value for the adj Node
					adjNode->g = calcG(currentNode, adjNode);
					// F = G+H
					adjNode->f = adjNode->g + adjNode->h;

					// Destination found, create path
					queuePath(adjNode);
					std::cerr << "\n Generating path... Finished.\n";
					bPathFound = true;
					return;
				}

				// Else If adjNode is on the open list already
				else if (nodeInVector(adjNode, openNodes))
				{
					// If (total movement cost to adjcent node through current Node) is less than (total movement cost to adjNode)
					if (currentNode->g + Utils::magnitude(sf::Vector2f(currentNode->position.getX() - adjNode->position.getX(), currentNode->position.getY() - adjNode->position.getY())) < adjNode->g)
					{
						// Sets adj parent to the closed node
						adjNode->parent = currentNode;
					}
				}

				// Else if adjNode is on the closed list
				else if (nodeInVector(adjNode, closedNodes)) {}

				// Otherwise
				else
				{
					// Parent the open node to current node
					adjNode->parent = currentNode;

					// Create G and F value for each open node
					adjNode->g = calcG(currentNode, adjNode);

					// F = G+H
					adjNode->f = adjNode->g + adjNode->h;

					// Add adjacent nodes to open list
					openNodes.push_back(adjNode);
				}
			}

			///////////////////// Determining Next Closed Node /////////////////////

			// If there is an open list
			if (!openNodes.empty())
			{
				// Variable to store current Node with smallest F value
				std::shared_ptr<Node> nodeWithSmallestF = openNodes.at(0); // Sets starting Node with smallest F value to the first Node in the vector

				// For all open nodes
				for (std::shared_ptr<Node> node : openNodes)
				{
					// If Node.F is less than currSmallest->f
					if (node->f < nodeWithSmallestF->f)
					{
						// nodeWithSmallestF replaced with this Node
						nodeWithSmallestF = node;
					}
					// Else If two Nodes have the same F value
					else if (node->f == nodeWithSmallestF->f)
					{
						// The Node with lowest H is set
						if (node->h < nodeWithSmallestF->h) nodeWithSmallestF = node;
					}
				}

				// Adds Node with smallest F value to the closed list
				closedNodes.push_back(nodeWithSmallestF);

				// Removes the Node with the smalled F value from the open list
				std::vector<std::shared_ptr<Node>> newOpenNodes;
				for (std::shared_ptr<Node> openNode : openNodes)
				{
					// If the openNode doesn't have the smallest F value
					if (openNode != nodeWithSmallestF)
					{
						// Pushes onto new open list
						newOpenNodes.push_back(openNode);
					}
				}
				// Replaces open list with new edited copy
				openNodes = newOpenNodes;
			}
			// Else the open list is empty
			else
			{
				std::cerr << "\n Generating path... Error - No path found.\n";
				system("pause");
				return;
			}
		}
	}

	// Else Nodes have not been intitialised
	else std::cerr << "\n Generating path... Error - Nodes not intitialised.\n"; system("pause");
}

// Returns the Node that is closest to the position
std::shared_ptr<Node> Pathfinding::closestNode(const ArPose kPosition)
{
	// In Nodes exist
	if (m_pNodes.size() > 0)
	{
		// Adds the first node as closest
		std::shared_ptr<Node> closestNode = m_pNodes.at(0);

		// For all Nodes
		for (std::shared_ptr<Node> node : m_pNodes)
		{
			// If distance from position to Node is smaller than position to closest Node
			if (Utils::magnitude(sf::Vector2f(node->position.getX() - kPosition.getX(), node->position.getY() - kPosition.getY())) < Utils::magnitude(sf::Vector2f(closestNode->position.getX() - kPosition.getX(), closestNode->position.getY() - kPosition.getY())))
			{
				// If the Node is accessible
				if (node->bAccessible)
				{
					// Replaces the closest Node
					closestNode = node;
				}
			}
		}

		// Returns closest Node
		return closestNode;
	}

	// Nodes don't exist: return nullptr
	return nullptr;
}

// Calculates G value of a Node
double Pathfinding::calcG(std::shared_ptr<Node> currentNode, std::shared_ptr<Node> targetNode)
{
	// Distance from the current Node and open Node
	double dDistToNode = Utils::magnitude(sf::Vector2f(targetNode->position.getX() - currentNode->position.getX(), targetNode->position.getY() - currentNode->position.getY()));
		
	// Returns resultant G value (distance to next Node plus current Node's G)
	return dDistToNode + currentNode->g;
}

// Returns whether a Node is within a distance of a Line
bool Pathfinding::nodeNearLine(const ArLineSegment kLine, const ArPose kNodePos, const double kDistance)
{
	// Defines nod upper and lower bounds in world coordinates
	sf::Vector2f nodeUpperBounds(kNodePos.getX() + m_dNodeDiameter*0.5, kNodePos.getY() + m_dNodeDiameter*0.5);
	sf::Vector2f nodeLowerBounds(kNodePos.getX() - m_dNodeDiameter*0.5, kNodePos.getY() - m_dNodeDiameter*0.5);

	// Creates an angular vector of the line
	sf::Vector2f angleVec = sf::Vector2f(kLine.getX2(), kLine.getY2()) - sf::Vector2f(kLine.getX1(), kLine.getY1());
	// Creates a unit vector of the angle
	sf::Vector2f angleUnitVec = angleVec / (float)Utils::magnitude(angleVec);

	// Creates a point at the start of the line
	sf::Vector2f point = sf::Vector2f(kLine.getX1(), kLine.getY1());

	// For length of line
	for (unsigned int i = 0; i < kLine.getLengthOf(); i++)
	{
		// Moves the point along the line
		point += angleUnitVec;

		// Displacement between point and Node
		sf::Vector2f displacement(kNodePos.getX() - point.x, kNodePos.getY() - point.y);

		// If the distance between point and Node is smaller than distance param
		if (Utils::magnitude(displacement) < kDistance)
		{
			// Return True: Node is near line
			return true; 
		}
	}

	// Return False: Node isn't near line
	return false;
}

// Returns whether a Node is within a vector of Nodes
bool Pathfinding::nodeInVector(std::shared_ptr<Node> nodeToFind, std::vector<std::shared_ptr<Node>> vector)
{
	// If a Node in the vector is the nodeToFind: Return True
	for (std::shared_ptr<Node> node : vector) if (node == nodeToFind) return true;

	// Return False: nodeToFind not found
	return false;
}

// Returns the distance from one Node to another
int Pathfinding::manhattanDist(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode)
{
	// Number of Nodes needed horizontally to match destination
	int xDist = (endNode->position.getX() - startNode->position.getX()) / m_dNodeDiameter;

	// Number of Nodes needed vertically to match destination
	int yDist = (endNode->position.getY() - startNode->position.getY()) / m_dNodeDiameter;

	// Combines the distances in the x and y axis
	int iDistance = abs(xDist) + abs(yDist);

	// Returns the combined distances, bound to positive
	return iDistance;
}

// Returns a vector of accessible adjacent Nodes
std::vector<std::shared_ptr<Node>> Pathfinding::getAdjacentNodes(std::shared_ptr<Node> node)
{
	// Defines vector of Nodes to store adjacent Nodes that are identified
	std::vector<std::shared_ptr<Node>> adjNodes;

	// If Node exists
	if (node != nullptr)
	{
		// Declares Node ptr to hold adjacent Nodes as they're processed
		std::shared_ptr<Node> adjNode;

		// For 8 cycles
		for (unsigned int i = 1; i <= 8; i++)
		{
			switch (i)
			{
				case 1:
				{
					// x o o
					// o n o
					// o o o
					adjNode = nodeFromPose(ArPose(node->position.getX() - m_dNodeDiameter, node->position.getY() + m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Nodes used to check diagonal accessibility
						// x o o
						// c n o
						// o o o
						std::shared_ptr<Node> checkNode1 = nodeFromPose(ArPose(node->position.getX() - m_dNodeDiameter, node->position.getY()));
						// x c o
						// o n o
						// o o o
						std::shared_ptr<Node> checkNode2 = nodeFromPose(ArPose(node->position.getX(), node->position.getY() + m_dNodeDiameter));

						// If checkNode1 Node exists and is not accessible
						if ((checkNode1 != nullptr) && !(checkNode1->bAccessible)) {}

						// Else If checkNode2 Node exists and is not accessible
						else if ((checkNode2 != nullptr) && !(checkNode2->bAccessible)) {}

						// Both checkNodes exist and are accessible
						else 
						{
							// Adds adjacent Node to vector
							adjNodes.push_back(adjNode);
						}
					}
				} break;

				case 2:
				{
					// o x o
					// o n o
					// o o o
					adjNode = nodeFromPose(ArPose(node->position.getX(), node->position.getY() + m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Adds adjacent Node to vector
						adjNodes.push_back(adjNode);
					}
				} break;

				case 3:
				{
					// o o x
					// o n o
					// o o o
					adjNode = nodeFromPose(ArPose(node->position.getX() + m_dNodeDiameter, node->position.getY() + m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Nodes used to check diagonal accessibility
						// o c x
						// o n o
						// o o o
						std::shared_ptr<Node> checkNode1 = nodeFromPose(ArPose(node->position.getX(), node->position.getY() + m_dNodeDiameter));
						// o o x
						// o n c
						// o o o
						std::shared_ptr<Node> checkNode2 = nodeFromPose(ArPose(node->position.getX() + m_dNodeDiameter, node->position.getY()));

						// If checkNode1 Node exists and is not accessible
						if ((checkNode1 != nullptr) && !(checkNode1->bAccessible)) {}

						// Else If checkNode2 Node exists and is not accessible
						else if ((checkNode2 != nullptr) && !(checkNode2->bAccessible)) {}

						// Both checkNodes exist and are accessible
						else
						{
							// Adds adjacent Node to vector
							adjNodes.push_back(adjNode);
						}
					}
				} break;

				case 4:
				{
					// o o o
					// o n x
					// o o o
					adjNode = nodeFromPose(ArPose(node->position.getX() + m_dNodeDiameter, node->position.getY()));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Adds adjacent Node to vector
						adjNodes.push_back(adjNode);
					}
				} break;

				case 5:
				{
					// o o o
					// o n o
					// o o x
					adjNode = nodeFromPose(ArPose(node->position.getX() + m_dNodeDiameter, node->position.getY() - m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Nodes used to check diagonal accessibility
						// o o o
						// o n c
						// o o x
						std::shared_ptr<Node> checkNode1 = nodeFromPose(ArPose(node->position.getX() + m_dNodeDiameter, node->position.getY()));
						// o o o
						// o n o
						// o c x
						std::shared_ptr<Node> checkNode2 = nodeFromPose(ArPose(node->position.getX(), node->position.getY() - m_dNodeDiameter));

						// If checkNode1 Node exists and is not accessible
						if ((checkNode1 != nullptr) && !(checkNode1->bAccessible)) {}

						// Else If checkNode2 Node exists and is not accessible
						else if ((checkNode2 != nullptr) && !(checkNode2->bAccessible)) {}

						// Both checkNodes exist and are accessible
						else
						{
							// Adds adjacent Node to vector
							adjNodes.push_back(adjNode);
						}
					}
				} break;

				case 6:
				{
					// o o o
					// o n o
					// o x o
					adjNode = nodeFromPose(ArPose(node->position.getX(), node->position.getY() - m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Adds adjacent Node to vector
						adjNodes.push_back(adjNode);
					}
				} break;

				case 7:
				{
					// o o o
					// o n o
					// x o o
					adjNode = nodeFromPose(ArPose(node->position.getX() - m_dNodeDiameter, node->position.getY() - m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Nodes used to check diagonal accessibility
						// o o o
						// o n o
						// x c o
						std::shared_ptr<Node> checkNode1 = nodeFromPose(ArPose(node->position.getX(), node->position.getY() - m_dNodeDiameter));
						// o o o
						// c n o
						// x o o
						std::shared_ptr<Node> checkNode2 = nodeFromPose(ArPose(node->position.getX() - m_dNodeDiameter, node->position.getY()));

						// If checkNode1 Node exists and is not accessible
						if ((checkNode1 != nullptr) && !(checkNode1->bAccessible)) {}

						// Else If checkNode2 Node exists and is not accessible
						else if ((checkNode2 != nullptr) && !(checkNode2->bAccessible)) {}

						// Both checkNodes exist and are accessible
						else
						{
							// Adds adjacent Node to vector
							adjNodes.push_back(adjNode);
						}
					}
				} break;

				case 8:
				{
					// o o o
					// x n o
					// o o o
					adjNode = nodeFromPose(ArPose(node->position.getX() - m_dNodeDiameter, node->position.getY()));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Adds adjacent Node to vector
						adjNodes.push_back(adjNode);
					}
				} break;
			}
		}
	}

	return adjNodes;
}

// Returns a Node at a given pose
std::shared_ptr<Node> Pathfinding::nodeFromPose(const ArPose kPosition)
{
	// For all Nodes
	for (std::shared_ptr<Node> node : m_pNodes)
	{
		// Defines nod upper and lower bounds in world coordinates
		ArPose nodeUpperBounds(node->position.getX() + m_dNodeDiameter*0.5, node->position.getY() + m_dNodeDiameter*0.5);
		ArPose nodeLowerBounds(node->position.getX() - m_dNodeDiameter*0.5, node->position.getY() - m_dNodeDiameter*0.5);

		// If position is within the Node's area
		if (Utils::pointInArea(kPosition, nodeUpperBounds, nodeLowerBounds))
		{
			// Position is within the Node's area: Return the Node
			return node;
		}
	}

	// No Node found: return nullptr
	return nullptr;
}

// Forms a queue of Nodes to the given Node
void Pathfinding::queuePath(std::shared_ptr<Node> targetNode)
{
	// Vector for stack of Nodes to be used in path
	std::vector<std::shared_ptr<Node>> pNodes;

	// Pushes destination Node onto vector
	pNodes.push_back(targetNode);

	// While Node has a parent
	while (targetNode->parent != nullptr)
	{
		// Make the Node it's Parent
		targetNode = targetNode->parent;
		// Push the parent onto the path of Nodes
		pNodes.push_back(targetNode);
	}

	// pNodes is not empty
	while(!pNodes.empty())
	{
		// Pushes the position of node onto path
		m_path.push(pNodes.back()->position);
		
		// Pops Node off Vector
		pNodes.pop_back();
	}
}

// Draws pathfinding representation to a display
void Pathfinding::draw(sf::RenderTarget& target)
{
	// If Nodes initialised
	if (m_bNodesInit)
	{
		// Arbitrary value of bezel for the display within the window
		double dMinBezel = 150;
		m_displayBezel = sf::Vector2f(dMinBezel, dMinBezel);

		// Coordinate offset to convert from Aria coord system to SFML coord system with added display bezel
		sf::Vector2f offset = sf::Vector2f(m_mapLowerBounds.getX() - m_displayBezel.x, m_mapUpperBounds.getY() + m_displayBezel.y);

		// Rectangle shape for drawing
		sf::RectangleShape rectShape;

		// BACKGROUND
		rectShape.setSize(sf::Vector2f(m_mapSize.x, m_mapSize.y)); // Size of Map
		rectShape.setFillColor(sf::Color(255.0f, 255.0f, 255.0f, 255.0f)); // White
		rectShape.setOrigin(sf::Vector2f(0.0f, rectShape.getSize().y));
		rectShape.setPosition(sf::Vector2f(m_mapLowerBounds.getX() - offset.x, Utils::invertDouble(m_mapLowerBounds.getY() - offset.y)));

		target.draw(rectShape);

		// NODES
		rectShape.setSize(sf::Vector2f(m_dNodeDiameter, m_dNodeDiameter)); // Size of Node
		rectShape.setOutlineColor(sf::Color(150.0f, 150.0f, 150.0f, 255.0f)); // Black Outline
		rectShape.setOutlineThickness(8); // Outline Thickness
		rectShape.setOrigin(rectShape.getSize()*0.5f); // Origin center

		// For every Node
		for (std::shared_ptr<Node> node : m_pNodes)
		{
			// If Node is nullptr
			if (node == nullptr) {}
			else
			{
				// If Node accessible
				if (node->bAccessible) rectShape.setFillColor(sf::Color(255.0f, 255.0f, 255.0f, 0.0f)); // Transparent
				// Else Node not accessible
				else rectShape.setFillColor(sf::Color(255.0f, 0.0f, 0.0f, 75.0f)); // Translucent Red

				rectShape.setPosition(sf::Vector2f(node->position.getX() - offset.x, Utils::invertDouble(node->position.getY() - offset.y))); // Moves rect to position

				target.draw(rectShape);
			}
		}

		// LINE SEGMENTS
		// For every Line segment in the map
		for (unsigned int i = 0; i < m_map.getLines()->size(); i++)
		{
			// Gets the line at index i
			ArLineSegment line = m_map.getLines()->at(i);

			rectShape.setSize(sf::Vector2f(line.getLengthOf(), 25.0f)); // Size of Node
			rectShape.setOutlineThickness(0.0f); // Outline Thickness
			rectShape.setFillColor(sf::Color(0.0f, 0.0f, 0.0f, 255.0f)); // Black
			rectShape.setOrigin(rectShape.getSize()*0.5f); // Origin center
			rectShape.setPosition(sf::Vector2f(line.getMidPoint().getX() - offset.x, Utils::invertDouble(line.getMidPoint().getY() - offset.y)));

			sf::Vector2f angleVec = sf::Vector2f(line.getEndPoint2().getX(), Utils::invertDouble(line.getEndPoint2().getY())) - sf::Vector2f(line.getEndPoint1().getX(), Utils::invertDouble(line.getEndPoint1().getY()));

			sf::Vector2f angleUnitVec = angleVec / (float)Utils::magnitude(angleVec);

			float lineAngle = Utils::angleFromUnitVec(angleUnitVec);

			rectShape.setRotation(lineAngle);

			target.draw(rectShape);
		}

		// PATH
		// Declares line and colour
		sf::Vertex line[2];
		sf::Color colour = sf::Color(255, 0, 0, 255);
		// Duplicates the path queue
		std::queue<ArPose> pathDupe = m_path;

		// If there is a queue
		if (!pathDupe.empty())
		{
			ArPose lastPoint = pathDupe.front();

			// Sets the first point of the line at the position in front of the queue
			line[0] = sf::Vertex(sf::Vector2f(lastPoint.getX() - offset.x, Utils::invertDouble(lastPoint.getY() - offset.y)), colour);

			// Sets the second point of the line at the position of the robot
			line[1] = sf::Vertex(sf::Vector2f(m_pRobot->getX() - offset.x, Utils::invertDouble(m_pRobot->getY() - offset.y)), colour);

			// Draws the line to target
			target.draw(line, 2, sf::Lines);

			// For every point in the path queue
			while (!pathDupe.empty())
			{
				// Sets the first point of the line to the position of the last position drawn
				line[0] = sf::Vertex(sf::Vector2f(lastPoint.getX() - offset.x, Utils::invertDouble(lastPoint.getY() - offset.y)), colour);

				// Sets the second point of the line at the position in front of the queue
				line[1] = sf::Vertex(sf::Vector2f(pathDupe.front().getX() - offset.x, Utils::invertDouble(pathDupe.front().getY() - offset.y)), colour);

				lastPoint = pathDupe.front();
				pathDupe.pop(); // Removes the point from the queue

				// Draws the line to target
				target.draw(line, 2, sf::Lines);

				// If it's on the last point in the cycle
				if (pathDupe.empty())
				{
					sf::CircleShape circleShape;

					circleShape.setRadius(75.f); // Size of Robot
					circleShape.setFillColor(sf::Color(255.0f, 0.0f, 0.0f, 255.0f)); // Red
					circleShape.setOrigin(sf::Vector2f(circleShape.getRadius(), circleShape.getRadius())); // Origin center
					circleShape.setPosition(sf::Vector2f(lastPoint.getX() - offset.x, Utils::invertDouble(lastPoint.getY() - offset.y)));

					target.draw(circleShape);
				}
			}
		}

		// MOVEMENT HISTORY
		// Duplicates the path queue
		std::vector<ArPose> historyDupe = m_moveHistory;
		
		rectShape.setSize(sf::Vector2f(m_pRobot->getRobotLength(), m_pRobot->getRobotWidth())); // Size of Robot
		rectShape.setFillColor(sf::Color(0.0f, 0.0f, 150.0f, 10.0f)); // Translucent Blue
		rectShape.setOrigin(rectShape.getSize()*0.5f); // Origin center

		// For every point in the path queue
		while (!historyDupe.empty())
		{
			// Moves the rectShape to the last historic Pose
			rectShape.setPosition(sf::Vector2f(historyDupe.back().getX() - offset.x, Utils::invertDouble(historyDupe.back().getY() - offset.y)));
			rectShape.setRotation(Utils::invertDouble(historyDupe.back().getTh()));

			historyDupe.pop_back(); // Removes the pose from the vector

			target.draw(rectShape);
		}

		// ROBOT
		rectShape.setFillColor(sf::Color(255.0f, 0.0f, 0.0f, 255.0f)); // Red
		rectShape.setPosition( sf::Vector2f( m_pRobot->getX() - offset.x, Utils::invertDouble(m_pRobot->getY() - offset.y) ) );
		rectShape.setRotation(Utils::invertDouble(m_pRobot->getTh()));

		target.draw(rectShape);
	}
}