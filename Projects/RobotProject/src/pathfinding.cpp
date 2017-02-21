
#include "pathfinding.h"

// Constructor
Pathfinding::Pathfinding(const ArMap kMap, ArRobot* kRobot, const ArPose kStartPose, const ArPose kGoalPose)
{
	m_map = kMap;

	m_pRobot = kRobot;

	m_startPose = kStartPose;
	m_goalPose = kGoalPose;

	m_dNodeDiameter = 550; // 0.55m // Robot Diameter 0.5m
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
		m_mapUpperBounds.x = Utils::maxDouble(m_mapUpperBounds.x, line.getX1(), line.getX2());
		m_mapUpperBounds.y = Utils::maxDouble(m_mapUpperBounds.y, line.getY1(), line.getY2());

		// If the x/y is smaller than the current then replace it
		m_mapLowerBounds.x = Utils::minDouble(m_mapLowerBounds.x, line.getX1(), line.getX2());
		m_mapLowerBounds.y = Utils::minDouble(m_mapLowerBounds.y, line.getY1(), line.getY2());
	}
	
	// Creates a vertex of the world size
	m_mapSize = Vertex(m_mapUpperBounds.x - m_mapLowerBounds.x, m_mapUpperBounds.y - m_mapLowerBounds.y);
	
	// Divides the map size by the diameter of the robot and rounds down to nearest whole number
	Vertex dividedMapSize(floor(m_mapSize.x / m_dNodeDiameter), floor(m_mapSize.y / m_dNodeDiameter));

	// For the number of nodes needed for the width
	for (unsigned int x = 1; x <= dividedMapSize.x; x++)
	{
		// For the number of nodes needed for the height
		for (unsigned int y = 1; y <= dividedMapSize.y; y++)
		{
			// Declares new Node
			std::shared_ptr<Node> newNode = std::shared_ptr<Node>(new Node());
			
			// Sets the new Node's world position
			newNode->position.x = m_mapLowerBounds.x + (m_dNodeDiameter * x) - m_dNodeDiameter*0.5;
			newNode->position.y = m_mapLowerBounds.y + (m_dNodeDiameter * y) - m_dNodeDiameter*0.5;
			
			// Sets new Node's index member
			newNode->index = m_pNodes.size();

			// Pushes the new Node onto vector member of Nodes
			m_pNodes.push_back(newNode);
		}
	}

	m_bNodesInit = true;

	std::cerr << "\n Generating nodes for A* pathfinding... Finished.\n";

	calcAccessibility();
}

// Calculates the accessibility of the Nodes
void Pathfinding::calcAccessibility()
{
	std::cerr << "\n Calculating node accessibility... \n";

	// For all Nodes
	for (std::shared_ptr<Node> node : m_pNodes)
	{
		//std::cerr
		//	<< "\n node->index: " << node->index
		//	<< '\n';
	
		// For every line segment in the map
		for (unsigned int i = 0; i < m_map.getLines()->size(); i++)
		{
			//std::cerr
			//	<< "\n line i: " << i
			//	<< '\n';
	
			// Gets the line at index i
			ArLineSegment line = m_map.getLines()->at(i);
	
			// If Line intersects Node
			if (lineIntersectNode(line, node->position))
			{
				node->bAccessible = false;
			}
		}
	}

	std::cerr << "\n Calculating node accessibility... Finished.\n";
}

// Creates a path to a target Node
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
			node->h = dist_NodeToNode(node, targetNode);
			//node->h = Utils::magnitude(Vertex(targetNode->position.x - node->position.x, targetNode->position.y - node->position.y));
		}

		///////////////////// Creating Open and Closed Lists /////////////////////

		// Explanation: https://www.youtube.com/watch?v=KNXfSOx4eEE
		// Declares vectors to store nodes to check and checked nodes
		std::vector<std::shared_ptr<Node>> openNodes;
		std::vector<std::shared_ptr<Node>> closedNodes;

		// If Node at your current location exists
		if (closestNode(Vertex(absolutePose(m_pRobot->getPose()).getX(), absolutePose(m_pRobot->getPose()).getY())) != nullptr)
		{
			// Add Node at your current location to closed list
			closedNodes.push_back(closestNode(Vertex(absolutePose(m_pRobot->getPose()).getX(), absolutePose(m_pRobot->getPose()).getY())));
		}
		else
		{
			std::cerr << "\n Generating path... Error - No available node.\n";
			system("pause");
			return;
		}

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

				// Else If adjNode is on the open list already
				else if (nodeInVector(adjNode, openNodes))
				{
					// If (total movement cost to adjcent node through current Node) is less than (total movement cost to adjNode)
					if (currentNode->g + Utils::magnitude(Vertex(currentNode->position.x - adjNode->position.x, currentNode->position.y - adjNode->position.y)) < adjNode->g)
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

					// Add adjacent nodes to open list
					openNodes.push_back(adjNode);
				}
			}

			///////////////////// Calculating Movement Cost And F Values /////////////////////

			// For all open Nodes
			for (std::shared_ptr<Node> openNode : openNodes)
			{
				// If open Node is not the target node
				if (openNode != targetNode)
				{
					// Create G and F value for each open node
					openNode->g = calcG(currentNode, openNode);
					// F = G+H
					openNode->f = openNode->g + openNode->h;
				}
				else
				{
					// Destination found, create path
					bPathFound = true;
					queuePath(openNode);
					std::cerr << "\n Generating path... Finished.\n";
					return;
				}
			}

			///////////////////// Determining Next Closed Node /////////////////////

			std::shared_ptr<Node> node_smallestF;

			// If there is an open list
			if (!openNodes.empty())
			{
				node_smallestF = openNodes.at(0);

				// For all open nodes
				for (std::shared_ptr<Node> node : openNodes)
				{
					// Smallest F value node made the current with node moved to closed
					if (node->f < node_smallestF->f)
					{
						node_smallestF = node;
					}
					// If two Nodes have the same F value then the Node with lowest H is set
					else if (node->h < node_smallestF->h)
					{
						node_smallestF = node;
					}
				}

				// Removes the Node with the smalled F value from the open list
				std::vector<std::shared_ptr<Node>> newOpenNodes;
				for (std::shared_ptr<Node> openNode : openNodes)
				{
					if (openNode != node_smallestF)
					{
						newOpenNodes.push_back(openNode);
					}
				}
				openNodes = newOpenNodes;

				if (!nodeInVector(node_smallestF, closedNodes))
				{
					// Adds said Node to the closed list
					closedNodes.push_back(node_smallestF);
				}
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

	else std::cerr << "\n Generating path... Error - Nodes not intitialised.\n";
}

double Pathfinding::calcG(std::shared_ptr<Node> currentNode, std::shared_ptr<Node> targetNode)
{
	// Distance from the current Node and open Node
	double dDistToNode = Utils::magnitude(Vertex(targetNode->position.x - currentNode->position.x, targetNode->position.y - currentNode->position.y));

	//sf::Vector2f angleVec = sf::Vector2f(targetNode->position.x, targetNode->position.y) - sf::Vector2f(currentNode->position.x, currentNode->position.y);
	//sf::Vector2f angleUnitVec = angleVec / (float)Utils::magnitude(Vertex(angleVec.x, angleVec.y));
	//
	//double currentToTargetAngle = Utils::angleFromUnitVec(Vertex(angleUnitVec.x, angleUnitVec.y));
	//
	//double dTurnCost = Utils::angleFromUnitVec();
	//dTurnCost = 

	return (dDistToNode * (1 / m_dNodeDiameter)) /*+ dTurnCost*/ + targetNode->parent->g;
}

bool Pathfinding::lineIntersectNode(const ArLineSegment kLine, const Vertex kNodePos)
{
	// Guide - https://gist.github.com/ChickenProp/3194723
	// Defines nod upper and lower bounds in world coordinates
	sf::Vector2f nodeUpperBounds(kNodePos.x + m_dNodeDiameter*0.5, kNodePos.y + m_dNodeDiameter*0.5);
	sf::Vector2f nodeLowerBounds(kNodePos.x - m_dNodeDiameter*0.5, kNodePos.y - m_dNodeDiameter*0.5);

	// Creates an angular vector of the line
	sf::Vector2f angleVec = sf::Vector2f(kLine.getX2(), kLine.getY2()) - sf::Vector2f(kLine.getX1(), kLine.getY1());
	// Creates a unit vector of the angle
	sf::Vector2f angleUnitVec = angleVec / (float)Utils::magnitude(Vertex(angleVec.x, angleVec.y));

	// Creates a point at the start of the line
	sf::Vector2f point = sf::Vector2f(kLine.getX1(), kLine.getY1());

	// For length of line
	for (unsigned int i = 0; i < kLine.getLengthOf(); i++)
	{
		// Moves the point along the line
		point += angleUnitVec;

		if (Utils::pointInArea(point, nodeUpperBounds, nodeLowerBounds))
		{
			return true;
		}
	}
		
	return false;
}

bool Pathfinding::nodeInVector(std::shared_ptr<Node> nodeToFind, std::vector<std::shared_ptr<Node>> vector)
{
	for (std::shared_ptr<Node> node : vector)
	{
		if (node == nodeToFind) return true;
	}

	return false;
}

int Pathfinding::dist_NodeToNode(std::shared_ptr<Node> startNode, std::shared_ptr<Node> endNode)
{
	// Number of Nodes needed horizontally to match destination
	int xDist = (endNode->position.x - startNode->position.x) / m_dNodeDiameter;

	// Number of Nodes needed vertically to match destination
	int yDist = (endNode->position.y - startNode->position.y) / m_dNodeDiameter;

	// Combines the distances in the x and y axis
	int iDistance = xDist + yDist;

	// Returns the combined distances, bound to positive
	return abs(iDistance);
}

std::vector<std::shared_ptr<Node>> Pathfinding::getAdjacentNodes(std::shared_ptr<Node> node)
{
	// Defines vector of Nodes to store adjacent Nodes that are identified
	std::vector<std::shared_ptr<Node>> adjNodes;

	// If Node exists
	if (node != nullptr)
	{
		std::shared_ptr<Node> adjNode;

		for (unsigned int i = 1; i <= 8; i++)
		{
			switch (i)
			{
				case 1:
				{
					// x o o
					// o n o
					// o o o
					adjNode = nodeFromPos(Vertex(node->position.x - m_dNodeDiameter, node->position.y + m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Nodes used to check diagonal accessibility
						// x o o
						// c n o
						// o o o
						std::shared_ptr<Node> checkNode1 = nodeFromPos(Vertex(node->position.x - m_dNodeDiameter, node->position.y));
						// x c o
						// o n o
						// o o o
						std::shared_ptr<Node> checkNode2 = nodeFromPos(Vertex(node->position.x, node->position.y + m_dNodeDiameter));

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
					adjNode = nodeFromPos(Vertex(node->position.x, node->position.y + m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						adjNodes.push_back(adjNode);
					}
				} break;

				case 3:
				{
					// o o x
					// o n o
					// o o o
					adjNode = nodeFromPos(Vertex(node->position.x + m_dNodeDiameter, node->position.y + m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Nodes used to check diagonal accessibility
						// o c x
						// o n o
						// o o o
						std::shared_ptr<Node> checkNode1 = nodeFromPos(Vertex(node->position.x, node->position.y + m_dNodeDiameter));
						// o o x
						// o n c
						// o o o
						std::shared_ptr<Node> checkNode2 = nodeFromPos(Vertex(node->position.x + m_dNodeDiameter, node->position.y));

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
					adjNode = nodeFromPos(Vertex(node->position.x + m_dNodeDiameter, node->position.y));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						adjNodes.push_back(adjNode);
					}
				} break;

				case 5:
				{
					// o o o
					// o n o
					// o o x
					adjNode = nodeFromPos(Vertex(node->position.x + m_dNodeDiameter, node->position.y - m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Nodes used to check diagonal accessibility
						// o o o
						// o n c
						// o o x
						std::shared_ptr<Node> checkNode1 = nodeFromPos(Vertex(node->position.x + m_dNodeDiameter, node->position.y));
						// o o o
						// o n o
						// o c x
						std::shared_ptr<Node> checkNode2 = nodeFromPos(Vertex(node->position.x, node->position.y - m_dNodeDiameter));

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
					adjNode = nodeFromPos(Vertex(node->position.x, node->position.y - m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						adjNodes.push_back(adjNode);
					}
				} break;

				case 7:
				{
					// o o o
					// o n o
					// x o o
					adjNode = nodeFromPos(Vertex(node->position.x - m_dNodeDiameter, node->position.y - m_dNodeDiameter));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						// Nodes used to check diagonal accessibility
						// o o o
						// o n o
						// x c o
						std::shared_ptr<Node> checkNode1 = nodeFromPos(Vertex(node->position.x, node->position.y - m_dNodeDiameter));
						// o o o
						// c n o
						// x o o
						std::shared_ptr<Node> checkNode2 = nodeFromPos(Vertex(node->position.x - m_dNodeDiameter, node->position.y));

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
					adjNode = nodeFromPos(Vertex(node->position.x - m_dNodeDiameter, node->position.y));

					// If Node at position exists
					if (adjNode != nullptr)
					{
						adjNodes.push_back(adjNode);
					}
				} break;
			}
		}
	}

	return adjNodes;
}

std::shared_ptr<Node> Pathfinding::nodeFromPos(const Vertex kPosition)
{
	for (std::shared_ptr<Node> node : m_pNodes)
	{
		// Defines nod upper and lower bounds in world coordinates
		Vertex nodeUpperBounds(node->position.x + m_dNodeDiameter*0.5, node->position.y + m_dNodeDiameter*0.5);
		Vertex nodeLowerBounds(node->position.x - m_dNodeDiameter*0.5, node->position.y - m_dNodeDiameter*0.5);

		// If position is within the node area
		if (Utils::pointInArea(kPosition, nodeUpperBounds, nodeLowerBounds))
		{
			return node;
		}
	}

	return nullptr;
}

std::shared_ptr<Node> Pathfinding::closestNode(const Vertex kPosition)
{
	// In Nodes exist
	if (m_pNodes.size() > 0)
	{
		// Adds the first node as closest
		std::shared_ptr<Node> closestNode = m_pNodes.at(0);

		for (std::shared_ptr<Node> node : m_pNodes)
		{
			if (Utils::magnitude(Vertex(node->position.x - kPosition.x, node->position.y - kPosition.y)) < Utils::magnitude(Vertex(closestNode->position.x - kPosition.x, closestNode->position.y - kPosition.y)))
			{
				if (node->bAccessible)
				{
					closestNode = node;
				}
			}
		}

		return closestNode;
	}

	return nullptr;
}

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

void Pathfinding::draw(sf::RenderTarget& target, sf::Font& font)
{
	// If Nodes initialised
	if (m_bNodesInit)
	{
		// Coordinate offset to convert from Aria coord system to SFML coord system with added display bezel
		Vertex offset(m_mapLowerBounds.x - m_displayBezel, m_mapUpperBounds.y + m_displayBezel);

		// Rectangle shape for drawing
		sf::RectangleShape rectShape;

		// BACKGROUND
		rectShape.setSize(sf::Vector2f(m_mapSize.x, m_mapSize.y)); // Size of Map
		rectShape.setFillColor(sf::Color(255.0f, 255.0f, 255.0f, 255.0f)); // White
		rectShape.setPosition(sf::Vector2f(m_displayBezel, m_displayBezel));

		target.draw(rectShape);

		// NODES
		rectShape.setSize(sf::Vector2f(m_dNodeDiameter, m_dNodeDiameter)); // Size of Node
		rectShape.setOutlineColor(sf::Color(150.0f, 150.0f, 150.0f, 255.0f)); // Black Outline
		rectShape.setOutlineThickness(12.5f); // Outline Thickness
		rectShape.setOrigin(rectShape.getSize()*0.5f); // Origin center

		// Declares a character size equal to 10% of the Node diameter
		unsigned int uiCharSize = (unsigned int)(m_dNodeDiameter * 0.1f);

		// Declares Text object
		sf::Text text;
		// Sets the Text Font
		//text.setFont(font);
		// Applies the character size to the text
		text.setCharacterSize(uiCharSize);
		// Sets text styling
		text.setColor(sf::Color::Black); 
		text.setStyle(sf::Text::Bold);

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

				rectShape.setPosition(sf::Vector2f(node->position.x - offset.x, Utils::invertDouble(node->position.y) + offset.y)); // Moves rect to position

				target.draw(rectShape);

				std::string sString;
				//sString += "index:(";
				//sString += std::to_string(node->index);
				//sString += ") g:(";
				//sString += std::to_string(node->g);
				//sString += ") h:(";
				//sString += std::to_string(node->h);
				//sString += ") f:(";
				//sString += std::to_string(node->f);
				//sString += ")";

				// Sets text string to Node index
				//text.setString(sString);
				// Sets position
				text.setPosition(sf::Vector2f(node->position.x, node->position.y - uiCharSize*0.5));

				target.draw(text);
			}
		}

		// LINE SEGMENTS
		// For every Line segment in the map
		for (unsigned int i = 0; i < m_map.getLines()->size(); i++)
		{
			// Gets the line at index i
			ArLineSegment line = m_map.getLines()->at(i);

			rectShape.setSize(sf::Vector2f(line.getLengthOf(), 50.0f)); // Size of Node
			rectShape.setOutlineThickness(0.0f); // Outline Thickness
			rectShape.setFillColor(sf::Color(0.0f, 0.0f, 0.0f, 255.0f)); // Black
			rectShape.setOrigin(rectShape.getSize()*0.5f); // Origin center
			rectShape.setPosition(sf::Vector2f(line.getMidPoint().getX() - offset.x, Utils::invertDouble(line.getMidPoint().getY()) + offset.y));

			sf::Vector2f angleVec = sf::Vector2f(line.getEndPoint2().getX(), Utils::invertDouble(line.getEndPoint2().getY())) - sf::Vector2f(line.getEndPoint1().getX(), Utils::invertDouble(line.getEndPoint1().getY()));

			sf::Vector2f angleUnitVec = angleVec / (float)Utils::magnitude(Vertex(angleVec.x, angleVec.y));

			float lineAngle = Utils::angleFromUnitVec(Vertex(angleUnitVec.x, angleUnitVec.y));

			rectShape.setRotation(lineAngle);

			target.draw(rectShape);
		}

		// PATH
		// Declares line and colour
		sf::Vertex line[2];
		sf::Color colour = sf::Color(255, 0, 0, 255);
		// Duplicates the path queue
		std::queue<Vertex> pathDupe = m_path;

		// If there is a queue
		if (pathDupe.size() > 0)
		{
			Vertex lastPoint = m_path.front();

			// For every point in the path queue
			while (!pathDupe.empty())
			{
				// Sets the first point of the line to the position of the last position drawn
				line[0] = sf::Vertex(sf::Vector2f(lastPoint.x - offset.x, Utils::invertDouble(lastPoint.y) + offset.y), colour);

				// Sets the second point of the line at the position in front of the queue
				line[1] = sf::Vertex(sf::Vector2f(pathDupe.front().x - offset.x, Utils::invertDouble(pathDupe.front().y) + offset.y), colour);

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
					circleShape.setPosition(sf::Vector2f(lastPoint.x - offset.x, Utils::invertDouble(lastPoint.y) + offset.y));

					target.draw(circleShape);
				}
			}
		}

		// ROBOT
		sf::CircleShape circleShape;

		circleShape.setRadius(m_pRobot->getRobotRadius()); // Size of Robot
		circleShape.setFillColor(sf::Color(255.0f, 0.0f, 0.0f, 255.0f)); // Red
		circleShape.setOrigin(sf::Vector2f(circleShape.getRadius(), circleShape.getRadius())); // Origin center
		circleShape.setPosition(sf::Vector2f(absolutePose(m_pRobot->getPose()).getX() - offset.x, Utils::invertDouble(absolutePose(m_pRobot->getPose()).getY()) + offset.y));

		target.draw(circleShape);
	}
}