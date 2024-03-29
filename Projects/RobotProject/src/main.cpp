#include "Aria.h"

// SFML
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

// Robot Action Classes
#include "avoid.h"
#include "path.h"

bool read_MapInfo(const std::string ksFilePath, ArPose* startPose, ArPose* goalPose)
{
	std::cerr << "\n Reading Map Info... \n";

	// Declares a file stream with the input file path
	std::fstream file(ksFilePath, std::ios_base::in);

	// Declares a string to store file lines
	std::string sLine;

	// While new lines can be gotten
	while (std::getline(file, sLine)) // Gets next line
	{
		// Creates a string stream with the line
		std::istringstream iss(sLine);

		// Declares a string to store the first word of a line
		std::string sString = "";
		// Assigns the first word of the line to sPrefix
		iss >> sString;

		// If prefix is:
		if (sString == "Cairn:")
		{
			iss >> sString;

			// While words can be gotten
			if (sString == "RobotHome")
			{
				// Puts robotHome x into string
				iss >> sString;

				// Sets home x to value from string
				startPose->setX
				(
					std::stod(sString.substr(0, sString.size()))
				);

				// Puts robotHome y into string
				iss >> sString;

				// Sets home y to value from string
				startPose->setY
				(
					std::stod(sString.substr(0, sString.size()))
				);

				// Puts robotHome heading into string
				iss >> sString;

				// Sets home th to value from string
				startPose->setTh
				(
					std::stod(sString.substr(0, sString.size()))
				);
			}

			// While words can be gotten
			else if (sString == "Goal")
			{
				// Puts robotHome x into string
				iss >> sString;

				// Sets home x to value from string
				goalPose->setX
				(
					std::stod(sString.substr(0, sString.size()))
				);

				// Puts robotHome y into string
				iss >> sString;

				// Sets home y to value from string
				goalPose->setY
				(
					std::stod(sString.substr(0, sString.size()))
				);

				// Puts robotHome heading into string
				iss >> sString;

				// Sets home th to value from string
				goalPose->setTh
				(
					std::stod(sString.substr(0, sString.size()))
				);
			}
		}

		// Else the prefix isn't recognised
		else {}
	}

	// Boolean to store whether an error was encountered
	bool bErrorPresent = false;

	// If StartPose has not been given a position
	if (startPose->getX() == INT32_MIN || startPose->getY() == INT32_MIN || startPose->getTh() == INT32_MIN)
	{
		bErrorPresent = true;

		if (startPose->getX() == INT32_MIN) std::cerr << "\n Reading Map Info... Error - Missing 'Cairn: RobotHome' X argument.\n";
		if (startPose->getY() == INT32_MIN) std::cerr << "\n Reading Map Info... Error - Missing 'Cairn: RobotHome' Y argument.\n";
		if (startPose->getTh() == INT32_MIN) std::cerr << "\n Reading Map Info... Error - Missing 'Cairn: RobotHome' Th argument.\n";
	}

	// If GoalPose has not been given a value
	if (goalPose->getX() == INT32_MIN || goalPose->getY() == INT32_MIN || goalPose->getTh() == INT32_MIN)
	{
		bErrorPresent = true;

		if (goalPose->getX() == INT32_MIN) std::cerr << "\n Reading Map Info... Error - Missing 'Cairn: Goal' X argument.\n";
		if (goalPose->getY() == INT32_MIN) std::cerr << "\n Reading Map Info... Error - Missing 'Cairn: Goal' Y argument.\n";
		if (goalPose->getTh() == INT32_MIN) std::cerr << "\n Reading Map Info... Error - Missing 'Cairn: Goal' Th argument.\n";
	}

	// Closes Environment file
	file.close();

	// Error reading map info
	if (bErrorPresent) return false; 

	// Successfully read map info
	std::cerr << "\n Reading Map Info... Finished. \n";

	return true;
}

int main(int argc, char **argv)
{
	// Aria Setup
	Aria::init();
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	// String for map choice
	std::string sInput;

	std::cerr << "\n Enter a Map name to load: './res/maps/YOURINPUT.map': ";
	std::cin >> sInput; 
	std::cerr << '\n';

	// Aria Map Setup
	// Map directory
	std::string sMapDir("./res/maps/" + sInput + ".map");

	// Creates an ArMap object
	ArMap map; 
	
	// Attempts to read map object data into object
	if (!map.readFile(sMapDir.c_str()))
	{
		std::cerr << "\n Error - Map '" << sMapDir.c_str() << "' could not be read.\n";
		std::cerr << '\n'; system("pause");
	}
	// Proceed with program
	else 
	{
		// Declares ArPose vairables for robot start and goal
		ArPose startPose(INT32_MIN, INT32_MIN, INT32_MIN);
		ArPose goalPose(INT32_MIN, INT32_MIN, INT32_MIN);

		// Attempts to read the map file and set the start and goal positions
		if (!read_MapInfo(sMapDir, &startPose, &goalPose))
		{
			std::cerr << '\n'; system("pause");
		}
		// Proceed with program
		else
		{
			// SFML Setup
			// Instantiates window
			sf::RenderWindow window(sf::VideoMode(1280, 720), "Pathfinding Representation - Coursework-2 (Submission 2) - Mobile Robotics - P14141609", sf::Style::Default);

			// Robot Setup
			ArRobot robot;
			ArRobotConnector robotConnector(&argParser, &robot);
			ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);
			// Always try to connect to the first laser:
			argParser.addDefaultArgument("-connectLaser");
			if (!robotConnector.connectRobot())
			{
				ArLog::log(ArLog::Terse, "Could not connect to the robot.");
				if (argParser.checkHelpAndWarnUnparsed()) { Aria::logOptions(); Aria::exit(1); }
			}
			// Trigger argument parsing
			if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed()) { Aria::logOptions(); Aria::exit(1); }
			ArKeyHandler keyHandler;
			Aria::setKeyHandler(&keyHandler);
			robot.attachKeyHandler(&keyHandler);
			puts("Press Escape to exit.");
			ArSonarDevice sonar;
			robot.addRangeDevice(&sonar);
			robot.runAsync(true);
			// try to connect to laser. if fail, warn but continue, using sonar only
			if (!laserConnector.connectLasers()) { ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only."); }
			// Turn on the motors
			robot.enableMotors();

			// Robot Behaviour Setup
			// Add a set of actions that combine together to effect the wander behavior
			ArActionStallRecover recover;
			ArActionBumpers bumpers;
			
			// Defines Actions
			Avoid avoid;
			Path path(map, startPose, goalPose);

			// Adds Actions to robot
			robot.addAction(&recover, 100);
			robot.addAction(&bumpers, 75);
			robot.addAction(&avoid, 50);
			robot.addAction(&path, 25);

			// Resets the robot to the map robot home pose
			ArRobotPacket pkt;
			pkt.setID(ArCommands::SIM_SET_POSE);
			pkt.uByteToBuf(0);
			pkt.byte4ToBuf(startPose.getX());
			pkt.byte4ToBuf(startPose.getY());
			pkt.byte4ToBuf(startPose.getTh());
			pkt.finalizePacket();
			robot.getDeviceConnection()->write(pkt.getBuf(), pkt.getLength());
			
			// Sets the pose of the robot to the start pose so odometry starts from that pose instead of 0,0,0
			robot.moveTo(startPose, false);

			// While the window is open
			while (window.isOpen())
			{
				// Event object for windows event calls
				sf::Event event;
				while (window.pollEvent(event))
				{
					// If Closed event is called
					if (event.type == sf::Event::Closed)
					{
						// Closes window and terminates Aria
						window.close();
						Aria::exit(0);
					}
				}

				// Clears window making it entirely black
				window.clear(sf::Color(160, 160, 160, 255));

				// Draws pathfinding info to display
				path.getPathfinding()->draw(window);

				// Sets view to size of the map
				window.setView(sf::View(sf::FloatRect(0.0f, 0.0f, path.getPathfinding()->getViewSize().x, path.getPathfinding()->getViewSize().y)));

				// Displays the current frame
				window.display();
			}

			// wait for robot task loop to end before exiting the program
			robot.waitForRunExit();
		}
	}

	// Terminates Aria
	Aria::exit(0);
}