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

void read_MapInfo(const std::string ksFilePath, ArPose* startPose, ArPose* goalPose)
{
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

	// Closes Environment file
	file.close();
}

static void callback_mapchange()
{
	//
}

int main(int argc, char **argv)
{
	// SFML Setup
	// Instantiates window
	sf::RenderWindow window(sf::VideoMode(1280, 720), "Pathfinding Representation - Coursework Submission 2 (Part2) - Mobile Robotics - P14141609", sf::Style::Default);

	sf::FileInputStream fStream;
	if (!fStream.open("res/arialbd.ttf")) { std::cerr << "\n Error - Failed to open FileInputStream\n"; } // Error

	// Declares font
	sf::Font font;
	// Loads font from file
	if (!font.loadFromStream(fStream)) { std::cerr << "\n Error - Failed to open FontFromStream\n"; } // Error

	// Aria Setup
	Aria::init();
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobot robot;
	ArRobotConnector robotConnector(&argParser, &robot);
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

	// Aria Map Setup
	// Map directory
	std::string ksMapFile("E:\\Documents\\University\\Year 3\\IMAT3404 - Mobile Robotics\\Maps\\Mine.map");
	//std::string ksMapFile("E:\\Documents\\University\\Year 3\\IMAT3404 - Mobile Robotics\\Maps\\Circle.map");

	// Creates an ArMap object with the given map file
	ArMap map; map.readFile(ksMapFile.c_str());

	// Declares ArPose vairables for robot start and goal
	ArPose robotStartPose;
	ArPose robotGoalPose;

	// Reads the map file and sets the start and goal positions
	read_MapInfo(ksMapFile, &robotStartPose, &robotGoalPose);
	
	// Always try to connect to the first laser:
	argParser.addDefaultArgument("-connectLaser");

	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (argParser.checkHelpAndWarnUnparsed())
		{
			// -help not given, just exit.
			Aria::logOptions();
			Aria::exit(1);
		}
	}
		
	// Trigger argument parsing
	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
	}

	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);

	puts("Press Escape to exit.");

	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	robot.runAsync(true);

	// try to connect to laser. if fail, warn but continue, using sonar only
	if (!laserConnector.connectLasers())
	{
		ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only.");
	}

	// turn on the motors
	robot.enableMotors();

	// USED FOR WANDER
	// Intitialises a seed for rand()
	//srand((unsigned int)time(NULL));

	// add a set of actions that combine together to effect the wander behavior
	ArActionStallRecover recover;
	ArActionBumpers bumpers;

	// Actions
	Avoid avoid;
	Path path(map, robotStartPose, robotGoalPose);

	robot.addAction(&recover, 100);
	robot.addAction(&bumpers, 75);
	robot.addAction(&avoid, 50);
	robot.addAction(&path, 25);
	
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
				// Closes window
				window.close();
				Aria::exit(0);
			}
		}

		// Clears window making it entirely black
		window.clear(sf::Color(160, 160, 160, 255));

		// Draws pathfinding info to display
		path.getPathfinding()->draw(window, font);

		// Sets view to size of the map
		window.setView(sf::View(sf::FloatRect(0.0f, 0.0f, path.getPathfinding()->getMapSize().x + path.getPathfinding()->getDisplayBezel()*2, path.getPathfinding()->getMapSize().y + path.getPathfinding()->getDisplayBezel()*2)));

		// Displays the current frame
		window.display();
	}

	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();

	Aria::exit(0);
}