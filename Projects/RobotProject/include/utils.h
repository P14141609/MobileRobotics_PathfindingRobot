#ifndef UTILS_H
#define UTILS_H

// SFML
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
//
#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include <math.h>

// Constant values for calculations
const float g_kfPi = 3.14159f;
const float g_kfDegToRad = g_kfPi / 180;

struct Vertex
{
	// Default Cnstructor
	Vertex() {}

	// Constructor
	Vertex(const double kX, const double kY) { x = kX; y = kY; }

	// Members
	double x; // x coordinate
	double y; // y coordinate
};

class Utils
{
	public:
		
		// Returns the string value of a state
		static std::string stateToString(const int kiState)
		{
			if (kiState == 0) return "IDLE";
			if (kiState == 1) return "ACTIVE";
			else return "ERROR";
		}

		// Return the smallest double provided
		static double minDouble(const double kdA, const double kdB)
		{
			if (kdA <= kdB) return kdA;
			else return kdB;
		}
		// For 3 inputs
		static double minDouble(const double kdA, const double kdB, const double kdC) { return minDouble(minDouble(kdA, kdB), kdC); }

		// Return the largest double provided
		static double maxDouble(const double kdA, const double kdB)
		{
			if (kdA >= kdB) return kdA;
			else return kdB;
		}
		// For 3 inputs
		static double maxDouble(const double kdA, const double kdB, const double kdC) { return maxDouble(maxDouble(kdA, kdB), kdC); }

		// Return the smallest integer provided
		static int minInt(const int kiA, const int kiB)
		{
			if (kiA <= kiB) return kiA;
			else return kiB;
		}
		// For 3 inputs
		static int minInt(const int kiA, const int kiB, const int kiC) { return minInt(minInt(kiA, kiB), kiC); }

		// Return the largest integer provided
		static int maxInt(const int kiA, const int kiB)
		{
			if (kiA >= kiB) return kiA;
			else return kiB;
		}
		// For 3 inputs
		static int maxInt(const int kiA, const int kiB, const int kiC) { return maxInt(maxInt(kiA, kiB), kiC); }

		// Return is point is within area
		static bool pointInArea(const Vertex kPoint, const Vertex kAreaUpperBound, const Vertex kAreaLowerBound)
		{
			// If point.x is less than lower bound: Out of bounds
			if (kPoint.x <= kAreaLowerBound.x) return false;

			// If point.y is less than lower bound: Out of bounds
			if (kPoint.y <= kAreaLowerBound.y) return false;

			// If point.x is greater than upper bound: Out of bounds
			if (kPoint.x >= kAreaUpperBound.x) return false;

			// If point.y is greater than upper bound: Out of bounds
			if (kPoint.y >= kAreaUpperBound.y) return false;

			// No False returns called: call return True
			return true;
		}
		// For sf::Vector2f input
		static bool pointInArea(const sf::Vector2f kPoint, const sf::Vector2f kAreaUpperBound, const sf::Vector2f kAreaLowerBound) 
		{
			return pointInArea(Vertex(kPoint.x, kPoint.y), Vertex(kAreaUpperBound.x, kAreaUpperBound.y), Vertex(kAreaLowerBound.x, kAreaLowerBound.y)); 
		}

		static double magnitude(const Vertex kVector) { return sqrt(pow(kVector.x, 2) + pow(kVector.y, 2)); }

		static double angleFromUnitVec(const Vertex kUnitVector) { return (atan2(kUnitVector.y, kUnitVector.x) / g_kfPi) * 180; }

		static double invertDouble(const double kDouble) { return kDouble * -1; }
};

#endif