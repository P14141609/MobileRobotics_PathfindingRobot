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

class Utils
{
	public:
		
		// Returns the smallest double provided
		static double minDouble(const double kdA, const double kdB)
		{
			if (kdA <= kdB) return kdA;
			else return kdB;
		}
		// Returns the smallest double provided
		static double minDouble(const double kdA, const double kdB, const double kdC) { return minDouble(minDouble(kdA, kdB), kdC); }

		// Returns the largest double provided
		static double maxDouble(const double kdA, const double kdB)
		{
			if (kdA >= kdB) return kdA;
			else return kdB;
		}
		// Returns the largest double provided
		static double maxDouble(const double kdA, const double kdB, const double kdC) { return maxDouble(maxDouble(kdA, kdB), kdC); }

		// Returns the smallest integer provided
		static int minInt(const int kiA, const int kiB)
		{
			if (kiA <= kiB) return kiA;
			else return kiB;
		}
		// Returns the smallest integer provided
		static int minInt(const int kiA, const int kiB, const int kiC) { return minInt(minInt(kiA, kiB), kiC); }

		// Returns the largest integer provided
		static int maxInt(const int kiA, const int kiB)
		{
			if (kiA >= kiB) return kiA;
			else return kiB;
		}
		// Returns the largest integer provided
		static int maxInt(const int kiA, const int kiB, const int kiC) { return maxInt(maxInt(kiA, kiB), kiC); }

		// Returns whether a point is within the lower and upper bounds
		static bool pointInArea(const ArPose kPoint, const ArPose kAreaUpperBound, const ArPose kAreaLowerBound)
		{
			// If point.x is less than lower bound: Out of bounds
			if (kPoint.getX() <= kAreaLowerBound.getX()) return false;

			// If point.y is less than lower bound: Out of bounds
			if (kPoint.getY() <= kAreaLowerBound.getY()) return false;

			// If point.x is greater than upper bound: Out of bounds
			if (kPoint.getX() >= kAreaUpperBound.getX()) return false;

			// If point.y is greater than upper bound: Out of bounds
			if (kPoint.getY() >= kAreaUpperBound.getY()) return false;

			// No False returns called: call return True
			return true;
		}

		// Returns the magnitude of a vector
		static double magnitude(const sf::Vector2f kVector) { return sqrt(pow(kVector.x, 2) + pow(kVector.y, 2)); }

		// Returns the angle of a unit vector in degrees
		static double angleFromUnitVec(const sf::Vector2f kUnitVector) { return (atan2(kUnitVector.y, kUnitVector.x) / g_kfPi) * 180; }

		// Returns the input inverted from positive to negative and vice versa
		static double invertDouble(const double kDouble) { return kDouble * -1; }

		// Returns a unit vector
		static sf::Vector2f angleUnitVector(const sf::Vector2f kVector)
		{
			return kVector / (float)Utils::magnitude(kVector);
		}
		// Returns a unit vector
		static sf::Vector2f angleUnitVector(const double kdAngle)
		{
			return sf::Vector2f(cosf(kdAngle * g_kfDegToRad), sinf(kdAngle * g_kfDegToRad));
		}

		// Returns a value bound to a given min and max
		static double bindNum(const double kdNumber, const double kdMin, const double kdMax)
		{
			double dResult = kdNumber;

			while (dResult > kdMax) dResult -= kdMax;
			while (dResult < kdMin) dResult += kdMax;

			return dResult;
		}
};

#endif