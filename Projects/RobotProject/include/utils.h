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

		static double magnitude(const sf::Vector2f kVector) { return sqrt(pow(kVector.x, 2) + pow(kVector.y, 2)); }

		static double angleFromUnitVec(const sf::Vector2f kUnitVector) { return (atan2(kUnitVector.y, kUnitVector.x) / g_kfPi) * 180; }

		static double invertDouble(const double kDouble) { return kDouble * -1; }

		// sf::Vector2f: Returns Point rotated around pivot
		static sf::Vector2f rotateAroundPoint(const sf::Vector2f kPivot, sf::Vector2f kPoint, const float kfAngle)
		{
			// Declares proportion of new pos in x/y
			sf::Vector2f unitVec = angleUnitVector(kfAngle);

			// Point translated relative to origin
			kPoint.x -= kPivot.x;
			kPoint.y -= kPivot.y;

			// Calculates the new X/Y being used to translate the point
			float newX = kPoint.x * unitVec.x - kPoint.y * unitVec.y;
			float newY = kPoint.x * unitVec.y + kPoint.y * unitVec.x;

			// Translates point back relative to pivot
			kPoint.x = newX + kPivot.x;
			kPoint.y = newY + kPivot.y;

			// Returns new point
			return kPoint;
		}

		// sf::Vector2f: Returns a Unit Vector
		static sf::Vector2f angleUnitVector(const sf::Vector2f kVector)
		{
			return kVector / (float)Utils::magnitude(kVector);
		}
		// sf::Vector2f: Returns a Unit Vector
		static sf::Vector2f angleUnitVector(const double kdAngle)
		{
			return sf::Vector2f(cosf(kdAngle * g_kfDegToRad), sinf(kdAngle * g_kfDegToRad));
		}

		static double bindNum(const double kdNumber, const double kdMin, const double kdMax)
		{
			double dResult = kdNumber;

			while (dResult > kdMax) dResult -= kdMax;
			while (dResult < kdMin) dResult += kdMax;

			return dResult;
		}
};

#endif