#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include "utils.h"

struct Range
{
	// Sensor ranges
	double dMin;
	double dMax;
};

struct Distance
{
	// Sensor distances
	// Front
	double dFront = INT32_MAX;
	// Two halves of Front
	double dLeftFront = INT32_MAX;
	double dRightFront = INT32_MAX;
	// Sides
	double dLeft = INT32_MAX;
	double dRight = INT32_MAX;
};

class Sensor
{
	public:	
		// Sensor ranges
		Range m_range;

		// Sensor distances
		Distance m_distance;
};

#endif