#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include "utils.h"

struct Range
{
	// Minimum range
	double dMin;
	// Maximum range
	double dMax;
};

struct Distance
{
	// Sensor distance: Front
	double dFront = INT32_MAX;
	// Sensor distance: Front-Left
	double dFrontLeft = INT32_MAX;
	// Sensor distance: Front-Right
	double dFrontRight = INT32_MAX;
	// Sensor distance: Left
	double dLeft = INT32_MAX;
	// Sensor distance: Right
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