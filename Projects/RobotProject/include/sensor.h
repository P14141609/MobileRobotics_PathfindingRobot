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
	double dFront;

	double dLeftFront;
	double dRightFront;

	double dLeft;
	double dRight;
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