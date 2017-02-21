#ifndef WANDER_H
#define WANDER_H

#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include "baseAction.h"
#include "utils.h"

class Wander : public BaseAction
{
	public:
	
		Wander(); // Constructor
		virtual ~Wander() {}  // Destructor

		virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
		ArActionDesired desiredState; // Holds state of the robot that we wish to action

	private:

		void calcState();

		int m_iDistanceMoved = -1;

		double m_dTargetHeading;
		int m_iTargetDistance;

		int m_iTargetPosX;
		int m_iTargetPosY;
};

#endif