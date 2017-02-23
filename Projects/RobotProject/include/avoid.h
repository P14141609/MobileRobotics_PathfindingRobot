#ifndef AVOID_H
#define AVOID_H

#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include "baseAction.h"
#include "utils.h"

class Avoid : public BaseAction
{
	public:
	
		// Constructor
		Avoid(); 
		// Deconstructor
		virtual ~Avoid() {} 

		// Body of the action
		virtual ArActionDesired * fire(ArActionDesired d);
		// State of the robot that we wish to action
		ArActionDesired desiredState;

	private:

		// Calculates the action's state
		void calcState();
};

#endif