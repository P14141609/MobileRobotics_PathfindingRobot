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
	
		Avoid(); // Constructor
		virtual ~Avoid() {}  // Destructor

		virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
		ArActionDesired desiredState; // Holds state of the robot that we wish to action

	private:

		void calcState();
};

#endif