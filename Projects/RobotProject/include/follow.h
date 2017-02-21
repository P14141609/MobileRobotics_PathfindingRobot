#ifndef FOLLOW_H
#define FOLLOW_H

#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include "baseAction.h"
#include "utils.h"

struct PIDData
{
	double dKp; // Proportional gain
	double dKi; // Integral gain
	double dKd; // Derivative gain

	// Control variables
	double dSetPoint; // Set point of the controller
	std::vector<double> error; // Array of errors
	double dOutput; // Final output signal
};

class Follow : public BaseAction
{
	public:

		Follow(); // Constructor
		virtual ~Follow() {}  // Destructor

		virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
		ArActionDesired desiredState; // Holds state of the robot that we wish to action

	private:

		void calcState();

		PIDData m_PID;

		// Calcuating functions
		double calcP();
		double calcI();
		double calcD();
		double calcRMSE();
};

#endif