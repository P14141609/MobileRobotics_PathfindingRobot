#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include "baseAction.h"
#include "utils.h"
#include "pathfinding.h"

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

class Path : public BaseAction
{
	public:
	
		Path(const ArMap kMap, const ArPose kStartPose, const ArPose kGoalPose); // Constructor
		virtual ~Path() {}  // Destructor
	
		virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
		ArActionDesired desiredState; // Holds state of the robot that we wish to action
	
		std::shared_ptr<Pathfinding> getPathfinding() { return m_pPathfinding; }

	private:

		std::shared_ptr<Pathfinding> m_pPathfinding = nullptr;

		PIDData m_PID;

		void calcState();

		// Calcuating functions
		double calcP();
		double calcI();
		double calcD();
		double calcRMSE();
};

#endif