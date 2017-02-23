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

		// Constructor
		Path(const ArMap kMap, const ArPose kStartPose, const ArPose kGoalPose);
		// Deconstructor
		virtual ~Path() {}  

		// Body of the action
		virtual ArActionDesired * fire(ArActionDesired d); 
		// State of the robot that we wish to action
		ArActionDesired desiredState; 
	
		// Returns the Pathfinding object
		std::shared_ptr<Pathfinding> getPathfinding() { return m_pPathfinding; }

	private:

		// Pathfinding object for creating a path
		std::shared_ptr<Pathfinding> m_pPathfinding = nullptr;

		// If the path has been complete
		bool m_bPathComplete;
		
		// PID data
		PIDData m_PID;

		// Calculates proportional gain
		double calcP();
		// Calculates integral gain
		double calcI();
		// Calculates derivative gain
		double calcD();
		// Calculates RMSE
		double calcRMSE();

		// Calculates the action's state
		void calcState();
};

#endif