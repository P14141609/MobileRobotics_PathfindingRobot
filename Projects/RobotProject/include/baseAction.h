#ifndef BASEACTION_H
#define BASEACTION_H

#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include "utils.h"
#include "sensor.h"

class BaseAction : public ArAction
{
	public:
		BaseAction() : ArAction("") {} // Constructor
		virtual ~BaseAction() {}  // Destructor

		virtual ArActionDesired * fire(ArActionDesired d) = 0; // Body of the action
		ArActionDesired desiredState; // Holds state of the robot that we wish to action
	
	protected:
	
		enum ActionState { IN_SETUP, IDLE, ACTIVE };
		ActionState m_state = IDLE;
		virtual void calcState() = 0;

		double m_dSpeed; // Speed of the robot in mm/s
		double m_dDeltaHeading; // Change in heading
	
		// Member for sensor data storage
		Sensor m_sensor;

		void calcSensors()
		{
			// Get sonar readings and pushes to member
			m_sensor.m_distance.dFront = myRobot->getClosestSonarRange(-25, 25);
			m_sensor.m_distance.dLeftFront = myRobot->getClosestSonarRange(0, 25);
			m_sensor.m_distance.dRightFront = myRobot->getClosestSonarRange(-25, 0);
			m_sensor.m_distance.dLeft = myRobot->getClosestSonarRange(25, 100);
			m_sensor.m_distance.dRight = myRobot->getClosestSonarRange(-100, -25);
		}

		// Returns the string value of a state
		static std::string stateToString(const int kiState)
		{
			if (kiState == 0) return "IDLE";
			if (kiState == 1) return "ACTIVE";
			else return "ERROR";
		}
};

#endif