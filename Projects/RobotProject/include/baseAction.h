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

		// Constructor
		BaseAction() : ArAction("") {}
		// Virtual: Deconstructor
		virtual ~BaseAction() {}

		// Virtual: Body of the action
		virtual ArActionDesired * fire(ArActionDesired d) = 0;
		// State of the robot that we wish to action
		ArActionDesired desiredState;
	
	protected:
	
		// Defines possible action states
		enum ActionState { IN_SETUP, IDLE, ACTIVE };
		// Current action state: default IN_SETUP
		ActionState m_state = IN_SETUP;

		// Speed of the robot in mm/s
		double m_dSpeed; 
		// Change in heading
		double m_dDeltaHeading;
	
		// Sensor data
		Sensor m_sensor;

		// Updates Sensor member with real-time data
		void updateSensors()
		{
			// Get sonar readings and pushes to member
			m_sensor.m_distance.dFront = myRobot->getClosestSonarRange(-25, 25);
			m_sensor.m_distance.dFrontLeft = myRobot->getClosestSonarRange(0, 25);
			m_sensor.m_distance.dFrontRight = myRobot->getClosestSonarRange(-25, 0);
			m_sensor.m_distance.dLeft = myRobot->getClosestSonarRange(25, 100);
			m_sensor.m_distance.dRight = myRobot->getClosestSonarRange(-100, -25);
		}

		// Returns the string value of an action state
		static std::string stateToString(const int kiState)
		{
			if (kiState == 0) return "IN_SETUP";
			if (kiState == 1) return "IDLE";
			if (kiState == 2) return "ACTIVE";
			else return "ERROR - MISSING ENUM VALUE";
		}

		// Virtual: Calculates the action's state
		virtual void calcState() = 0;
};

#endif