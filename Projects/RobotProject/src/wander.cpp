
#include "wander.h"

// Constructor
Wander::Wander()
{
	m_dSpeed = 200; // Set the robots speed to 0.2 m/s. 0.2 is top speed
	m_dDeltaHeading = 0; // Straight line

	m_sensor.m_range.dMin = 1500.0; // 1.5m
	//m_sensor.m_range.dMax = 0.0;
}

// Body of action
ArActionDesired * Wander::fire(ArActionDesired d)
{
	desiredState.reset(); // reset the desired state (must be done)
	calcState();
	std::cerr << "\n Wander State: " << stateToString(m_state) << '\n';

	std::cerr
		<< "\n m_dDeltaHeading: " << m_dDeltaHeading
		<< "\n m_dTargetHeading: " << m_dTargetHeading
		<< "\n m_iTargetDistance: " << m_iTargetDistance
		<< "\n m_iDistanceMoved: " << m_iDistanceMoved
		<< '\n';

	// Get sonar readings
	calcSensors();

	switch (m_state)
	{
		case IDLE:
		{
		}break;
	
		case ACTIVE:
		{
			m_dDeltaHeading = 0; // Resets the delta heading to 0

			// If DistanceMoved is in reset state
			if (m_iDistanceMoved == -1)
			{
				// Declares a cone, in degrees, that the delta heading will lie in
				double dCone = 140;

				// Defines random double for a delta heading
				m_dDeltaHeading = (rand() % (int)dCone) - (dCone*0.5); // Creates random distance between -70 and 70
				
				// Defines TargetHeading
				m_dTargetHeading = myRobot->getTh() + m_dDeltaHeading;

				// Creates random distance between 500 and 1500
				m_iTargetDistance = (rand() % 1000) + 500; 

				// Iterates DistanceMoved
				m_iDistanceMoved = myRobot->getMoveDoneDist();
			}
			// Else if DistanceMoved is greater or equal to TargetDistance
			else if (m_iDistanceMoved >= m_iTargetDistance)
			{
				// DistanceMoved put into a reset state
				m_iDistanceMoved = -1;
			}
			// Else
			else
			{
				// Iterates DistanceMoved
				m_iDistanceMoved += myRobot->getMoveDoneDist();
			}

			// Consequence
			desiredState.setVel(m_dSpeed); // set the speed of the robot in the desired state
			desiredState.setHeading(m_dTargetHeading); // Set the heading change of the robot
		}break;
	}

	return &desiredState; // give the desired state to the robot for actioning
}

void Wander::calcState()
{
	// When no walls or objects are detected within 1.5m
	if (Utils::minDouble(m_sensor.m_distance.dFront, m_sensor.m_distance.dLeft, m_sensor.m_distance.dRight) >= m_sensor.m_range.dMin)
	{
		m_state = ACTIVE;
	}

	else
	{
		m_state = IDLE;
	}
}