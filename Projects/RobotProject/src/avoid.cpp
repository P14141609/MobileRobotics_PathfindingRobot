
#include "avoid.h"

// Constructor
Avoid::Avoid()
{
	m_dSpeed = 25; // Set the robots speed to 0.025 m/s. 0.2 is top speed
	m_dDeltaHeading = 0; // Straight line

	m_sensor.m_range.dMax = 250.0; // 0.25m
}

// Body of action
ArActionDesired * Avoid::fire(ArActionDesired d)
{
	desiredState.reset(); // reset the desired state (must be done)
	calcState();
	system("cls");
	std::cerr << "\n Avoid State: " << stateToString(m_state) << '\n';

	std::cerr
		<< "\n Sensors: front(" << m_sensor.m_distance.dFront << ")"
		<< "\n myRobot.Speed: " << myRobot->getVel()
		<< "\n myRobot.Pose: x(" << myRobot->getX() << ") y(" << myRobot->getY() << ") th(" << myRobot->getTh() << ")"
		<< '\n';

	// Get sonar readings
	updateSensors();

	switch (m_state)
	{
		case IDLE:
		{
		}break;

		case ACTIVE:
		{
			// If both sides are neaby
			if ((m_sensor.m_distance.dLeft <= m_sensor.m_range.dMax) && (m_sensor.m_distance.dRight <= m_sensor.m_range.dMax))
			{
				if (m_sensor.m_distance.dLeft <= m_sensor.m_distance.dRight)
				{
					m_dDeltaHeading = -20.0;
					m_dSpeed = 0.0;
				}

				else
				{
					m_dDeltaHeading = 20.0;
					m_dSpeed = 0.0;
				}
			}
			// Else If a single edge is nearby
			else
			{
				if (m_sensor.m_distance.dLeft <= m_sensor.m_distance.dRight)
				{
					m_dDeltaHeading = -10.0;
					m_dSpeed = 0.0;
				}

				else
				{
					m_dDeltaHeading = 10.0;
					m_dSpeed = 0.0;
				}
			}

			// Consequence
			desiredState.setVel(m_dSpeed); // set the speed of the robot in the desired state
			desiredState.setDeltaHeading(m_dDeltaHeading); // Set the heading change of the robot
		}break;

		default: break;
	}

	return &desiredState; // give the desired state to the robot for actioning
}

void Avoid::calcState()
{
	// If sensor distance has not been calculated
	if (m_sensor.m_distance.dFront == INT32_MAX)
	{
		m_state = IN_SETUP;
	}

	// Else If there's an edge within the range on the front sensor
	else if (m_sensor.m_distance.dFront <= m_sensor.m_range.dMax)
	{
		m_state = ACTIVE;
	}

	// Otherwise
	else
	{
		m_state = IDLE;
	}
}