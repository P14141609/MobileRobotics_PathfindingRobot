
#include "avoid.h"

// Constructor
Avoid::Avoid()
{
	m_dSpeed = 25; // Set the robots speed to 0.025 m/s. 0.2 is top speed
	m_dDeltaHeading = 0; // Straight line

	//m_sensor.m_range.dMin = 0.0;
	m_sensor.m_range.dMax = 250.0; // 0.25m
}

// Body of action
ArActionDesired * Avoid::fire(ArActionDesired d)
{
	desiredState.reset(); // reset the desired state (must be done)
	calcState();
	system("cls");
	std::cerr << "\n Avoid State: " << Utils::stateToString(m_state) << '\n';

	std::cerr
		<< "\n Sensors: front(" << m_sensor.m_distance.dFront << ") left(" << m_sensor.m_distance.dLeft << ") right(" << m_sensor.m_distance.dRight << ")"
		<< "\n Speed: " << m_dSpeed
		<< "\n Heading: " << myRobot->getPose().getTh()
		<< "\n Position: x(" << myRobot->getX() << ") y(" << myRobot->getY() << ")"
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
	}

	return &desiredState; // give the desired state to the robot for actioning
}

void Avoid::calcState()
{
	// If there's an edge within the range
	if (Utils::minDouble(m_sensor.m_distance.dFront, m_sensor.m_distance.dLeft, m_sensor.m_distance.dRight) <= m_sensor.m_range.dMax)
	//if ((Utils::minDouble(m_sensor.m_distance.dFront, m_sensor.m_distance.dLeftFront, m_sensor.m_distance.dRightFront) <= m_sensor.m_range.dMax) 
	//	|| (Utils::minDouble(m_sensor.m_distance.dLeft, m_sensor.m_distance.dRight) <= m_sensor.m_range.dMax*0.25))
	//if ((m_sensor.m_distance.dFront <= m_sensor.m_range.dMax) || (Utils::minDouble(m_sensor.m_distance.dLeft, m_sensor.m_distance.dRight) <= m_sensor.m_range.dMax*0.75))
	{
		m_state = ACTIVE;
	}

	else
	{
		m_state = IDLE;
	}
}