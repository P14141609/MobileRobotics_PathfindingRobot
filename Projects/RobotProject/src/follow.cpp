
#include "follow.h"

// Constructor
Follow::Follow()
{
	m_dSpeed = 200.0; // Set the robots speed to 0.2 m/s. 0.2 is top speed
	m_dDeltaHeading = 0; // Straight line

	m_sensor.m_range.dMin = 250.0; // 0.25m
	m_sensor.m_range.dMax = 1500.0; // 1.5m

	m_PID.dSetPoint = 700.0; // 0.7 m from edge

	// Defines PID gain values
	m_PID.dKp = 0.125;
	m_PID.dKi = 0.0005;
	m_PID.dKd = 2.5;
}

// Body of action
ArActionDesired * Follow::fire(ArActionDesired d)
{
	desiredState.reset(); // reset the desired state (must be done)
	calcState();
	std::cerr << "\n Follow State: " << stateToString(m_state) << '\n';

	std::cerr
		<< "\n P: " << calcP()
		<< "\n I: " << calcI()
		<< "\n D: " << calcD()
		<< "\n RMSE: " << calcRMSE()
		<< '\n';

	// Get sonar readings
	calcSensors();

	switch (m_state)
	{
		case IDLE:
		{
			m_PID.error.clear();
		}break;

		case ACTIVE:
		{
			// If the front edge is in range
			if (m_sensor.m_distance.dFront <= m_PID.dSetPoint + 700)
			{
				// If the right edge is closer than the left edge
				if (m_sensor.m_distance.dRight <= m_sensor.m_distance.dLeft)
				{
					// Push error - dist to left edge
					m_PID.error.push_back(m_sensor.m_distance.dLeft - m_PID.dSetPoint);
				}

				// Else If the left edge is closer than the right edge
				else if (m_sensor.m_distance.dLeft <= m_sensor.m_distance.dRight)
				{
					// Push error - dist to right edge
					m_PID.error.push_back(m_PID.dSetPoint - m_sensor.m_distance.dRight);
				}
			}

			// Else If the right edge is the closest
			else if (m_sensor.m_distance.dRight <= Utils::minDouble(m_sensor.m_distance.dFront, m_sensor.m_distance.dLeft))
			{
				// Push error - dist to right edge
				m_PID.error.push_back(m_PID.dSetPoint - m_sensor.m_distance.dRight);
			}

			// Else If the left edge is the closest
			else if (m_sensor.m_distance.dLeft <= Utils::minDouble(m_sensor.m_distance.dFront, m_sensor.m_distance.dRight))
			{
				// Push error - dist to left edge
				m_PID.error.push_back(m_sensor.m_distance.dLeft - m_PID.dSetPoint);
			}
			
			// Calculate output
			m_PID.dOutput = calcP() + calcI() + calcD();

			// Implement control action
			m_dDeltaHeading = m_PID.dOutput;

			// Consequence
			desiredState.setVel(m_dSpeed); // set the speed of the robot in the desired state
			desiredState.setDeltaHeading(m_dDeltaHeading); // Set the heading change of the robot
		}break;
	}

	return &desiredState; // give the desired state to the robot for actioning
}

double Follow::calcP()
{
	if (m_PID.error.size() > 0)
	{
		// P = Kp * e(t)
		return m_PID.dKp * m_PID.error.at(m_PID.error.size() - 1);
	}
	else
		return 0.0;
}
double Follow::calcI()
{
	if (m_PID.error.size() > 0)
	{
		// Threshold: num of errors to sum
		unsigned int threshold = 100;
		// Sum errors
		double sumOfErrors = 0.0;
		unsigned int iLastErrorToSum = (m_PID.error.size() - threshold);

		// For every entry in m_error
		for (unsigned int i = 0; i < m_PID.error.size(); i++)
		{			
			//I cannot make a bloody for loop at sums the last x entries
			// If i is greater or equal to size-threshold
			// Eg i = 12 this is greater than (size 25 - threshold 15 = 10): add to sum
			if (i >= iLastErrorToSum)
			{
				// Add entry to sum of errors
				sumOfErrors += m_PID.error.at(i);
			}
		}

		double dI = sumOfErrors * m_PID.dKi;
		
		// Multiply by gain
		return dI;
	}
	else
		return 0.0;
}
double Follow::calcD()
{
	if (m_PID.error.size() > 1)
	{
		// D = Kd * (e(t) - e(t-1))
		return m_PID.dKd * (m_PID.error.at(m_PID.error.size() - 1) - m_PID.error.at(m_PID.error.size() - 2));
	}
	else
	{
		return 0.0;
	}
}
double Follow::calcRMSE()
{
	double RMSE = 0.0;
	for (unsigned int i = 0; i < m_PID.error.size(); i++)
	{
		RMSE += m_PID.error[i];
	}
	RMSE *= RMSE;
	RMSE = sqrt(RMSE / m_PID.error.size());
	return RMSE;
}

void Follow::calcState()
{
	// If there's an edge within range infront or to either side AND If all edges are greater than the minimum range
	if (Utils::minDouble(m_sensor.m_distance.dFront, m_sensor.m_distance.dLeft, m_sensor.m_distance.dRight) <= m_sensor.m_range.dMax && Utils::minDouble(m_sensor.m_distance.dFront, m_sensor.m_distance.dLeft, m_sensor.m_distance.dRight) >= m_sensor.m_range.dMin)
	{
		m_state = ACTIVE;
	}

	else
	{
		m_state = IDLE;
	}
}