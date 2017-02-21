
#include "path.h"

// Constructor
Path::Path(const ArMap kMap, const ArPose kStartPose, const ArPose kGoalPose)
{
	//
	m_pPathfinding = std::shared_ptr<Pathfinding>(new Pathfinding(kMap, Aria::getRobotList()->front(), kStartPose, kGoalPose));

	m_dSpeed = 200.0; // Set the robots speed to 0.2 m/s. 0.2 is top speed
	m_dDeltaHeading = 0; // Straight line

	m_sensor.m_range.dMin = 250.0; // 0.25m
	//m_sensor.m_range.dMax = 250.0;

	// Defines PID gain values
	m_PID.dKp = 0.125;
	m_PID.dKi = 0.0005;
	m_PID.dKd = 2.5;
}

// Body of action
ArActionDesired * Path::fire(ArActionDesired d)
{
	desiredState.reset(); // reset the desired state (must be done)
	calcState();
	std::cerr << "\n Pathfinding State: " << Utils::stateToString(m_state) << '\n';

	std::cerr
		<< "\n m_pPathfinding->getNodes().size(): " << m_pPathfinding->getNodes().size()
		<< "\n m_pPathfinding->getPath().size(): " << m_pPathfinding->getPath().size()
		<< "\n P: " << calcP()
		<< "\n I: " << calcI()
		<< "\n D: " << calcD()
		<< "\n RMSE: " << calcRMSE()
		<< '\n';

	// Get sonar readings
	calcSensors();

	// If pathfinding nodes have not been created
	if (m_pPathfinding->getNodes().empty()) { m_pPathfinding->createNodes(); };

	switch (m_state)
	{
		case IDLE:
		{
			m_PID.error.clear();
		}break;

		case ACTIVE:
		{
			// If the path is empty
			if (m_pPathfinding->getPath().empty())
			{
				//int iInput;
				//
				//// Pick target node
				//std::cerr << "\nPick a node to path towards: ";
				//std::cin >> iInput; std::cerr << '\n';

				// Creates path to Node 0
				m_pPathfinding->createPathTo(m_pPathfinding->closestNode(Vertex(m_pPathfinding->getGoalPose().getX(), m_pPathfinding->getGoalPose().getY())));
			}
			// Path is not empty
			else
			{
				//// Calculate output
				//m_PID.dOutput = calcP() + calcI() + calcD();
				//
				//// Implement control action
				//m_dDeltaHeading = m_PID.dOutput;
				//
				//// Consequence
				//desiredState.setVel(m_dSpeed); // set the speed of the robot in the desired state
				//desiredState.setDeltaHeading(m_dDeltaHeading); // Set the heading change of the robot
			}
		}break;
	}

	return &desiredState; // give the desired state to the robot for actioning
}

double Path::calcP()
{
	if (m_PID.error.size() > 0)
	{
		// P = Kp * e(t)
		return m_PID.dKp * m_PID.error.at(m_PID.error.size() - 1);
	}
	else
		return 0.0;
}
double Path::calcI()
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
double Path::calcD()
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
double Path::calcRMSE()
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

void Path::calcState()
{
	// If no edge is within minimum range
	if (Utils::minDouble(m_sensor.m_distance.dFront, m_sensor.m_distance.dLeft, m_sensor.m_distance.dRight) > m_sensor.m_range.dMin)
	{
		m_state = ACTIVE;
	}

	else
	{
		m_state = IDLE;
	}
}