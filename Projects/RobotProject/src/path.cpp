
#include "path.h"

// Constructor
Path::Path(const ArMap kMap, const ArPose kStartPose, const ArPose kGoalPose)
{
	//
	m_pPathfinding = std::shared_ptr<Pathfinding>(new Pathfinding(kMap, Aria::getRobotList()->front(), kStartPose, kGoalPose));

	m_bPathComplete = false;

	m_dSpeed = 200.0; // Set the robots speed to 0.2 m/s. 0.2 is top speed
	m_dDeltaHeading = 0; // Straight line

	m_sensor.m_range.dMin = 250.0; // 0.25m

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

	std::cerr << "\n Path State: " << stateToString(m_state) << '\n';

	std::cerr
		<< "\n m_pPathfinding->getNodes().size(): " << m_pPathfinding->getNodes().size()
		<< "\n m_pPathfinding->getPath().size(): " << m_pPathfinding->getPath().size()
		<< "\n P: " << calcP()
		<< "\n I: " << calcI()
		<< "\n D: " << calcD()
		<< "\n RMSE: " << calcRMSE()
		<< '\n';

	// Updates sensor readings
	updateSensors();

	// If pathfinding nodes have not been created
	if (m_pPathfinding->getNodes().empty()) { m_pPathfinding->createNodes(); };

	switch (m_state)
	{
		case IDLE:
		{
			// Clears PID error vector
			m_PID.error.clear();
		}break;

		case ACTIVE:
		{
			// If path has been completed
			if (m_bPathComplete)
			{
				std::cerr << "\n Path Complete. \n";
			}
			// Else path is underway
			else
			{
				// If the path queue is empty
				if (m_pPathfinding->getPath().empty())
				{
					// Creates path to the Node closest to the goal pose
					m_pPathfinding->createPathTo(m_pPathfinding->closestNode(m_pPathfinding->getGoalPose()));
				}
				// Path queue is not empty
				else
				{
					std::cerr
						<< "\n m_pPathfinding->getPath().front().getX(): " << m_pPathfinding->getPath().front().getX()
						<< "\n m_pPathfinding->getPath().front().getY(): " << m_pPathfinding->getPath().front().getY()
						<< "\n myRobot->getX(): " << myRobot->getX()
						<< "\n m_pPathfinding->myRobot->getY(): " << myRobot->getY()
						<< '\n';

					// Vector of displacement between Robot and the next path Node
					sf::Vector2f displacement = sf::Vector2f(m_pPathfinding->getPath().front().getX(), m_pPathfinding->getPath().front().getY()) - sf::Vector2f(myRobot->getX(), myRobot->getY());
					// Distance to next path Node
					double distToNode = Utils::magnitude(displacement);

					std::cerr << "\n distToNode: " << distToNode << '\n';

					// If the Robot is close to the Node
					if (distToNode < myRobot->getRobotRadius())
					{
						// If Node is the last in the path
						if (m_pPathfinding->getPath().size() == 1)
						{
							// Set path to completed
							m_bPathComplete = true;
						}

						// Pops Node off queue
						m_pPathfinding->popPath();
						// Clears PID error vector
						m_PID.error.clear();
					}
					else
					{
						// Angle of the line in degrees
						double lineAngle = Utils::angleFromUnitVec(Utils::angleUnitVector(displacement));

						// Binds the line angle from 0-360 degrees
						lineAngle = Utils::bindNum(lineAngle, 0, 360);

						// Difference between the line and robot angles
						double angleDiff = lineAngle - myRobot->getTh();

						std::cerr
							<< "\n lineAngle: " << lineAngle
							<< "\n myRobot->getTh(): " << myRobot->getTh()
							<< "\n angleDiff: " << angleDiff
							<< '\n';

						// Angle difference applied to setpoint
						m_PID.dSetPoint = angleDiff;

						// Pushes setpoint onto error vector
						m_PID.error.push_back(m_PID.dSetPoint);

						// Calculates output
						m_PID.dOutput = m_PID.dSetPoint;// calcP() + calcI() + calcD();

						// Implement control action
						m_dDeltaHeading = m_PID.dOutput;

						// Consequence
						desiredState.setVel(m_dSpeed); // set the speed of the robot in the desired state
						desiredState.setDeltaHeading(m_dDeltaHeading); // Set the heading change of the robot
					}
				}
			}
		}break;

		default: break;
	}

	return &desiredState; // give the desired state to the robot for actioning
}

double Path::calcP()
{
	// If error vector is not empty
	if (!m_PID.error.empty())
	{
		// P = Kp * e(t)
		return m_PID.dKp * m_PID.error.at(m_PID.error.size() - 1);
	}
	// Else return 0
	else return 0.0;
}
double Path::calcI()
{
	// If error vector is not empty
	if (!m_PID.error.empty())
	{
		// Threshold: num of errors to sum
		unsigned int threshold = 100;
		// Sum of errors
		double sumOfErrors = 0.0;
		unsigned int iLastErrorToSum = (m_PID.error.size() - threshold);

		// For every entry in the error vector
		for (unsigned int i = 0; i < m_PID.error.size(); i++)
		{
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
	// Else return 0
	else return 0.0;
}
double Path::calcD()
{
	// If error vector contains more than 1 error
	if (m_PID.error.size() > 1)
	{
		// D = Kd * (e(t) - e(t-1))
		return m_PID.dKd * (m_PID.error.at(m_PID.error.size() - 1) - m_PID.error.at(m_PID.error.size() - 2));
	}
	// Else return 0
	else return 0.0;
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
	// If sensor distance has not been calculated
	if (m_sensor.m_distance.dFront == INT32_MAX)
	{
		m_state = IN_SETUP;
	}

	// Else If no edge is within minimum range
	else if (m_sensor.m_distance.dFront > m_sensor.m_range.dMin)
	{
		m_state = ACTIVE;
	}

	// Otherwise
	else
	{
		m_state = IDLE;
	}
}