
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
			desiredState.setVel(0.0); // set the speed of the robot in the desired state
			desiredState.setDeltaHeading(0.0); // Set the heading change of the robot

			if (m_bPathComplete)
			{
				std::cerr << "\n Path Complete. \n";
			}
			else
			{
				// If the path is empty
				if (m_pPathfinding->getPath().empty())
				{
					// Creates path to the Node closest to the goal Pose
					m_pPathfinding->createPathTo(m_pPathfinding->closestNode(m_pPathfinding->getGoalPose()));
				}
				// Path is not empty
				else
				{
					std::cerr
						<< "\n m_pPathfinding->getGoalPose.getX(): " << m_pPathfinding->getGoalPose().getX()
						<< "\n m_pPathfinding->getGoalPose.getY(): " << m_pPathfinding->getGoalPose().getY()
						<< "\n m_pPathfinding->getStartPose.getX(): " << m_pPathfinding->getStartPose().getX()
						<< "\n m_pPathfinding->getStartPose.getY(): " << m_pPathfinding->getStartPose().getY()
						<< "\n m_pPathfinding->getStartPose.getTh(): " << m_pPathfinding->getStartPose().getTh()

						<< "\n m_pPathfinding->getPath().front().getX(): " << m_pPathfinding->getPath().front().getX()
						<< "\n m_pPathfinding->getPath().front().getY(): " << m_pPathfinding->getPath().front().getY()
						<< "\n m_pPathfinding->trueX(myRobot->getX()): " << m_pPathfinding->trueX(myRobot->getX())
						<< "\n m_pPathfinding->trueY(myRobot->getY()): " << m_pPathfinding->trueY(myRobot->getY())
						<< "\n m_pPathfinding->trueTh(myRobot->getTh()): " << m_pPathfinding->trueTh(myRobot->getTh())
						<< '\n';

					double distToNode = Utils::magnitude(
						sf::Vector2f(
							m_pPathfinding->getPath().front().getX() - m_pPathfinding->trueX(myRobot->getX()),
							m_pPathfinding->getPath().front().getY() - m_pPathfinding->trueY(myRobot->getY())
					));

					std::cerr << "\n distToNode: " << distToNode << '\n';

					if (distToNode < myRobot->getRobotRadius())
					{
						if (m_pPathfinding->getPath().size() == 1)
						{
							m_bPathComplete = true;
						}

						m_pPathfinding->popPath();
						m_PID.error.clear();
					}
					else
					{
						// Vector of displacement between Robot and the next Path point
						sf::Vector2f angleVec = sf::Vector2f(m_pPathfinding->trueX(myRobot->getX()), m_pPathfinding->trueY(myRobot->getY())) - sf::Vector2f(m_pPathfinding->getPath().front().getX(), m_pPathfinding->getPath().front().getY());
						// Unit vector of this displacement
						sf::Vector2f angleUnitVec = angleVec / (float)Utils::magnitude(angleVec);

						// Angle of the unit vector in degrees
						double lineAngle = Utils::angleFromUnitVec(angleUnitVec);
						double robotAngle = m_pPathfinding->trueTh(myRobot->getTh());

						while (lineAngle > 360)  lineAngle -= 360;
						while (lineAngle < 0)  lineAngle += 360;

						double angleDiff = lineAngle - robotAngle;

						std::cerr
							<< "\n lineAngle: " << lineAngle
							<< "\n robotAngle: " << robotAngle
							<< "\n angleDiff: " << angleDiff
							<< '\n';

						// Setpoint
						m_PID.dSetPoint = angleDiff;

						m_PID.error.push_back(m_PID.dSetPoint);

						// Calculate output
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