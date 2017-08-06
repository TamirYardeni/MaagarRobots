
#include "Robot.h"

Robot::Robot(cv::Size2f robotSize) : m_size(robotSize)
{
	m_hamster = NULL;
	m_diameter = max(m_size.width,m_size.height);
	m_radius = m_diameter / 2.0;
}

Robot::~Robot(){
    delete m_hamster;
}

int Robot::Init() {
    try{
        m_hamster = new HamsterAPI::Hamster(1);
        return 1;
    }
    catch (const HamsterAPI::HamsterError & connection_error) {
		HamsterAPI::Log::i("Client", connection_error.what());
	}

    return 0;
}

void Robot::postInit(const Pose startPose)
{
	cout << "Robot post connection init" << endl;

	//m_hamster->setInitialPose(m_startPose);
	m_currPos = startPose;
	m_lastPos = m_currPos;
}

void Robot::update()
{
	Pose pose = m_hamster->getPose();
	update(pose.getX(), pose.getY(), pose.getHeading());
}

void Robot::update(float x, float y, float heading)
{
	m_currPos.setX(x);

	// Flip Y pos and heading to fit slam map
	m_currPos.setY(y);
	m_currPos.setHeading(heading);

	// Compute delta
	m_deltaPos.setX(m_currPos.getX() - m_lastPos.getX());
	m_deltaPos.setY(m_currPos.getY() - m_lastPos.getY());
	m_deltaPos.setHeading(m_currPos.getHeading() - m_lastPos.getHeading());

	m_lastPos = m_currPos;
}

bool Robot::gotoPoint(cv::Point2f goal) {
	float angleBetweenRad = atan2(goal.y - getYPos(),goal.x - getXPos());

	int angleBetweenDeg = (int)(angleBetweenRad * RAD2DEG);
	int robotYaw = (int)getYaw();

	if (angleBetweenDeg < 0) angleBetweenDeg+=360;
	if (robotYaw < 0) robotYaw+=360;

	int angleDiff = (robotYaw - angleBetweenDeg);

	// Check if diff between robot yaw and angle between robot pos to waypoint is small
	if (abs(angleDiff) < 10)
	{
		moveForward(0);
	}
	else
	{
		if (angleDiff > 0) {
			if (abs(angleDiff) < 180)
				turnRight();
			else
				turnLeft();
		}
		else {
			if (abs(angleDiff) < 180)
				turnLeft();
			else
				turnRight();
		}

	}

	// Check collision with goal
	float waypointDistance = sqrt(pow(getXPos() - goal.x,2) + pow(getYPos() - goal.y,2));

	return (waypointDistance < 0.2);
}

void Robot::wander() {
	if (isFrontFree()) {
			moveForward();
	}
	else {
		stopMoving();
		if (isLeftFree()) {
			turnLeft();
		}
		else if (isRightFree()) {
			turnRight();
		}
		else if (isBackFree()) {
			moveBackwards();
		}
		else {
			HamsterAPI::Log::i("Client", "I am stuck!");
		}
	}
}

void Robot::getScansBetween(double min, double max, std::vector<double> & distances) {
	HamsterAPI::LidarScan scan = m_hamster->getLidarScan();

	for (size_t i = 0; i < scan.getScanSize(); i++) {
		double degree = scan.getScanAngleIncrement() * i;
		if (degree >= min && degree <= max)
			distances.push_back(scan.getDistance(i));
	}
}

bool Robot::willCollide(std::vector<double> distances, int angle_from_center) {
	HamsterAPI::LidarScan scan = m_hamster->getLidarScan();
	int collisions = 0;

	for (size_t i = distances.size() / 2 - angle_from_center / 2;
			i < distances.size() / 2 + angle_from_center / 2; i++)
		if (distances[i] < 1)//scan.getMaxRange() / 4.0)
			collisions++;

	return collisions >= angle_from_center / 4.0;
}

bool Robot::isFrontFree() {
	// Degrees : [90, 270]

	std::vector<double> distances;

	getScansBetween(90, 270, distances);

	return !willCollide(distances, 40);
}

bool Robot::isLeftFree() {
	// Degrees : [180,360]

	std::vector<double> distances;

	getScansBetween(180, 360, distances);

	return !willCollide(distances, 40);
}

bool Robot::isRightFree() {
	// Degrees : [0, 180]

	std::vector<double> distances;

	getScansBetween(0, 180, distances);

	return !willCollide(distances, 40);
}

bool Robot::isBackFree() {
	// Degrees : [270,360], [0, 90]

	std::vector<double> distances;

	getScansBetween(270, 360, distances);
	getScansBetween(0, 90, distances);

	return !willCollide(distances, 40);
}

void Robot::moveForward(float angle) {
	m_hamster->sendSpeed(0.3, angle);
}

void Robot::turnLeft(float angle) {
	m_hamster->sendSpeed(0.04, angle);
}

void Robot::turnRight(float angle) {
	m_hamster->sendSpeed(0.04, angle);
}

void Robot::moveBackwards(float angle) {
	m_hamster->sendSpeed(-0.1, angle);
}

void Robot::stopMoving() {
    m_hamster->sendSpeed(0.0, 0.0);
}
