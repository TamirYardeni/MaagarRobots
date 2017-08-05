#include "RobotManager.h"

RobotManager::RobotManager(
	Hamster * hamster, //LocalizationManager * localizationManager,
	int inflationRadius,
	double mapHeight, double mapWidth)
{
	this->hamster = hamster;
	//this->localizationManager = localizationManager;
	this->inflationRadius = inflationRadius;
	this->mapHeight = mapHeight;
	this->mapWidth = mapWidth;

	this->prevX = 0;
	this->prevY = 0;
	this->prevYaw = 0;
	this->currX = 0;
	this->currY = 0;
	this->currYaw = 0;
}

void RobotManager::Initialize(Location startLocation)
{
	hamsterStartX = startLocation.x - (mapWidth / 2);
	hamsterStartY = startLocation.y - (mapHeight / 2);

	currX = hamsterStartX;
	currY = hamsterStartY;
	currYaw = startLocation.yaw;

	Pose initialPose;
	initialPose.setX(hamsterStartX);
	initialPose.setY(hamsterStartY);
	initialPose.setHeading(startLocation.yaw);

	// The call to setInitialPose sets the parameter pose as the relative
	// origin of the robot's coordinate system.
	// The robot wouldn't start moving without this call, and after calling it - the values
	// returned from getPose would be relative to the initialPose we chose to set.
	sleep(3);
	hamster->setInitialPose(initialPose);

	cout << "Start position robot : x - " << hamsterStartX << "  y - " << hamsterStartY << endl;

	//localizationManager->InitParticles();

	UpdateLocation();
}

double RobotManager::GetDeltaX() const
{
	return currX - prevX;
}

double RobotManager::GetDeltaY() const
{
	return currY - prevY;
}

double RobotManager::GetDeltaYaw() const
{
	return currYaw - prevYaw;
}

Location RobotManager::GetCurrHamsterLocation()
{
	/*Particle * topParticle = localizationManager->GetTopParticle();

	Location currLocation;
	currLocation = {
			.x = topParticle->x + 2*inflationRadius,
			.y = topParticle->y + 2*inflationRadius,
			.yaw = topParticle->yaw
	};*/

	Pose currPose = hamster->getPose();

	// Substract the initial pose coordinates from the current pose to get
	// coordinates relative to (0,0) instead of to the initial pose we set
	double currX = (currPose.getX() - hamsterStartX) * 10;
	double currY = (currPose.getY() - hamsterStartY) * 10;

	double currYaw = currPose.getHeading();

	if (currYaw < 0)
	{
		currYaw += 360;
	}
	else if (currYaw > 360)
	{
		currYaw -= 360;
	}

	Location currLocation = { .x = currX, .y = currY, .yaw = currYaw };

	return currLocation;
}

void RobotManager::UpdateLocation()
{
	prevX = currX;
	prevY = currY;
	prevYaw = currYaw;

	Location currentLocation = GetCurrHamsterLocation();

	// Update the current and previous locations by the position of the robot
	currX = currentLocation.x;
	currY = currentLocation.y;
	currYaw = currentLocation.yaw;
}

RobotManager::~RobotManager() {
	// TODO Auto-generated destructor stub
}
