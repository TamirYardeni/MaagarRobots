/*
 * RobotManager.h
 *
 *  Created on: Jul 1, 2017
 *      Author: user
 */

#ifndef ROBOTMANAGER_H_
#define ROBOTMANAGER_H_

#include "../Models/Location.h"
#include <HamsterAPIClientCPP/Hamster.h>
using namespace HamsterAPI;
using namespace std;
/**
    RobotManager.h
    Purpose: This class uses to calculate the "delta" in the movement
    	     of the Hamster robot.
    	     Its main function of this class- UpdateLocation should be called
    	     every time the Hamster moves.
*/
class RobotManager
{
private:
	Hamster * hamster;
	double hamsterStartX, hamsterStartY;
	double prevX, prevY, prevYaw, currX, currY, currYaw;
	int inflationRadius;
	double mapHeight, mapWidth;

public:
	RobotManager(Hamster * hamster,int inflationRadius,double mapHeight, double mapWidth);
	void Initialize(Location startLocation);
	double GetDeltaX() const;
	double GetDeltaY() const;
	double GetDeltaYaw() const;
	Location GetCurrHamsterLocation();
	void UpdateLocation();
	virtual ~RobotManager();
};

#endif /* ROBOTMANAGER_H_ */
