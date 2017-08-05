/*
 * VisualDisplay.h
 *
 *  Created on: Jul 1, 2017
 *      Author: user
 */

#ifndef VISUALDISPLAY_H_
#define VISUALDISPLAY_H_

#include "../Models/Location.h"
#include "../Models/Grid.h"
#include <string>
#include <vector>
#include <queue>
#include <HamsterAPIClientCPP/Hamster.h>
using namespace std;
using namespace HamsterAPI;

class VisualDisplay
{
private:
	Grid grid;
	int startRow;
	int startCol;
	int goalRow;
	int goalCol;
	vector<vector<bool> > occupationMap;
	int height;
	int width;
	vector<vector<int> > mapFromPlannedRoute;
	cv::Mat_<cv::Vec3b> routeCvMat;
	string plannedRoute;
	vector<Location> waypoints;
	int numOfWaypoints;
	void InitMapWithRoute();
	void ColorPixelByRoute(int currentCellValue, int i, int j);
	void ColorPixelByParticles(int currentCellValue, int i, int j);

public:
	VisualDisplay(Grid * grid, string plannedRoute, vector<Location> * waypoints, int numOfWaypoints);
	Location ConvertToHamsterLocation(Location waypoint);
	void PrintWaypoints();
	void PrintRouteCvMat();
	virtual ~VisualDisplay();
};

#endif /* VISUALDISPLAY_H_ */
