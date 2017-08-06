#ifndef MAP_H_
#define MAP_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include <iostream>

#include "PathPlanner.h"
#include "Particle.h"
#include "Robot.h"
using namespace std;
using namespace HamsterAPI;

#define CLR_WHITE 255
#define CLR_BLACK 0
#define CLR_GRAY_DARK 100
#define CLR_GRAY_BRIGHT 180
#define RGB_RED_INDX 2
#define RGB_GREEN_INDX 1
#define RGB_BLUE_INDX 0

class Map {
private:
    Robot* m_robot;
    OccupancyGrid* m_lidarMap;
    OccupancyGrid* m_slamMap;
    OccupancyGrid* m_blowMap;
    OccupancyGrid* m_coarseMap;
    cv::Point2i m_robotGridPos;
    cv::Size2i m_robotGridSize;
    float m_resolution;

    OccupancyGrid* getTransformSLAM();
    void clearMap(OccupancyGrid* grid);
	void dyeRange(OccupancyGrid* grid,unsigned yStart, unsigned yEnd, unsigned xStart, unsigned xEnd, HamsterAPI::Cell color);
	OccupancyGrid* computeBlowMap();
	OccupancyGrid* computeCoarseMap();
	void computeLidarMap();
	//cv::Point2i PosToGridPos(float x,float y);

public:

	Map(Robot* robot,float resolution);
	~Map();

	// X - col
	// Y - row
	static cv::Point2i PosToGrid(float x,float y,int width,int height,float mapRes)
	{
		return cv::Point2i(
					(width / 2) + (int)(x / mapRes),
					(height / 2) - (int)(y / mapRes)
				);
	}

	static cv::Point2f GridToPos(int row,int col,int width,int height,float mapRes)
	{
		return cv::Point2f(
				(col - (width / 2)) * mapRes,
				-(row - (height / 2)) * mapRes);
	}

	void update();

	void postInit();

	// Other info
	int getHeight()
	{
		return m_slamMap->getHeight();
	}

	int getWidth()
	{
		return m_slamMap->getWidth();
	}

	float getResolution()
	{
		return m_resolution;
	}

	HamsterAPI::Cell getCell(int x,int y)
	{
		return m_slamMap->getCell(x,y);
	}

	OccupancyGrid* getSLAMMap()
	{
		return m_slamMap;
	}

	OccupancyGrid* getCoarseMap()
	{
		return m_coarseMap;
	}

	// Images
    cv::Mat getLidarMapImage();
    cv::Mat getCoarseMapImage(Path* path,list<cv::Point2i> waypoints,cv::Point2i startPoint,cv::Point2i goalPoint);
    cv::Mat getSLAMMapImage(vector<Particle*> particles,vector<cv::Point2f> path,vector<cv::Point2f> waypoints,cv::Point2f startPoint,cv::Point2f goalPoint);
    cv::Mat getBlowMapImage();
};

#endif /* MAP_H_ */
