/*
 * HamsterConf.h
 *
 *  Created on: Jul 13, 2017
 *      Author: user
 */

#ifndef SRC_CONFIGURATIONMANAGER_H_
#define SRC_CONFIGURATIONMANAGER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <opencv/cv.h>
#include <HamsterAPIClientCPP/Hamster.h>

using namespace HamsterAPI;
using namespace std;

class ConfigurationManager
{
public:
	static int Init()
	{
		s_instance = new ConfigurationManager(Pose(0,0,0),cv::Point2f(4,-3),cv::Size2f(0.20,0.20),0.05);

		return 0;
	}

	static int Init(std::string fileName)
	{
		ifstream file(fileName.c_str());
		if(!file.is_open()) return 1;

		string value;
		float params[8];
		int i = 0;
		while (file.good() && i < 7) {
			getline(file, value, ',');
			params[i] = atof(value.c_str());
			i++;
		}

		getline(file, value, '\n');
		params[i] = atof(value.c_str());

		file.close();

		Pose pose = Pose((float)params[0], (float)params[1], (float)params[2]);
		cv::Point2f point = cv::Point2f((float)params[3], (float)params[4]);
		cv::Size2f size = cv::Size2f((float)params[5], (float)params[6]);
		float res = (float)params[7];

		s_instance = new ConfigurationManager(pose, point, size, res);

		return 0;
	}

	static void PrintAll()
	{
		cout << "Start Position: (X: " << s_instance->m_startPose.getX() << ", Y: " << s_instance->m_startPose.getY()  << ") [Yaw: " << s_instance->m_startPose.getHeading() << "]" << endl;
		cout << "Goal Position:  (X: " << s_instance->m_goalPoint.x << ", Y: " << s_instance->m_goalPoint.y  << ")" << endl;
		cout << "Robot Size:  [" << s_instance->m_robotSize.width << "," << s_instance->m_robotSize.height  << "]" << endl;
		cout << "Map Resolution (Meters): " << s_instance->m_mapResolutionMeter << endl;
	}

	static Pose getStartPose()
	{
		return s_instance->m_startPose;
	}

	static cv::Size2f getRobotSize()
	{
		return s_instance->m_robotSize;
	}

	static cv::Point2f getGoalPoint()
	{
		return s_instance->m_goalPoint;
	}

	static float getMapResolution()
	{
		return s_instance->m_mapResolutionMeter;
	}

	static void Destroy()
	{
		if (s_instance)
		{
			delete s_instance;
		}
	}
private:
	static ConfigurationManager *s_instance;

	// Class members
	Pose m_startPose;
	cv::Point2f m_goalPoint;
	cv::Size2f m_robotSize;
	float m_mapResolutionMeter;

	ConfigurationManager(Pose startPose,cv::Point2f goalPoint,cv::Size2f robotSize,float mapResolutionMeter){
		// x, y, Yaw
		m_startPose = startPose;
		m_goalPoint = goalPoint;
		m_robotSize = robotSize;
		m_mapResolutionMeter = mapResolutionMeter;
	}
};

// Define static members
ConfigurationManager *ConfigurationManager::s_instance = 0;


#endif /* SRC_CONFIGURATIONMANAGER_H_ */
