/*
 * Authors: Guy Melul  - 316467737
 *          Yoni Lahav - 315025445
 */
#include <HamsterAPIClientCPP/Hamster.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <iostream>
#include <string>

#include "ConfigurationManager.h"
#include "LocalizationManager.h"
#include "WaypointsManager.h"
#include "PathPlanner.h"
#include "Map.h"
#include "Robot.h"

using namespace std;
using namespace HamsterAPI;

#define MIN_NUM_OF_MOVES 200

int main(int argc, char ** argv) {
	string windowNameLidar("LIDAR-view");
	string windowNameSLAM("SLAM-view");
	string windowNameBlow("BLOW-view");
	string windowNameCoarse("COARSE-view");
	string configPath("config.csv");

	try {
		if (ConfigurationManager::Init()) {
			cout << "Could not initialize robot configuration." << endl;
			return 1;
		}

		ConfigurationManager::PrintAll();

		Robot *hamsterRobot = new Robot(ConfigurationManager::getRobotSize());

		// Exit if init failed
		if(!hamsterRobot->Init()) {
			cout << "Could not initialize robot" << endl;
			return 1;
		}
		cout << "Robot initialized" << endl;

		// Pass resolution
		Map *envMap = new Map(hamsterRobot,ConfigurationManager::getMapResolution());

		cout << "Map initialized" << endl;

		//cv::namedWindow(windowNameLidar,cv::WINDOW_NORMAL);
		cv::namedWindow(windowNameSLAM,cv::WINDOW_NORMAL);
		//cv::namedWindow(windowNameBlow,cv::WINDOW_NORMAL);
		//cv::namedWindow(windowNameCoarse,cv::WINDOW_NORMAL);

		cv::resizeWindow(windowNameSLAM,768,768);
		//cv::resizeWindow(windowNameBlow,768,768);
		//cv::resizeWindow(windowNameCoarse,512,512);

		HamsterAPI::Hamster* hamster = hamsterRobot->getHamster();

		bool flagContinue=false;

		/* Dummy while to eliminate no messages error */
		while (!flagContinue) {
			try {
				if (hamster->isConnected()) {
					hamster->getAgentID();
					flagContinue=true;
				}
			}
			catch (const HamsterAPI::HamsterError & message_error) {
				if (message_error.error() != ERROR_CODE_NO_MESSAGES)
					HamsterAPI::Log::i("Client", message_error.what());
			}
			catch (const exception& e)
			{
				HamsterAPI::Log::i("Client", e.what());
			}
			catch (...)
			{
				cout << "Unknown exception at connection to hasmter" << endl;
			}
		}

		// Run post init
		bool isRobotPostInitDone=false;
		bool isMapPostInitDone=false;

		while(!(isRobotPostInitDone && isMapPostInitDone)) {
			try {
				if (hamster->isConnected()) {
					if(!isRobotPostInitDone) {
						hamsterRobot->postInit(ConfigurationManager::getStartPose());
						isRobotPostInitDone=true;
					}

					envMap->postInit();
					isMapPostInitDone=true;
				}
			}
			catch (const HamsterAPI::HamsterError & message_error) {
				if (message_error.error() != ERROR_CODE_NO_MESSAGES)
					HamsterAPI::Log::i("Client", message_error.what());
			}
			catch (const exception& e)
			{
				HamsterAPI::Log::i("Client", e.what());
			}
			catch (...)
			{
				cout << "Unknown exception at post init" << endl;
			}
		}

		// Init Localization manager and PathPlanner
		LocalizationManager* localizationManager = new LocalizationManager(hamster,envMap->getSLAMMap());
		
		Pose startPose = ConfigurationManager::getStartPose();
		cv::Point2f goalPoint = ConfigurationManager::getGoalPoint();
		cv::Point2i goalGridPoint;
		OccupancyGrid* coarseGrid = envMap->getCoarseMap();

		// Robot pos to corase grid pos
		cv::Point2i startPoint = Map::PosToGrid(startPose.getX(),startPose.getY(),coarseGrid->getWidth(),coarseGrid->getHeight(),coarseGrid->getResolution());

		goalGridPoint =  Map::PosToGrid(goalPoint.x,goalPoint.y,coarseGrid->getWidth(),coarseGrid->getHeight(),coarseGrid->getResolution());

		cout << "Robot path: (" << startPoint.y << "," << startPoint.x << ") -> (" << goalGridPoint.y << "," << goalGridPoint.x << ")" << endl;

		PathPlanner* pathPlanner = new PathPlanner(coarseGrid,startPoint.y,startPoint.x);
		Path* path = pathPlanner->getPath(goalGridPoint);

		if (path->empty()) {
			cout << "No path found from " << startPoint.y << "," << startPoint.x << ") to (" << goalGridPoint.y << "," << goalGridPoint.x << ")" << endl;
			return 1;
		}
		else {
			cout << "Path found" << endl;
		}

		// Waypoint
		WaypointsManager* waypointManager = new WaypointsManager(coarseGrid);
		list<cv::Point2i> waypoints = waypointManager->computeWaypoints(path);

		if (waypoints.empty()) {
			cout << "Error, Waypoints could not be decided" << endl;
			return 1;
		}
		else {
			cout << "Waypoints are set" << endl;
		}

		// Convert points to map real points
		vector<cv::Point2f> slamPath(path->size());
		slamPath.clear();

		for (Path::iterator iter = (*path).begin();iter != (*path).end();++iter) {
			Node* node = *iter;

			cv::Point2f slamPoint = Map::GridToPos((float)node->row + 0.5f,(float)node->col + 0.5f,
						coarseGrid->getWidth(), coarseGrid->getHeight(), coarseGrid->getResolution());

			//cout << "[PATH] (" << slamPoint.x << "," << slamPoint.y << ")" << endl;

			slamPath.push_back(slamPoint);
		}

		vector<cv::Point2f> slamWaypoints(waypoints.size());
		slamWaypoints.clear();

		for (list<cv::Point2i>::iterator iter = waypoints.begin();iter != waypoints.end();++iter) {

			cv::Point2f slamPoint = Map::GridToPos((float)iter->y + 0.5f,(float)iter->x + 0.5f,
						coarseGrid->getWidth(), coarseGrid->getHeight(), coarseGrid->getResolution());

			//cout << "[WPNT] (" << slamPoint.x << "," << slamPoint.y << ")" << endl;

			slamWaypoints.push_back(slamPoint);
		}

		// Initialize localization manager
		cv::Point2i robotStart = Map::PosToGrid(slamPath[0].x, slamPath[0].y, envMap->getSLAMMap()->getWidth(), envMap->getSLAMMap()->getHeight(), envMap->getSLAMMap()->getResolution());
		localizationManager->initParticles(robotStart);

		// Path walking
		int nextWaypointIndex = 0;

		// Best particle
		Particle *bestBeliefParticle;
		int moves = 0;

		// Main loop
		while (hamster->isConnected()) {
			try {
				sleep(0.3);

				localizationManager->update(hamsterRobot->getDeltaX(),hamsterRobot->getDeltaY(),hamsterRobot->getDeltaYaw());

				/*
				// After some calibration, use best particle to move
				if (moves > MIN_NUM_OF_MOVES)
				{
					bestBeliefParticle = localizationManager->getBestBeliefParticle();
					hamsterRobot->update(bestBeliefParticle->x, bestBeliefParticle->y, bestBeliefParticle->yaw);
				}
				else
				{
					hamsterRobot->update();
					++moves;
				}
				*/

				hamsterRobot->update();
				envMap->update();

				if (nextWaypointIndex < (int)slamWaypoints.size())
				{
					if(hamsterRobot->gotoPoint(slamWaypoints[nextWaypointIndex]))
					{
						cout << "Reached waypoint " << nextWaypointIndex + 1 << endl;
						nextWaypointIndex++;

						// Check if goal has been reached
						if (nextWaypointIndex >= (int)slamWaypoints.size())
						{
							cout << "Goal Reached!" << endl;
							cout << "Final Position: (X: " << hamsterRobot->getXPos() << ", Y: " << hamsterRobot->getYPos()  << ") [Yaw: " << hamsterRobot->getYaw() << "]" << endl;
							hamsterRobot->stopMoving();
						}
					}
					else {
						//cout << "Position: (X: " << hamsterRobot->getXPos() << ", Y: " << hamsterRobot->getYPos()  << ") [Yaw: " << hamsterRobot->getYaw() << "]" << endl;
					}
				}

				/*
				 * Update UI
				 */
				cv::imshow(windowNameSLAM,envMap->getSLAMMapImage(localizationManager->getParticles(),
						slamPath,
						slamWaypoints,
						cv::Point2f(startPose.getX(),startPose.getY()),
						ConfigurationManager::getGoalPoint()));

				//cv::imshow(windowNameLidar,envMap->getLidarMapImage());
				//cv::imshow(windowNameBlow,envMap->getBlowMapImage());
				//cv::imshow(windowNameCoarse,envMap->getCoarseMapImage(path,waypoints,startPoint,goalPoint));
				cv::waitKey(1);
			} catch (const HamsterAPI::HamsterError & message_error) {
				if (message_error.error() != ERROR_CODE_NO_MESSAGES)
					HamsterAPI::Log::i("Client", message_error.what());
			}
			catch (const exception& e)
			{
				HamsterAPI::Log::i("Client", e.what());
			}
			catch (...)
			{
				cout << "Unknown exception at main loop" << endl;
			}
		}

		cout << "Hamster disconnected" << endl;

		cv::destroyAllWindows();
		delete waypointManager;
		delete pathPlanner;
		delete localizationManager;
		delete envMap;
		delete hamsterRobot;
		ConfigurationManager::Destroy();

		HamsterAPI::Log::i("Client", "Shutdown complete");
	}
	catch (const exception& e) {
		HamsterAPI::Log::i("Client", e.what());
	}
	catch (const HamsterError& e) {
		HamsterAPI::Log::i("Client", e.what());
	}
	catch (...) {
		cout << "Unknown exception at main end" << endl;
	}
	return 0;
}

