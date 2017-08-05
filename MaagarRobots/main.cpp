#include "math.h"
#include "Config/ConfigurationManager.h"
#include "PathPlanner/PathPlanner.h"
#include "Models/Map.h"
#include "WaypointsManager/WaypointsManager.h"
#include "MovementManager/MovementManager.h"
#include "RobotManager/RobotManager.h"
#include "VisualDisplay/VisualDisplay.h"

int main()
{
	try
	{
		Hamster * hamster = new HamsterAPI::Hamster(1);

		sleep(3);
		OccupancyGrid occupancyGrid = hamster->getSLAMMap();

		sleep(1);
		double mapHeight = occupancyGrid.getHeight();
		double mapWidth = occupancyGrid.getWidth();
		double mapResolution = occupancyGrid.getResolution();

		ConfigurationManager configurationManager(mapHeight, mapWidth);
		Location startLocation = configurationManager.GetStartLocation();
		Location goalLocation = configurationManager.GetGoalLocation();
		int robotRadiusInCm = configurationManager.GetRobotRadiusInCm();

		Map map = Map(&occupancyGrid, robotRadiusInCm, startLocation, goalLocation, mapHeight, mapWidth);

		Grid grid = map.grid;


		RobotManager robotManager(hamster,map.inflationRadius, mapHeight, mapWidth);

		PathPlanner pathPlanner = PathPlanner(&grid);
		string plannedRoute = pathPlanner.plannedRoute;

		WayPointsManager waypointsManager;

		int numOfWaypoints = waypointsManager.CalculateWaypoints(plannedRoute, startLocation, goalLocation);
		vector<Location> waypoints = waypointsManager.waypoints;

		// Print the map including the planned route and chosen waypoints

		VisualDisplay visualDisplay = VisualDisplay(&grid, plannedRoute, &waypoints, numOfWaypoints);

		visualDisplay.PrintWaypoints();

		MovementManager movementManager(hamster, &robotManager, &visualDisplay);

		robotManager.Initialize(startLocation);

		Location currLocation;
		int waypointIndex = 0;
		Location currWaypoint, hamsterWaypoint;
		double deltaX = 0, deltaY = 0, deltaYaw = 0;


		while (hamster->isConnected())
		{
			try
			{
				while (waypointIndex < numOfWaypoints)
				{
					currLocation = robotManager.GetCurrHamsterLocation();

					currWaypoint = waypoints.at(waypointIndex);

					// Convert cv::Mat location to HamsterAPI::Hamster location
					hamsterWaypoint = visualDisplay.ConvertToHamsterLocation(currWaypoint);

					double distanceFromWaypoint =
						sqrt(pow(currLocation.x - hamsterWaypoint.x, 2) +
							 pow(currLocation.y - hamsterWaypoint.y, 2));

					bool isWaypointReached = distanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;

					if (!isWaypointReached)
					{
						movementManager.NavigateToWaypoint(&hamsterWaypoint);
					}
					else
					{
						cout << endl <<
							"Robot has arrived to waypoint x- " << hamsterWaypoint.x << "  y- " << hamsterWaypoint.y << "" << endl;
					}

					waypointIndex++;

				}

				movementManager.StopMoving();
				cout << "Robot has arrived to goal position!" << endl;

				return 0;
			}
			catch (const HamsterAPI::HamsterError & message_error)
			{
				HamsterAPI::Log::i("Client", message_error.what());
			}
		}
	}
	catch (const HamsterAPI::HamsterError & connection_error)
	{
		HamsterAPI::Log::i("Client", connection_error.what());
	}

	return 0;
}
