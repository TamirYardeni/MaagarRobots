
#include "Map.h"

Map::Map(Robot* robot,float resolution) : m_robotGridPos(0,0),m_robotGridSize(0,0)
{
    m_robot = robot;
    m_slamMap = NULL;
    m_lidarMap = NULL;
    m_blowMap = NULL;
    m_coarseMap = NULL;
    m_resolution = resolution;
}

Map::~Map()
{
    delete m_lidarMap;
    delete m_blowMap;
    delete m_slamMap;
    delete m_coarseMap;
}

void Map::postInit()
{
	m_slamMap = getTransformSLAM();

	m_robotGridSize = cv::Size2i(ceil(m_robot->getDiameter() / m_slamMap->getResolution()),ceil(m_robot->getDiameter() / m_slamMap->getResolution()));

	m_blowMap = computeBlowMap();

	m_coarseMap = computeCoarseMap();
	/*
	m_lidarMap = new OccupancyGrid(m_slamMap->getWidth(),m_slamMap->getHeight(),m_slamMap->getResolution());*/

	cout << "Robot Size (Pixel): " << m_robotGridSize.width << endl;
	cout << "SLAM: " << m_slamMap->getResolution() << " [" << m_slamMap->getWidth() << " X " << m_slamMap->getHeight() << "]" << endl;
}

/*cv::Point2i Map::PosToGridPos(float x,float y) {
	// Pixel_Pos = (Meter_Pos/Resolution)
	return cv::Point2i(x/m_slamMap->getResolution() + m_slamMap->getWidth() / 2.0,
				   	   y/m_slamMap->getResolution() + m_slamMap->getHeight() / 2.0);
}*/

void Map::update()
{
	m_robotGridPos = Map::PosToGrid(m_robot->getXPos(),m_robot->getYPos(),m_slamMap->getWidth(),m_slamMap->getHeight(),m_slamMap->getResolution());
}

cv::Mat Map::getSLAMMapImage(vector<Particle*> particles,vector<cv::Point2f> path,vector<cv::Point2f> waypoints,cv::Point2f startPoint,cv::Point2f goalPoint) {
    int width = m_slamMap->getWidth();
	int height = m_slamMap->getHeight();
    unsigned char pixel;
	cv::Mat m = cv::Mat(width, height,CV_8UC3);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (m_slamMap->getCell(i, j) == CELL_FREE)
				pixel = CLR_WHITE;
			else if (m_slamMap->getCell(i, j) == CELL_OCCUPIED)
				pixel = CLR_BLACK;
			else
				pixel = CLR_GRAY_BRIGHT;

			m.at<cv::Vec3b>(i,j) = cv::Vec3b(pixel,pixel,pixel);
		}
    }

	// Draw X,Y axis
	//cv::line(m,cv::Point2i(m_slamMap->getWidth() / 2.0,0),cv::Point2i(m_slamMap->getWidth() / 2.0,m_slamMap->getHeight()),cv::Scalar(255,0,0),1,1,0);
	//cv::line(m,cv::Point2i(0,m_slamMap->getHeight() / 2),cv::Point2i(m_slamMap->getWidth(),m_slamMap->getHeight() / 2),cv::Scalar(255,0,0),1,1,0);

	char str[80];
	sprintf(str,"X: %.2f , Y: %.2f [ Yaw: %.2f ]",m_robot->getXPos(),m_robot->getYPos(),m_robot->getYaw());
	cv::putText(m, str, cv::Point2i(30,30),cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3, 8);

	str[0] = '\0';
	sprintf(str,"Row: %d , Col: %d",m_robotGridPos.y,m_robotGridPos.x);
	cv::putText(m, str, cv::Point2i(30,70),cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3, 8);

	// Draw path
	for(vector<cv::Point2f>::iterator iter = path.begin(); iter != path.end();++iter)
	{
		cv::Point2i point = Map::PosToGrid(iter->x,iter->y,width,height,m_resolution);
		cv::circle(m,point,1,cv::Scalar(0,0,255),1);
	}

	for (vector<cv::Point2f>::iterator iter = waypoints.begin();iter != waypoints.end();++iter)
	{
		cv::Point2i point = Map::PosToGrid(iter->x,iter->y,width,height,m_resolution);
		cv::circle(m,point,1,cv::Scalar(0,230,230),5);
		cv::circle(m,point,4,cv::Scalar(0,0,0),1);
	}

	// Draw start point
	cv::circle(m,Map::PosToGrid(startPoint.x,startPoint.y,width,height,m_resolution),1,cv::Scalar(235,0,0),5);
	cv::circle(m,Map::PosToGrid(startPoint.x,startPoint.y,width,height,m_resolution),4,cv::Scalar(150,0,0),1);

	// Draw goal point
	cv::circle(m,Map::PosToGrid(goalPoint.x,goalPoint.y,width,height,m_resolution),1,cv::Scalar(0,235,0),5);
	cv::circle(m,Map::PosToGrid(goalPoint.x,goalPoint.y,width,height,m_resolution),4,cv::Scalar(0,150,0),1);

	/* Draw robot, robot yaw, X and Y axis, robot location */
	cv::circle(m,m_robotGridPos,1,cv::Scalar(255,0,255),5);

	cv::Point2i headX_gridPos = Map::PosToGrid(m_robot->getXPos() + cos(m_robot->getYaw()*DEG2RAD),
											 m_robot->getYPos() + sin(m_robot->getYaw()*DEG2RAD)
											 ,width,height,m_resolution);

	cv::Point2i headY_gridPos = Map::PosToGrid(m_robot->getXPos() + cos(90*DEG2RAD + m_robot->getYaw()*DEG2RAD),
											 m_robot->getYPos() + sin(90*DEG2RAD + m_robot->getYaw()*DEG2RAD)
											 ,width,height,m_resolution);

	// Draw robot axis
	cv::line(m,m_robotGridPos,headX_gridPos,cv::Scalar(0,0,255),2,1,0);
	cv::line(m,m_robotGridPos,headY_gridPos,cv::Scalar(0,255,0),2,1,0);


	int startIndex = 0;
	if (particles.size() > 5)
	{
		startIndex = 5;
	}


	for (vector<Particle*>::iterator iter = (particles.begin() + startIndex);iter != particles.end();++iter)
	{
		Particle* particle = *iter;
		cv::Point2i point = Map::PosToGrid(particle->x,particle->y,width,height,m_resolution);
		cv::circle(m,point,1,cv::Scalar(0,0,240),2);
		//cv::circle(m,point,4,cv::Scalar(0,0,0),1);
	}

	for (int i=1;i < startIndex;++i)
	{
		Particle* particle = particles[i];
		cv::Point2i point = Map::PosToGrid(particle->x,particle->y,width,height,m_resolution);
		cv::circle(m,point,1,cv::Scalar(0,240,0),3);
		cv::circle(m,point,1,cv::Scalar(0,0,0),1);
	}

	if (particles.size() > 0) {
		Particle* particle = particles[0];
		cv::Point2i point = Map::PosToGrid(particle->x,particle->y,width,height,m_resolution);
		cv::circle(m,point,1,cv::Scalar(250,250,0),4);
		cv::circle(m,point,4,cv::Scalar(0,0,0),1);

		str[0] = '\0';
		sprintf(str,"[Belief] X: %.2f , Y: %.2f",particle->x,particle->y);
		cv::putText(m, str, cv::Point2i(30,110),cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar::all(255), 3, 8);
	}


	/*
	HamsterAPI::LidarScan scan = m_robot->getHamster()->getLidarScan();
	double meters_x;
	double meters_y;
	int pixels_x;
	int pixels_y;
	for(size_t i=0;i<scan.getScanSize();i++)
		{
			// Calculate Lidar Scan Position from Robot Meter Position
			// Scan_Pos = Robot_Pos + Scan_Distance*cos/sin(Heading + Scan_Angle_Increment)
			// + m_robot->getYaw()*DEG2RAD - 180*DEG2RAD
			meters_x = m_robot->getXPos() + scan.getDistance(i)*cos(scan.getScanAngleIncrement()*DEG2RAD*i + m_robot->getYaw()*DEG2RAD - 180*DEG2RAD);
			meters_y = m_robot->getYPos() + scan.getDistance(i)*sin(scan.getScanAngleIncrement()*DEG2RAD*i + m_robot->getYaw()*DEG2RAD - 180*DEG2RAD);

			// Calculate Scan Pixel Position from Scan Meter Position
			// Pixel_Pos = (Meter_Pos/Resolution) + Grid_Center
			pixels_x = (meters_x/m_slamMap->getResolution()) + m_slamMap->getWidth()/2.0;
			pixels_y = (meters_y/m_slamMap->getResolution()) + m_slamMap->getHeight()/2.0;

			cv::circle(m,cv::Point2i(pixels_y,pixels_x),1,cv::Scalar(0,0,240),2);
		}*/

	return m;
}

OccupancyGrid* Map::getTransformSLAM()
{
	const OccupancyGrid& slamMap = m_robot->getHamster()->getSLAMMap();
	int width = slamMap.getWidth();
	int height = slamMap.getHeight();
	OccupancyGrid* transformed_slam = new OccupancyGrid(width,height,m_resolution);

	// TODO: convert resolutions if needed
	// if (slamMap.getResolution() != m_resolution)

	unsigned char pixel;
	cv::Mat m = cv::Mat(width, height,CV_8UC1);
	Cell cell;
	int translate_col = 15;
	int translate_row = 56;

	cout << "	Converting SLAM to an image" << endl;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (slamMap.getCell(i, j) == CELL_FREE)
				pixel = CLR_WHITE;
			else if (slamMap.getCell(i, j) == CELL_OCCUPIED)
				pixel = CLR_BLACK;
			else
				pixel = CLR_GRAY_BRIGHT;

			m.at<unsigned char>(i,j) = pixel;
		}
	}

	cout << "	Rotating by 30 degress" << endl;
	/* Rotate by -30 deg to make slam map parallel to X axis */
	cv::Mat post_m;
	cv::Mat m_rotate = cv::getRotationMatrix2D(cv::Point2i(width / 2,height / 2),-30.0,1); // -30
	cv::warpAffine(m,
				   post_m,
				   m_rotate,
				   cv::Size2i(width,height),
				   cv::INTER_LINEAR,
				   cv::BORDER_CONSTANT,
				   cv::Scalar(CLR_GRAY_BRIGHT));

	/* Translate map to make SLAM map (0,0) at the center of the grid */

	cout << "	Translating image to center of grid" << endl;
	for (int i = height - translate_row - 1; i >= 0; i--) {
		for (int j = width - translate_col - 1; j >= 0; j--) {
			pixel = post_m.at<unsigned char>(i,j);

			// Translate
			post_m.at<unsigned char>(i+translate_row,j+translate_col) = pixel;

			// Mark as empty
			post_m.at<unsigned char>(i,j) = CLR_GRAY_BRIGHT;
		}
	}

	cout << "	Converting back to OG" << endl;
	// Copy image to OG
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (post_m.at<unsigned char>(i,j) == CLR_WHITE)
				cell = CELL_FREE;
			else if (post_m.at<unsigned char>(i,j) == CLR_BLACK)
				cell = CELL_OCCUPIED;
			else
				cell = CELL_UNKNOWN;

			transformed_slam->setCell(i,j,cell);
		}
	}

	cout << "	Done" << endl;

	return transformed_slam;
}

OccupancyGrid* Map::computeCoarseMap()
{
	// Number of rows and cols in robot size in pixels
	int rows = m_blowMap->getHeight() / m_robotGridSize.height;
	int cols = m_blowMap->getWidth() / m_robotGridSize.width;
	int gridRow;
	int gridCol;

	// res to robot size in pixels
	double resolution = m_blowMap->getResolution() * max(m_robotGridSize.width,m_robotGridSize.height);
	bool isOccupied;
	bool isFree;
	
	cout << "Coarse: [" << rows << "," << cols << "] " << resolution << endl;
	OccupancyGrid* coarseMap = new OccupancyGrid(rows,cols,resolution);

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			isOccupied = false;
			isFree = false;

			// Blow grid coordinates
			gridRow = i * m_robotGridSize.height;
			gridCol = j * m_robotGridSize.width;

			for (int k = gridRow; k < gridRow + m_robotGridSize.height && !isOccupied; k++) {
				for (int t = gridCol; t < gridCol + m_robotGridSize.width && !isOccupied; t++) {
					if (CELL_OCCUPIED == m_blowMap->getCell(k,t))
					{
						isOccupied = true;
					}
					else if (CELL_FREE == m_blowMap->getCell(k,t))
					{
						isFree = true;
					}

				}
			}

			if (isOccupied)
			{
				coarseMap->setCell(i,j,CELL_OCCUPIED);
			}
			else if (isFree)
			{
				coarseMap->setCell(i,j,CELL_FREE);
			}
			else
			{
				coarseMap->setCell(i,j,CELL_UNKNOWN);
			}
		}
	}

	return coarseMap;
}

OccupancyGrid* Map::computeBlowMap() {
	int height = m_slamMap->getHeight();
	int width = m_slamMap->getWidth();
	double resolution = m_slamMap->getResolution();
	int pixel_start_x;
	int pixel_end_x;
	int pixel_start_y;
	int pixel_end_y;
    OccupancyGrid* blowMap = new OccupancyGrid(*m_slamMap);
    cv::Size2f robotHalfSize(m_robot->getRadius(),m_robot->getRadius());
    cv::Size2i pixelRes(ceil(robotHalfSize.width / resolution),ceil(robotHalfSize.height / resolution));

    for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (m_slamMap->getCell(i, j) == CELL_OCCUPIED) {
				pixel_start_x = j - pixelRes.width;
				pixel_end_x   = j + pixelRes.width;
				pixel_start_y = i - pixelRes.height;
				pixel_end_y   = i + pixelRes.height;
				dyeRange(blowMap,pixel_start_y,pixel_end_y,pixel_start_x,pixel_end_x,CELL_OCCUPIED);
			}
    	}
    }

    return (blowMap);
}

void Map::clearMap(OccupancyGrid* grid) {
    int width = grid->getWidth();
    int height = grid->getHeight();

    for(int i=0;i < width;i++)
    {
        for(int j=0;j< height;j++)
        {
            grid->setCell(i,j,HamsterAPI::CELL_FREE);
        }
    }
}

void Map::dyeRange(OccupancyGrid* grid,unsigned yStart, unsigned yEnd, unsigned xStart, unsigned xEnd, HamsterAPI::Cell color){
	for (unsigned yDye = yStart; yDye <= yEnd; yDye++) {
		for (unsigned xDye = xStart; xDye <= xEnd; xDye++) {
				grid->setCell(yDye ,xDye,color);
		}
	}
}

void Map::computeLidarMap() {
    // Get Scan and Robot Pose
	HamsterAPI::LidarScan scan = m_robot->getHamster()->getLidarScan();
	HamsterAPI::Pose pose = m_robot->getHamster()->getPose();
    double meters_x;
    double meters_y;
    int pixels_x;
    int pixels_y;
    double curr_meters_x;
    double curr_meters_y;
    int curr_pixels_x;
    int curr_pixels_y;
    double distance_from_center;

    clearMap(m_lidarMap);

	for(size_t i=0;i<scan.getScanSize();i++)
	{
		HamsterAPI::Cell cell;

		// If scan distance is at max distance, then all the way is free
		if(fabs(scan.getDistance(i) - scan.getMaxRange()) < 0.1)
			cell = HamsterAPI::CELL_FREE;
		else
			cell = HamsterAPI::CELL_OCCUPIED;

		// Calculate Lidar Scan Position from Robot Meter Position
		// Scan_Pos = Robot_Pos + Scan_Distance*cos/sin(Heading + Scan_Angle_Increment)
		meters_x = pose.getX() + scan.getDistance(i)*cos(scan.getScanAngleIncrement()*DEG2RAD*i + pose.getHeading()*DEG2RAD - 180*DEG2RAD);
		meters_y = pose.getY() + scan.getDistance(i)*sin(scan.getScanAngleIncrement()*DEG2RAD*i + pose.getHeading()*DEG2RAD - 180*DEG2RAD);

		// Calculate Scan Pixel Position from Scan Meter Position
		// Pixel_Pos = (Meter_Pos/Resolution) + Grid_Center
		pixels_x = (meters_x/m_lidarMap->getResolution()) + m_lidarMap->getWidth()/2.0;
		pixels_y = (meters_y/m_lidarMap->getResolution()) + m_lidarMap->getHeight()/2.0;

		// Distance of scan from center of robot in pixels
	    distance_from_center = hypot(meters_x-pose.getX(), meters_y-pose.getY());

		// Mark all the line path to the end of the scan with free
		for(double d=0;d<distance_from_center;d += 0.01)
		{
			 // Current Position in Meters
			 curr_meters_x = pose.getX() + d*cos(scan.getScanAngleIncrement()*DEG2RAD*i + pose.getHeading()*DEG2RAD - 180*DEG2RAD);
			 curr_meters_y = pose.getY() + d*sin(scan.getScanAngleIncrement()*DEG2RAD*i + pose.getHeading()*DEG2RAD - 180*DEG2RAD);

			 // Transform to Pixels
			 curr_pixels_x = (curr_meters_x/m_lidarMap->getResolution()) + m_lidarMap->getWidth()/2.0;
			 curr_pixels_y = (curr_meters_y/m_lidarMap->getResolution()) + m_lidarMap->getHeight()/2.0;

			 m_lidarMap->setCell(curr_pixels_x,curr_pixels_y,HamsterAPI::CELL_FREE);
		}

		m_lidarMap->setCell(pixels_x,pixels_y,cell);
	}
}

cv::Mat Map::getBlowMapImage()
{
    int width = m_blowMap->getWidth();
	int height = m_blowMap->getHeight();
    unsigned char pixel;
	cv::Mat m = cv::Mat(width, height,CV_8UC3);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (m_blowMap->getCell(i, j) == CELL_FREE){
				pixel = CLR_WHITE;
			}
			else if (m_blowMap->getCell(i, j) == CELL_OCCUPIED) {
				pixel = CLR_BLACK;
			}
			else {
				pixel = CLR_GRAY_BRIGHT;
			}

			m.at<cv::Vec3b>(i,j) = cv::Vec3b(pixel,pixel,pixel);
		}
    }

	cv::circle(m,m_robotGridPos,7,cv::Scalar(255,0,255));
	cv::circle(m,m_robotGridPos,2,cv::Scalar(0,0,255));

	cv::circle(m,cv::Point2i(m_slamMap->getWidth() / 2.0,m_slamMap->getHeight() / 2.0),6,cv::Scalar(0,0,255));
	cv::circle(m,cv::Point2i(m_slamMap->getWidth() / 2.0,m_slamMap->getHeight() / 2.0),2,cv::Scalar(0,0,255));

    return m;
}

cv::Mat Map::getCoarseMapImage(Path* path,list<cv::Point2i> waypoints,cv::Point2i startPoint,cv::Point2i goalPoint)
{
    int width = m_coarseMap->getWidth();
	int height = m_coarseMap->getHeight();
	cv::Mat m = cv::Mat(width, height,CV_8UC3);
	unsigned char pixel;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (m_coarseMap->getCell(i, j) == CELL_FREE){
				pixel = CLR_WHITE;
			}
			else if (m_coarseMap->getCell(i, j) == CELL_OCCUPIED) {
				pixel = CLR_BLACK;
			}
			else {
				pixel= CLR_GRAY_BRIGHT;
			}

			m.at<cv::Vec3b>(i,j) = cv::Vec3b(pixel,pixel,pixel);
		}
    }

	// Draw start point
	cv::circle(m,startPoint,1,cv::Scalar(255,0,0));

	// Draw goal point
	cv::circle(m,goalPoint,1,cv::Scalar(0,255,0));

	// Draw path
	for(Path::iterator iter = path->begin(); iter != path->end();++iter)
	{
		Node* pathNode = *iter;
		m.at<cv::Vec3b>(pathNode->row,pathNode->col) = cv::Vec3b(0,0,255);
	}

	for (list<cv::Point2i>::iterator iter = waypoints.begin();iter != waypoints.end();++iter)
	{
		cv::Point2i waypoint = *iter;
		m.at<cv::Vec3b>(waypoint) = cv::Vec3b(0,255,255);
	}

	return m;
}

cv::Mat Map::getLidarMapImage()
{
    int width = m_lidarMap->getWidth();
	int height = m_lidarMap->getHeight();
    unsigned char pixel;
	cv::Mat m = cv::Mat(width, height,CV_8UC1);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (m_lidarMap->getCell(i, j) == CELL_FREE)
				pixel = CLR_BLACK;
			else if (m_lidarMap->getCell(i, j) == CELL_OCCUPIED)
				pixel = CLR_WHITE;
			else
				pixel = CLR_GRAY_BRIGHT;

			m.at<unsigned char>(i,j) = pixel;
		}
    }

    return m;
}
