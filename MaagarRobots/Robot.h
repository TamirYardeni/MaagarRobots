
#ifndef ROBOT_H_
#define ROBOT_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include <iostream>
using namespace std;
using namespace HamsterAPI;

class Robot {
private:
    HamsterAPI::Hamster * m_hamster;
	HamsterAPI::Pose m_currPos;
	HamsterAPI::Pose m_lastPos;
	HamsterAPI::Pose m_deltaPos;
    cv::Size2f m_size;
    float m_radius;
    float m_diameter;
    Pose m_startPose;
public:

    // ctor
	Robot(cv::Size2f robotSize);

    // Release hamster instance
	~Robot();

    /*
    Inits the hamster instance
    return 0 on success, otherwise fail
    */
    int Init();

    void postInit(const Pose startPose);

    void update();

    void update(float x, float y, float heading);

    HamsterAPI::Pose getPose()
    {
    	return m_currPos;
    }

    float getDeltaYaw()
    {
    	return m_deltaPos.getHeading();
    }

    float getDeltaY()
    {
    	return m_deltaPos.getY();
    }

    float getDeltaX()
    {
    	return m_deltaPos.getX();
    }

    // Head angle (YAW)
    float getYaw()
    {
    	return m_currPos.getHeading();
    };

    float getXPos() {
        return m_currPos.getX();
    }

    float getYPos() {
        return m_currPos.getY();
    }

    HamsterAPI::Hamster* getHamster() {
        return m_hamster;
    }

    cv::Size2f getSize() {
    	return m_size;
    }

    float getRadius()
    {
    	return m_radius;
    }

    float getDiameter()
    {
    	return m_diameter;
    }

    bool gotoPoint(cv::Point2f goal);

    void wander();

    void getScansBetween(double min, double max, std::vector<double> & distances);

    bool willCollide(std::vector<double> distances, int angle_from_center);

	bool isRightFree();

	bool isLeftFree();

	bool isFrontFree();

    bool isBackFree();

    void moveForward(float angle = 0.0);

    void turnLeft(float angle = 45.0);

    void turnRight(float angle = -45.0);

    void moveBackwards(float angle = 0.0);

    void stopMoving();
};

#endif /* ROBOT_H_ */
