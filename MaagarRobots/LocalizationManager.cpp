/*
 * LocalizationManager.cpp
 *
 *  Created on: Jul 13, 2017
 *      Author: user
 */

#include "LocalizationManager.h"

LocalizationManager::LocalizationManager(Hamster *hamster,OccupancyGrid* slamMap) : m_particles(PARTICLES_NUM) {
	m_hamster = hamster;
	m_grid = slamMap;
}

LocalizationManager::~LocalizationManager() {
	// Delete all particles
	for (unsigned int i = 0;i < m_particles.size();++i)
	{
		if (m_particles[i])
		{
			delete (m_particles[i]);
			m_particles[i] = NULL;
		}
	}
}

void LocalizationManager::setRandomLocation(Particle *particle,cv::Point2i robotGridPos) {
	do {
		particle->j = robotGridPos.x + (rand() % (RAND_RADIUS*2)) - RAND_RADIUS;
		particle->i = robotGridPos.y + (rand() % (RAND_RADIUS*2)) - RAND_RADIUS;
	}
	while (m_grid->getCell(particle->i,particle->j) != CELL_FREE);

	cv::Point2f particlePos = Map::GridToPos(particle->i,particle->j,m_grid->getWidth(),m_grid->getHeight(),m_grid->getResolution());
	particle->x = particlePos.x;
	particle->y = particlePos.y;

	particle->yaw = rand() % 360;
}

void LocalizationManager::initParticles(cv::Point2i robotGridPos)
{
	for (unsigned int i = 0; i < m_particles.size(); ++i) {
		m_particles[i] = new Particle();
		setRandomLocation(m_particles[i],robotGridPos);
	}
}

double LocalizationManager::computeBelief(Particle *particle)
{
	LidarScan scan = m_hamster->getLidarScan();
	int hits = 0;
	int misses = 0;
	double obsX;
	double obsY;
	int pixelsY;
	int pixelsX;
	double angle;
	float belief;

	for (unsigned int i = 0; i < scan.getScanSize(); ++i)
	{
		angle = scan.getScanAngleIncrement() * i * DEG2RAD;

		if (scan.getDistance(i) < scan.getMaxRange() - 1)
		{
			obsX = (particle->x) + scan.getDistance(i) * cos(angle + particle->yaw * DEG2RAD - 180 * DEG2RAD);
			obsY = (particle->y) + scan.getDistance(i) * sin(angle + particle->yaw * DEG2RAD - 180 * DEG2RAD);

			cv::Point2i scanGridPos = Map::PosToGrid(obsX,obsY,m_grid->getWidth(),m_grid->getHeight(),m_grid->getResolution());
			pixelsX = scanGridPos.x;
			pixelsY = scanGridPos.y;

			if (m_grid->getCell(pixelsY,pixelsX) == CELL_OCCUPIED) {
				hits++;
			}
			else {
				misses++;
			}
		}
	}

	belief = (float)hits / (hits + misses);

	return belief;
}

void LocalizationManager::update(double deltaX, double deltaY,double deltaYaw)
{
	Particle *particle;
	for (unsigned int i = 0; i < m_particles.size(); ++i) {
		particle = m_particles[i];
		particle->x += deltaX;
		particle->y += deltaY;
		particle->yaw += deltaYaw;

		cv::Point2i particleGridPos = Map::PosToGrid(particle->x,particle->y,m_grid->getWidth(),m_grid->getHeight(),m_grid->getResolution());
		particle->i = particleGridPos.y;
		particle->j = particleGridPos.x;
		particle->belief = computeBelief(particle);
	}

	sortParticles();
	createChildren();
}

void LocalizationManager::sortParticles()
{
	// sort particles by belief
	std::sort(m_particles.begin(),m_particles.end(),LocalizationManager::sortByBelief);
}

void LocalizationManager::createChildren()
{
	for (int i = 0; i < PARTICLES_REMOVE; ++i)
	{
		doChildParticle(i);
	}
}

vector<Particle*> LocalizationManager::getParticles()
{
	return m_particles;
}

Particle* LocalizationManager::getBestBeliefParticle()
{
	sortParticles();
	return m_particles[0];
}

void LocalizationManager::doChildParticle(int i)
{
	int angle;
	int rand_distance;
	int randFather;
	Particle *childParticle = new Particle();
	Particle *fatherParticle;

	do {
		angle = rand() % 360;
		rand_distance = rand() % 10;
		randFather = rand() % (PARTICLES_NUM - PARTICLES_REMOVE);
		fatherParticle = m_particles[randFather];
		childParticle->j = (fatherParticle->j + rand_distance * cos(angle));
		childParticle->i = (fatherParticle->i + rand_distance * sin(angle));
	}
	while (m_grid->getCell(childParticle->i, childParticle->j) != CELL_FREE);

	cv::Point2f childPos = Map::GridToPos(childParticle->i,childParticle->j,m_grid->getWidth(),m_grid->getHeight(),m_grid->getResolution());
	childParticle->x = childPos.x;
	childParticle->y = childPos.y;
	childParticle->yaw = angle;
	childParticle->belief = computeBelief(childParticle);
	m_particles[m_particles.size() - PARTICLES_REMOVE + i] = childParticle;
}
