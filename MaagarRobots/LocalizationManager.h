/*
 * LocalizationManager.h
 *
 *  Created on: Jul 13, 2017
 *      Author: user
 */

#ifndef SRC_LOCALIZATIONMANAGER_H_
#define SRC_LOCALIZATIONMANAGER_H_

#include "Robot.h"
#include "Map.h"
#include "Particle.h"
#include <HamsterAPIClientCPP/Hamster.h>

using namespace std;
using namespace HamsterAPI;

#define PARTICLES_NUM 90
#define PARTICLES_REMOVE 40
#define RAND_RADIUS 50

class LocalizationManager {
private:
	vector<Particle*> m_particles;
	Hamster* m_hamster;
	OccupancyGrid* m_grid;
public:
	LocalizationManager(Hamster *hamster,OccupancyGrid* slamMap);
	~LocalizationManager();

	// Randomize a location for a particle
	void setRandomLocation(Particle *particle,cv::Point2i robotGridPos);

	// Initialize particle vector
	void initParticles(cv::Point2i robotGridPos);

	// Compute belief for a particle
	double computeBelief(Particle *particle);

	// Update particles positions (and create new ones)
	void update(double deltaX, double deltaY,double deltaYaw);

	// Return particles
	vector<Particle*> getParticles();

	// Return particle with best belief
	Particle* getBestBeliefParticle();

	// Sort the particles by belief
	// First index - Highest value
	// Last index - Lowest value
	void sortParticles();

	// Create new particles from current particles
	void createChildren();

	// Do the actual creation of the new particle
	void doChildParticle(int i);

	// Compare function between particles
	static int sortByBelief(Particle* first,Particle* second)
	{
		return (first->belief > second->belief);
	}
};

#endif /* SRC_LOCALIZATIONMANAGER_H_ */
