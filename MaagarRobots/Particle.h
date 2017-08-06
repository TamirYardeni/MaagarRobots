/*
 * Particle.h
 *
 *  Created on: Jul 13, 2017
 *      Author: user
 */

#ifndef SRC_PARTICLE_H_
#define SRC_PARTICLE_H_

struct Particle {
	float x;
	float y;
	float yaw;
	double belief;
	int i;
	int j;
};

#endif /* SRC_PARTICLE_H_ */
