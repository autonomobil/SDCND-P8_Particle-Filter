/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;
//  http://www.cplusplus.com/reference/random/default_random_engine/
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	//   Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	//   Add random Gaussian noise to each particle.
	num_particles = 123;

	// Resize vectors
	particles.resize(num_particles);
	weights.resize(num_particles);

	// set weight init
	double w_init = 1.0/num_particles;

	//Create normal distributions for x, y and theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// Initializes particles vector with Gaussian
	for (int i = 0; i < num_particles; ++i) {
		particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = w_init;
	}

	is_initialized =  true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	// Calculate x y and yaw for each particle
	for (int i = 0; i < num_particles; i++){

		if(fabs(yaw_rate)<= 0.0001){ //ignore yaw-rate
			double vel_dt = velocity * delta_t;

			particles[i].x += vel_dt * cos(particles[i].theta) ;
			particles[i].y += vel_dt * sin(particles[i].theta) ;
		}
		else{ //consider yaw-rate
			double vel_dt = velocity / yaw_rate;
			double theta_rate =  particles[i].theta + yaw_rate * delta_t;

			particles[i].x += vel_dt * ( sin(theta_rate) - sin(particles[i].theta));
			particles[i].y += vel_dt * (-cos(theta_rate) + cos(particles[i].theta));
			particles[i].theta = theta_rate;

		}
		// Add Gaussian noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	//   Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	//   this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// 	 Update the weights of each particle using a mult-variate Gaussian distribution.

	double sig_x = std_landmark[0];
	double sig_y = std_landmark[1];
	double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
	// clears weight vector
	weights.clear();

	for (int i = 0; i < num_particles; i++){
		// init weights
		particles[i].weight = 1.0;

		double x_p = particles[i].x;
		double y_p = particles[i].y;
		double sin_the = sin(particles[i].theta);
		double cos_the = cos(particles[i].theta);

		/////////// init & set predicted as a vector LandmarkObs
		vector<LandmarkObs> predicted;

		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			// get x,y coordinates from landmarks
			float x_land = map_landmarks.landmark_list[j].x_f;
			float y_land = map_landmarks.landmark_list[j].y_f;

			// check if in range, if not -> ignore
			if (fabs(dist(x_p, y_p, x_land, y_land)) <= sensor_range) {
				// add prediction to vector
				predicted.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, x_land, y_land });
			}
		}

		/////////// init & set observations_trans as vector LandmarkObs
		vector<LandmarkObs> observations_trans;

		for (unsigned int j = 0; j < observations.size(); j++) {
			// get x,y coordinates (Particle/car based)
			float x_obs = observations[j].x;
			float y_obs = observations[j].y;

			// transform to map
			double x_trans = x_p + cos_the * x_obs - sin_the * y_obs;
			double y_trans = y_p + sin_the * x_obs + cos_the * y_obs;
			// add transform to vector
			observations_trans.push_back(LandmarkObs{observations[j].id, x_trans, y_trans});
		}


		/////////// Association and getting x,y values for weight calculation
		for (unsigned int j = 0; j < observations_trans.size(); j++) {
				// init x,y values for weight calculation
				double x_obs, y_obs, x_pre, y_pre;
				// init min_distance to largest number
				double min_distance = numeric_limits<double>::max();

				for (unsigned int p = 0; p < predicted.size(); p++) {

					double distance = dist(observations_trans[j].x, observations_trans[j].y, predicted[p].x, predicted[p].y);

					// search for min distance
					if (distance < min_distance) {
						x_obs = observations_trans[j].x;
						y_obs = observations_trans[j].y;
						x_pre = predicted[p].x;
						y_pre = predicted[p].y;

						min_distance = distance;
					}
				}

			// calculate exponent
			float exponent = (pow(x_obs - x_pre, 2) / (2 * sig_x * sig_x)) + (pow(y_obs - y_pre, 2) / (2 * sig_y * sig_y));

			// calculate weight
			particles[i].weight *= gauss_norm * exp(-exponent);
		}
		weights.push_back(particles[i].weight);
	}
}

void ParticleFilter::resample() {
	// http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	discrete_distribution<> particle_dist(weights.begin(), weights.end());

	vector<Particle> particles_resamp;

	particles_resamp.resize(num_particles);

	for (int i = 0; i < num_particles; i++) {
			particles_resamp[i] = particles[particle_dist(gen)];
	}
	particles = particles_resamp;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
