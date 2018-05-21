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

default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// adjust number of particles to gain more accurate position
	num_particles = 100;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for(int i = 0; i < num_particles; i++)
	{
		Particle random_particle;
		random_particle.id = i;
		random_particle.x = dist_x(gen);
		random_particle.y = dist_y(gen);
		random_particle.theta = dist_theta(gen);
		random_particle.weight = 1.0;

		particles.push_back(random_particle);
		weights.push_back(random_particle.weight);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	for(int i = 0; i < num_particles; i++)
	{
		// load current position
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;
		// create prediction
		double pred_x;
		double pred_y;
		double pred_theta;

		if(fabs(yaw_rate) < 0.001)
		{
			pred_x = x + velocity * cos(theta) * delta_t;
			pred_y = y + velocity * sin(theta) * delta_t;
			pred_theta = theta;
		}
		else
		{
			pred_x = x + (velocity/yaw_rate) * (sin(theta + yaw_rate*delta_t) - sin(theta));
			pred_y = y + (velocity/yaw_rate) * (cos(theta) - cos(theta + yaw_rate*delta_t));
			pred_theta = theta + yaw_rate*delta_t;
		}

		// add noise
		normal_distribution<double> noise_x(0, std_pos[0]);
		normal_distribution<double> noise_y(0, std_pos[1]);
		normal_distribution<double> noise_theta(0, std_pos[2]);

		particles[i].x = pred_x + noise_x(gen);
		particles[i].y = pred_y + noise_y(gen);
		particles[i].theta = pred_theta + noise_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for(unsigned int i = 0; i < observations.size(); i++)
	{
		// create current observation
		LandmarkObs obs = observations[i];
		// init min_dist to maximum for searching
		double min_dist = numeric_limits<double>::max();
		// init landmark id
		int closest_landmark_id = -1;

		for(unsigned int j = 0; j < predicted.size(); j++)
		{
			// current prediction
			LandmarkObs pred = predicted[j];

			double current_dist = dist(obs.x, obs.y, pred.x, pred.y);
			// find predicted measurement that nearest the current observed landmark
			if(current_dist < min_dist)
			{
				min_dist = current_dist;
				closest_landmark_id = pred.id;
			}
		}
		// set observation id to cloest predicted landmark id
		observations[i].id = closest_landmark_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	double weight_normalizer = 0.0;
	for(int i = 0; i < num_particles; i++)
	{
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;

		/***************************************************************************
		*************Transform observation from car coord to map coord.*************
		****************************************************************************/
		vector<LandmarkObs> trans_observations;
		for(unsigned int j = 0; j < observations.size(); j++)
		{
			LandmarkObs trans_obs;
			trans_obs.id = j;
			trans_obs.x = p_x + (cos(p_theta) * observations[j].x) - (sin(p_theta) * observations[j].y);
			trans_obs.y = p_y + (sin(p_theta) * observations[j].x) + (cos(p_theta) * observations[j].y);
			trans_observations.push_back(trans_obs);
		}

		/******************************************************************************
		*************Filter landmarks within sensor range of each particle*************
		*******************************************************************************/
		vector<LandmarkObs> pred_landmarks;
		for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			int lm_id = map_landmarks.landmark_list[j].id_i;
			float lm_x = map_landmarks.landmark_list[j].x_f;
			float lm_y = map_landmarks.landmark_list[j].y_f;

			// instead using distance to searching particle circular region,
			// considers rectangular region is much faster
			if(fabs(lm_x - p_x) <= sensor_range && fabs(lm_y - p_y) <= sensor_range)
			{
				pred_landmarks.push_back(LandmarkObs{ lm_id, lm_x, lm_y});
			}
		}

		/*****************************************************************************
		*****************Associate observations to predicted landmarks****************
		************************via nearest neighbor algorithm************************
		******************************************************************************/
		dataAssociation(pred_landmarks, trans_observations);

		/****************************************************************************
		**********************Calculate weight of each particle**********************
		*****************************************************************************/
		//reset weight
		particles[i].weight = 1.0;

		for(unsigned int j = 0; j < trans_observations.size(); j++)
		{
			double obs_x = trans_observations[j].x;
			double obs_y = trans_observations[j].y;
			int obs_id = trans_observations[j].id;

			//get the x,y of the prediction landmark associated with the current observation
			double pred_x, pred_y;
			for(unsigned int k = 0; k < pred_landmarks.size(); k++)
			{
				if (pred_landmarks[k].id == obs_id)
				{
					pred_x = pred_landmarks[k].x;
					pred_y = pred_landmarks[k].y;
				}
			}
			// calculate weight for this observation with multivariate Gaussian
			double sigma_x = std_landmark[0];
			double sigma_y = std_landmark[1];
			double normalizer = (1.0/(2.0 * M_PI * sigma_x * sigma_y));
			double obs_weight = normalizer * exp(-1.0 * ((pow((obs_x - pred_x), 2)/(2.0 * pow(sigma_x, 2))) + (pow((obs_y - pred_y), 2)/(2.0 * pow(sigma_y, 2)))));

			particles[i].weight *= obs_weight;

		}

		weight_normalizer += particles[i].weight;
	}

	/*****************************************************************************
	********************Normalize the weights of all particles********************
	******************************************************************************/
	// cout << "weight_normalizer" << weight_normalizer << endl;
	for(unsigned int i = 0; i < particles.size(); i++)
	{
		particles[i].weight /= weight_normalizer;
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> resampled_particles;
	// generate random starting index
	uniform_int_distribution<int> particle_index(0, num_particles - 1);
	int current_index = particle_index(gen);

	double beta = 0.0;
	double max_weight = *max_element(weights.begin(), weights.end());

	// uniform random distribution [0.0, max_weight)
	uniform_real_distribution<double> random_weight(0.0, max_weight);

	// spin the resample wheel!
	for (int i = 0; i < num_particles; i++)
	{
		beta += random_weight(gen) * 2.0;

		while (beta > weights[current_index])
		{
			beta -= weights[current_index];
			current_index = (current_index + 1) % num_particles;
		}
		resampled_particles.push_back(particles[current_index]);
	}

	particles = resampled_particles;
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
