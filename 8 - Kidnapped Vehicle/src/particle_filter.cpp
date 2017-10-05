/*
* particle_filter.cpp
*  Original Author: Tiffany Huang (Dec 12, 2016)
*/

#include <random>
#include <algorithm>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

// Engine for the creation of particles
static default_random_engine gen;

const int PARTICLE_COUNT = 60;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// NOTE: Consult particle_filter.h for more information about this method

	// Set the number of particles.
	num_particles = PARTICLE_COUNT;
	double starting_weight_ = 1.0;

	// Create normal (gaussian) distribution
	// for x, y and theta (Yaw)
	normal_distribution<double> ndist_particleX(x, std[0]);
	normal_distribution<double> ndist_particleY(y, std[1]);
	normal_distribution<double> ndist_particleTheta(theta, std[2]);

	//Initializing
	for (int i = 0; i < num_particles; i++) {
		Particle p;
		p.id = i;
		p.x = ndist_particleX(gen);
		p.y = ndist_particleY(gen);
		p.theta = ndist_particleTheta(gen);
		p.weight = starting_weight_;

		weights.push_back(starting_weight_);
		particles.push_back(p);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	// Add measurements to each particle and add random Gaussian noise.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Engine for later creation of particles
	default_random_engine gen;

	num_particles = PARTICLE_COUNT;

	bool is_heading_straight_ = false;
	if(fabs(yaw_rate) < 0.0001)
	{
		is_heading_straight_ = true;
	}
	double diff_theta_ = yaw_rate * delta_t;

	normal_distribution<double> ndist_x(0.0, std_pos[0]);
	normal_distribution<double> ndist_y(0.0, std_pos[1]);
	normal_distribution<double> ndist_theta(0.0, std_pos[2]);

	for (int i = 0;  i < num_particles; i++) {

		double theta_ =  particles[i].theta;
		// The x,y and yaw is calculated differently
		// depending on whether the yaw_rate is zero or not
		if (is_heading_straight_) {
			particles[i].x += velocity * delta_t * cos(theta_);
			particles[i].y += velocity * delta_t * sin(theta_);
			//no change in particles[i].theta since angle (yaw) is 0 when heading straight
		} else {
			particles[i].x += velocity / yaw_rate * (sin(theta_ + diff_theta_) - sin(theta_));
			particles[i].y += velocity / yaw_rate * (cos(theta_) - cos(theta_ + diff_theta_));
			particles[i].theta = theta_ + diff_theta_;
		}
		//Generate noise for particles
		double noise_x_ = ndist_x(gen);
		double noise_y_ = ndist_y(gen);
		double noise_theta_ = ndist_theta(gen);

		// Add noise to the particles
		particles[i].x += noise_x_;
		particles[i].y += noise_y_ ;
		particles[i].theta += noise_theta_ ;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed
	// measurement and assign the observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code.
	// But you will probably find it useful to implement this method
	// and use it as a helper during the updateWeights phase.
	for (int i = 0; i < observations.size(); i++) {
		// initialize landmark identifier from map to be associate with observations
		int map_ID;

		// initialize minimum distance to maximum possible value
		double min_distance = numeric_limits<double>::max();

		//Get the current observation
		LandmarkObs current_obs = observations[i];

		for (int l = 0; l < predicted.size(); l++) {
			//Get current prediction
			LandmarkObs pred_obs = predicted[l];

			//Determine distance between the current and predicted landmarks
			double current_distance = dist(current_obs.x, current_obs.y, pred_obs.x, pred_obs.y);

			// Find the predicted landmark that is closest to the current observed landmark
			if(current_distance < min_distance) {
				min_distance = current_distance;
				map_ID = l;
			}
		}
		// Set observation id to the one for the closest predicted landmark id
		observations[i].id = map_ID;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
	const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
		// Update the weights of each particle using a mult-variate Gaussian distribution. You can read
		// more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
		// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
		// according to the MAP'S coordinate system. You will need to transform between the two systems.
		// Keep in mind that this transformation requires both rotation AND translation (but no scaling).
		// The following is a good resource for the theory:
		// https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
		// and the following is a good resource for the actual equation to implement
		// (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
		// constants used later for calculating the new weights

		num_particles = PARTICLE_COUNT;

		for (int  i = 0; i < num_particles; i++) {

			double p_x = particles[i].x;
			double p_y = particles[i].y;
			double p_theta = particles[i].theta;

			vector<LandmarkObs> probable_landmarks; //Landmarks with the sensor range
			vector<LandmarkObs> transformed_observations; // map observations that are transformed

			// LOCATE LANDMARKS ON MAP THAT ARE WITHING THE SENSOR range
			// for each map landmark
			for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
				// Get id and x and y coordinates
				float lm_x = map_landmarks.landmark_list[j].x_f;
				float lm_y = map_landmarks.landmark_list[j].y_f;
				int lmap_ID = map_landmarks.landmark_list[j].id_i;

				// Add probable landmarks only if the landmarks fall within the sensor range of the particle
				double abs_x_distance_tween_landmark_and_particle = fabs(lm_x - p_x);
				double abs_y_distance_tween_landmark_and_particle = fabs(lm_y - p_y);
				//double error = sqrt(abs_x_distance_tween_landmark_and_particle * abs_x_distance_tween_landmark_and_particle
				//	+ abs_y_distance_tween_landmark_and_particle * abs_y_distance_tween_landmark_and_particle);

				//if(error < sensor_range) {
				if(abs_x_distance_tween_landmark_and_particle <= sensor_range
					&& abs_y_distance_tween_landmark_and_particle <= sensor_range) {
						//Add to the probable_landmarks vector
						/**
						LandmarkObs landmark = {
						lmap_ID,
						lm_x,
						lm_y
					};
					probable_landmarks.push_back(landmark);
					**/
					probable_landmarks.push_back(LandmarkObs { lmap_ID, lm_x, lm_y });
				}
			}

			// TRANSFORM OBSERVATIONS (PARTICLE'S VIEWPOINT) INTO MAP COORDINATES
			// Transform vehicle coordinates to map coordinates and hold it in a Vector of Landmark Observvations
			for(int j =0; j < observations.size(); j++) {
				double transformed_obs_id = observations[j].id;
				double transformed_x = p_x + observations[j].x * cos(p_theta) - observations[j].y * sin(p_theta);
				double transformed_y = p_y + observations[j].y * cos(p_theta) + observations[j].x * sin(p_theta);
				//Add to the transformed_observations vector
				/**
				LandmarkObs transformedObs = {
					transformed_obs_id,
					transformed_x,
					transformed_y
				};
				transformed_obs.push_back(transformedObs);
				**/
				transformed_observations.push_back(LandmarkObs { observations[j].id, transformed_x, transformed_y});
			}

			// ASSOCIATE PROBABLE LANDMARK (WITHIN RANGE) WITH MAP OBSERVATIONS
			dataAssociation(probable_landmarks, transformed_observations);

			// COMPARE ACTUAL OBSERVATION OF THE VEHICLE WITH THE OBSERVATION
			// OF THE PARTICLE AND ASSIGN GREATER WEIGHTS TO PROXIMATE PARTICLES
			// WHICH WILL BE USED WHEN RESAMPLING PARTICLES TO PICK HIGHER
			// PROBABLE PARTICLES (LOCATION)

			// Reinitiailize the weights
			particles[i].weight = 1.0;
			double w = 1.0; //Initialized Weight to 1 (starting weight)

			double x_diff, y_diff;
			double obs_x, obs_y, pred_x, pred_y;
			int assoc_prediction_id;

			double std_x = std_landmark[0];
			double std_y = std_landmark[1];
			//calculate normalization term
			double gauss_norm = (1/(2 * M_PI * std_x * std_y));
			double std_x_square = pow(std_x,2) * 2;
			double std_y_square = pow(std_y,2) * 2;

			for (int j = 0; j < transformed_observations.size(); j++) {
				// Placeholers Observations and Predictions

				assoc_prediction_id = transformed_observations[j].id;
				obs_x = transformed_observations[j].x;
				obs_y = transformed_observations[j].y;

				// Get x, y coordinates of prediction associate with current observations
				pred_x = probable_landmarks[assoc_prediction_id].x;
				pred_y = probable_landmarks[assoc_prediction_id].y;

				// Calculate the weights for observation with Multivariate Gaussian distribution
				x_diff = pred_x - obs_x;
				y_diff = pred_y - obs_y;

				double obs_mvgd_w = gauss_norm * exp(-(pow(x_diff,2)/(2 * std_x_square) + (pow(y_diff,2)/(2 * std_y_square))));

				// Product of obs_mgvd_w with the total observation weight
				w *= obs_mvgd_w;
			}

			particles[i].weight = w;
			weights[i] = w;
		}
}

void ParticleFilter::resample() {
		// NOTE: You may find std::discrete_distribution helpful here.
		//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

		// Vector for new particles
		vector<Particle> resampled_particles;

		//random_device rdevice;
		default_random_engine gen;
		//Using discrete distribution to identify and return particles by weight
		//Resample particles with replacement with probability proportional to their weight.
		discrete_distribution<int> disc_Idx(weights.begin(), weights.end());

		num_particles = PARTICLE_COUNT;
		double starting_weight_ = 1.0;

		for (int i = 0; i < num_particles; i++) {
			const int idx = disc_Idx(gen);
			Particle p;
			p.id = idx ;
			p.x = particles[idx].x;
			p.y = particles[idx].y;
			p.theta = particles[idx].theta;
			p.weight = starting_weight_;

			resampled_particles.push_back(p);
		}
		// Replace old particles with resampled particles
		particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

	return particle;
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
