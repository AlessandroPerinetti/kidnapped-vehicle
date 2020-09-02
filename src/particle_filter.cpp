/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <map>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  	std::default_random_engine gen;

  	num_particles = 100;  // TODO: Set the number of particles
  
  	Particle particle;
  
  	std::normal_distribution<double> dist_x(x, std[0]);
  	std::normal_distribution<double> dist_y(y, std[1]);
  	std::normal_distribution<double> dist_theta(theta, std[2]);

  	particle.weight = 1;
    
  	for (int i = 0; i < num_particles ; ++i){			
    	particle.x = dist_x(gen);
  		particle.y = dist_y(gen);
  		particle.theta = dist_theta(gen);
    
		particles_list.push_back(particle);
    	weights_list.push_back(1);
  	}
  	is_initialized = true;
  
  	std::cout << "Initialization DONE! " << std::endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
	/**
   	* TODO: Add measurements to each particle and add random Gaussian noise.
   	* NOTE: When adding noise you may find std::normal_distribution 
    *   and std::default_random_engine useful.
   	*  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   	*  http://www.cplusplus.com/reference/random/default_random_engine/
   	*/
	std::default_random_engine gen;
  
  	for (int i=0 ; i<num_particles; ++i){
    	double x_prediction;
    	double y_prediction;
        double theta_prediction;
    	// Define the motion model equations
    	if (yaw_rate == 0) {
        	x_prediction = particles_list[i].x + velocity * delta_t * cos(particles_list[i].theta);
      		y_prediction = particles_list[i].y + velocity * delta_t * sin(particles_list[i].theta);
      		theta_prediction = particles_list[i].theta;
        }
      	else {
      		x_prediction = particles_list[i].x + velocity/yaw_rate * (sin(particles_list[i].theta + yaw_rate * delta_t) - sin(particles_list[i].theta));
      		y_prediction = particles_list[i].y + velocity/yaw_rate * (cos(particles_list[i].theta) - cos(particles_list[i].theta + yaw_rate * delta_t));
      		theta_prediction = particles_list[i].theta + yaw_rate * delta_t;
      	}
      
    	// Add random gaussian noise to the prediction measurements
      	std::normal_distribution<double> noisy_x_prediction(x_prediction, std_pos[0]);
  		std::normal_distribution<double> noisy_y_prediction(y_prediction, std_pos[1]);
 		std::normal_distribution<double> noisy_theta_prediction(theta_prediction, std_pos[2]);
      
      	particles_list[i].x = noisy_x_prediction(gen);
      	particles_list[i].y = noisy_y_prediction(gen);
      	particles_list[i].theta = noisy_theta_prediction(gen);
  	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  	/**
   	* TODO: Update the weights of each particle using a mult-variate Gaussian 
   	*   distribution. You can read more about this distribution here: 
   	*   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   	* NOTE: The observations are given in the VEHICLE'S coordinate system. 
   	*   Your particles are located according to the MAP'S coordinate system. 
   	*   You will need to transform between the two systems. Keep in mind that
  	*   this transformation requires both rotation AND translation (but no scaling).
   	*   The following is a good resource for the theory:
   	*   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   	*   and the following is a good resource for the actual equation to implement
   	*   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   	*/
  
  	// Define the dividing factor of the Multivariate Gaussian Distribution 
  	const double a = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
  
	for (int i = 0; i < num_particles; ++i){
    
    	double gaussian_distribution = 1.0;
      
    	for (int j = 0; j < observations.size(); ++j) {
        	double transformed_x_observation , transformed_y_observation;
          	transformed_x_observation = particles_list[i].x + observations[j].x * cos(particles_list[i].theta) - observations[j].y * sin(particles_list[i].theta);
          	transformed_y_observation = particles_list[i].y + observations[j].x * sin(particles_list[i].theta) + observations[j].y * cos(particles_list[i].theta);
         	std::vector<double> landmark_observation_distance_list(map_landmarks.landmark_list.size());
          
          	for(int k = 0; k< map_landmarks.landmark_list.size(); ++k){
            	// Check if the landmark is in range from the considered particle
              	bool is_in_range =  sqrt(pow(particles_list[i].x - map_landmarks.landmark_list[k].x_f ,2) + pow(particles_list[i].y - map_landmarks.landmark_list[k].y_f ,2)) <= sensor_range;
              
              	if(is_in_range){
                	//Compute the euclidean distance between the landmark position and the observed landmark position with the observed position from the i-th particle     
					landmark_observation_distance_list[k] =  sqrt(pow(transformed_x_observation - map_landmarks.landmark_list[k].x_f, 2) + pow(transformed_y_observation - map_landmarks.landmark_list[k].y_f, 2));
              	}
              	else { // if the landmark is too distant from the particle, it can be detected
                	landmark_observation_distance_list[k] = sensor_range + 1000;
              	}           
            }
          	//Extract the index corresponding to the landmark closer to the observation 
          	int closest_landmark_index = std::distance(landmark_observation_distance_list.begin(), min_element(landmark_observation_distance_list.begin(), landmark_observation_distance_list.end()));
        	
          	// Evaluate the exponent of the multivariate gaussian distribution
          	double b = pow((transformed_x_observation - map_landmarks.landmark_list[closest_landmark_index].x_f),2) / (2*pow(std_landmark[0],2)) + pow((transformed_y_observation - 						map_landmarks.landmark_list[closest_landmark_index].y_f),2) / (2*pow(std_landmark[1],2));
      		
          	gaussian_distribution *= a * exp(-b);
        }
      	particles_list[i].weight = gaussian_distribution;
      	weights_list[i] = gaussian_distribution;
    }
}

void ParticleFilter::resample() {
  	/**
   	* TODO: Resample particles with replacement with probability proportional 
   	*   to their weight. 
   	* NOTE: You may find std::discrete_distribution helpful here.
   	*   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   	*/
  
  	std::vector<Particle> resampled_particle_list(num_particles);

  	std::random_device rd;
  	std::default_random_engine gen(rd());
  	for (int i = 0; i < num_particles; ++i) {
    	std::discrete_distribution<int> index(weights_list.begin(), weights_list.end());
    	resampled_particle_list[i] = particles_list[index(gen)];
  	}
  
  	particles_list = resampled_particle_list;  
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}