/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 *
 * Modification
 * @author Mahadir Ahmad
 * @date Jan 9, 2021
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iterator>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    /**
     * TODO: Set the number of particles. Initialize all particles to
     *   first position (based on estimates of x, y, theta and their uncertainties
     *   from GPS) and all weights to 1.
     * TODO: Add random Gaussian noise to each particle.
     * NOTE: Consult particle_filter.h for more information about this method
     *   (and others in this file).
     */
    num_particles = 1000;  // TODO: Set the number of particles
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    for (int i = 0; i < num_particles; i++) {
        //set weight to 1 for cumulative product later
        weights.push_back(1);
        Particle particle;
        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1;
        particles.push_back(particle);
    }
    is_initialized = true;

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
    //random Gaussian noise
    normal_distribution<double> dist_x(0.0, std_pos[0]);
    normal_distribution<double> dist_y(0.0, std_pos[1]);
    normal_distribution<double> dist_theta(0.0, std_pos[2]);
    for (int i = 0; i < num_particles; i++) {
        if (fabs(yaw_rate) < 1e-7) {
            // close to zero, use the yaw_rate = 0
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
        } else {
            // yaw_rate <> 0
            particles[i].x +=
                    (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            particles[i].y +=
                    (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }

        // add the noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);

    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> sensed_landmarks,
                                     vector<LandmarkObs> &observations) {
    /**
     * TODO: Find the sensed_landmarks measurement that is closest to each
     *   observed measurement and assign the observed measurement to this
     *   particular landmark.
     * NOTE: this method will NOT be called by the grading code. But you will
     *   probably find it useful to implement this method and use it as a helper
     *   during the updateWeights phase.
     */
    for (int i = 0; i < observations.size(); i++) {
        double min_distance = 1e7;
        // init the id to first pos
        observations[i].id = sensed_landmarks[0].id;
        for (int j = 0; j < sensed_landmarks.size(); j++) {
            double distance_temp = dist(
                    observations[i].x,
                    observations[i].y,
                    sensed_landmarks[j].x,
                    sensed_landmarks[j].y
            );
            //std::cout << observations[i].x << " " << sensed_landmarks[j].x << " " << observations[i].y << " " <<  sensed_landmarks[j].y  << std::endl;
            if (distance_temp < min_distance) {
                // new least distance
                // update the min_distance to the new one
                min_distance = distance_temp;
                // set the observation landmark id to this landmark
                observations[i].id = sensed_landmarks[j].id;
            }
        }
    }

}

/**
 * Calculate the gausian multivariate probability
 * @param sig_x
 * @param sig_y
 * @param x_obs
 * @param y_obs
 * @param mu_x
 * @param mu_y
 * @return
 */
double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
    // calculate normalization term
    double gauss_norm;
    gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

    // calculate exponent
    double exponent;
    exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

    // calculate weight using normalization terms and exponent
    double weight;
    weight = gauss_norm * exp(-exponent);

    return weight;
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

    for (int i = 0; i < particles.size(); i++) {

        double x_p = particles[i].x;
        double y_p = particles[i].y;
        double theta = particles[i].theta;

        //transform the observation to map coordinate
        vector<LandmarkObs> trans_observations;
        for (int j = 0; j < observations.size(); j++) {
            double x_c = observations[j].x;
            double y_c = observations[j].y;
            double x_map = x_p + (cos(theta) * x_c) - (sin(theta) * y_c);
            double y_map = y_p + (sin(theta) * x_c) + (cos(theta) * y_c);

            LandmarkObs observation{};
            observation.x = x_map;
            observation.y = y_map;
            // set closest to first index in sensed landmarks
            observation.id = 0;
            trans_observations.push_back(observation);
        }

        // get the all the landmarks within sensor range
        // total landmarks in this case are pretty small so for now just iterate from beginning
        vector<LandmarkObs> sensed_landmarks;
        for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
            double x_f = map_landmarks.landmark_list[k].x_f;
            double y_f = map_landmarks.landmark_list[k].y_f;
            if (dist(x_f, y_f, x_p, y_p) <= sensor_range) {
                // the landmark is within the sensor range
                LandmarkObs landmark{};
                landmark.x = x_f;
                landmark.y = y_f;
                landmark.id = map_landmarks.landmark_list[k].id_i;
                sensed_landmarks.push_back(landmark);
            }
        }

        // associate each observations to the landmark by nearest neighbour
        dataAssociation(sensed_landmarks, trans_observations);
        // reset to 1
        weights[i] = 1.0;
        for (int l = 0; l < trans_observations.size(); l++) {
            // the id start with 1 and is incremental
            int map_pos = trans_observations[l].id - 1;
            weights[i] *= multiv_prob(
                    std_landmark[0],
                    std_landmark[1],
                    trans_observations[l].x,
                    trans_observations[l].y,
                    map_landmarks.landmark_list[map_pos].x_f,
                    map_landmarks.landmark_list[map_pos].y_f
            );
        }
        particles[i].weight = weights[i];

    }


}

void ParticleFilter::resample() {
    /**
     * TODO: Resample particles with replacement with probability proportional
     *   to their weight.
     * http://www.cplusplus.com/reference/random/discrete_distribution/
     */
    const int nrolls = weights.size(); // number of samples
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> samples;
    for (int i = 0; i < nrolls; i++) {
        int index = distribution(gen);
        samples.push_back(particles[index]);
    }
    particles = samples;

}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
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
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}