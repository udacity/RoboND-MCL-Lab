#ifndef ROBOT_MCL_H_
#define ROBOT_MCL_H_

#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers
#include <vector>
#include "PfConfig.h"
namespace pf
{
    using namespace std;

    struct Particle
    {
        double x;
        double y;
        double orient;
    };

    class Robot 
    {
        public:
                Robot();

                void set(double new_x, double new_y, double new_orient);

                void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise);

                vector<double> sense();

                void move(double turn, double forward);

                string show_pose();

                string read_sensors();

                double measurement_prob(vector<double> measurement);

                std::pair<double,double>  get_robot_pose(void);

        private:
                double gen_gauss_random(double mean, double variance);

                double gaussian(double mu, double sigma, double x);

                double x, y, orient;
                double forward_noise, turn_noise, sense_noise;
    };

    class ParticleFilter
    {
        public:
                ParticleFilter(int particleNumber);

                void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise);

                void move(double turn, double forward);

                std::vector<Particle> get_particles(void);

                void update(std::vector<double> &measurement);

        private:
                double gen_gauss_random(double mean, double variance);

                double gaussian(double mu, double sigma, double x);

                double max_weight(void);

                void update_weights(std::vector<double> &measurement);

                void resampling_wheel(void);

                std::vector<Particle> particles;
                std::vector<double> weights_;
                double forward_noise_, turn_noise_, sense_noise_;
                int particleNumber_;
    };
    
    // Functions
    double gen_real_random();
    double mod(double first_term, double second_term);
    double evaluation(std::pair<double,double> robotPose, std::vector<Particle> &p, int n);

#ifndef USE_UNIT_TESTING
    void visualization(std::pair<double,double> robotPose,
                        std::vector<Particle> &p, 
                        std::vector<Particle> &pr,
                        int n, int step);
#endif
}
#endif