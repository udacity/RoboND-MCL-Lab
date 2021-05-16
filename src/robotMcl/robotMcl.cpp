#include"robotMcl.hpp"

#ifndef USE_UNIT_TESTING
#include <matplot/matplot.h>
#endif

namespace pf
{
    // Random Generators
    random_device rd;
    mt19937 gen(rd());

    // Map size in meters
    double world_size = 100.0;

    // Landmarks
    std::vector<std::vector<double>> landMarks = { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
                                                   { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
                                                   { 80.0, 20.0 }, { 80.0, 50.0 } };

    Robot::Robot()
    {
        // Constructor
        x = gen_real_random() * world_size; // robot's x coordinate
        y = gen_real_random() * world_size; // robot's y coordinate
        orient = gen_real_random() * 2.0 * M_PI; // robot's orientation

        forward_noise = 0.0; //noise of the forward movement
        turn_noise = 0.0; //noise of the turn
        sense_noise = 0.0; //noise of the sensing
    }

    void Robot::set(double new_x, double new_y, double new_orient)
    {
        // Set robot new position and orientation
        if (new_x < 0 || new_x >= world_size)
            throw std::invalid_argument("X coordinate out of bound");
        if (new_y < 0 || new_y >= world_size)
            throw std::invalid_argument("Y coordinate out of bound");
        if (new_orient < 0 || new_orient >= 2 * M_PI)
            throw std::invalid_argument("Orientation must be in [0..2pi]");

        x = new_x;
        y = new_y;
        orient = new_orient;
    }

    void Robot::set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // Simulate noise, often useful in particle filters
        forward_noise = new_forward_noise;
        turn_noise = new_turn_noise;
        sense_noise = new_sense_noise;
    }

    vector<double> Robot::sense()
    {
        // Measure the distances from the robot toward the landmarks
        const int size(landMarks.size());
        vector<double> z(size);
        double dist;

        for (int i = 0; i < size; i++) {
            dist = sqrt(pow((x - landMarks[i][0]), 2) + pow((y - landMarks[i][1]), 2));
            dist += gen_gauss_random(0.0, sense_noise);
            z[i] = dist;
        }
        return z;
    }

    void Robot::move(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");

        // turn, and add randomness to the turning command
        orient = orient + turn + gen_gauss_random(0.0, turn_noise);
        orient = mod(orient, 2 * M_PI);

        // move, and add randomness to the motion command
        double dist = forward + gen_gauss_random(0.0, forward_noise);
        x = x + (cos(orient) * dist);
        y = y + (sin(orient) * dist);

        // cyclic truncate
        x = mod(x, world_size);
        y = mod(y, world_size);
    }

    string Robot::show_pose()
    {
        // Returns the robot current position and orientation in a string format
        return "[x=" + to_string(x) + " y=" + to_string(y) + " orient=" + to_string(orient) + "]";
    }

    string Robot::read_sensors()
    {
        // Returns all the distances from the robot toward the landmarks
        vector<double> z = sense();
        string readings = "[";
        for (int i = 0; i < z.size(); i++) {
            readings += to_string(z[i]) + " ";
        }
        readings[readings.size() - 1] = ']';

        return readings;
    }

    double Robot::measurement_prob(vector<double> measurement)
    {
        // Calculates how likely a measurement should be
        double prob = 1.0;
        double dist;
        const int szie(landMarks.size());

        for (int i = 0; i < szie; i++) 
        {
            dist = sqrt(pow((x - landMarks[i][0]), 2) + pow((y - landMarks[i][1]), 2));
            prob *= gaussian(dist, sense_noise, measurement[i]);
        }

        return prob;
    }

    std::pair<double,double>  Robot::get_robot_pose(void)
    {
        return std::make_pair(x,y);
    }

    double Robot::gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    double Robot::gaussian(double mu, double sigma, double x)
    {
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
    }



    ParticleFilter::ParticleFilter(int particleNumber)
    {
        particleNumber_ = particleNumber;
        particles.resize(particleNumber);
        weights_.resize(particleNumber);

        for(int i = 0; i< particleNumber_; i++)
        {
            particles[i].x = gen_real_random() * world_size; // robot's x coordinate
            particles[i].y = gen_real_random() * world_size; // robot's y coordinate
            particles[i].orient = gen_real_random() * 2.0 * M_PI; // robot's orientation
        }

        forward_noise_ = 0.0; //noise of the forward movement
        turn_noise_ = 0.0; //noise of the turn
        sense_noise_ = 0.0; //noise of the sensing
    }

    void ParticleFilter::set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // Simulate noise, often useful in particle filters
        forward_noise_ = new_forward_noise;
        turn_noise_ = new_turn_noise;
        sense_noise_ = new_sense_noise;
    }

    void ParticleFilter::move(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");

        double x,y,orient;
        for(int i =0 ; i<particleNumber_; i++)
        {
            x = particles[i].x;
            y = particles[i].y;
            orient = particles[i].orient;
            // turn, and add randomness to the turning command
            orient = orient + turn + gen_gauss_random(0.0, turn_noise_);
            orient = mod(orient, 2 * M_PI);

            // move, and add randomness to the motion command
            double dist = forward + gen_gauss_random(0.0, forward_noise_);
            x = x + (cos(orient) * dist);
            y = y + (sin(orient) * dist);

            // cyclic truncate
            x = mod(x, world_size);
            y = mod(y, world_size);

            //update particle
            particles[i].x = x;
            particles[i].y = y;
            particles[i].orient = orient;            
        }
    }

    std::vector<Particle> ParticleFilter::get_particles(void)
    {
        return particles;
    }

    void ParticleFilter::update(std::vector<double> &measurement)
    {
        update_weights(measurement);
        resampling_wheel();
    }


    double ParticleFilter::gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        std::normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    double ParticleFilter::gaussian(double mu, double sigma, double x)
    {
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
    }

    double ParticleFilter::max_weight(void)
    {
        // Identify the max element in an array
        double max = 0;
        for (int i = 0; i < particleNumber_; i++) 
        {
            if (weights_[i] > max)
                max = weights_[i];
        }
        return max;
    }

    void ParticleFilter::update_weights(std::vector<double> &measurement)
    {
        // Calculates how likely a measurement should be
        double dist,x,y;
        for(int j=0; j<particleNumber_; j++)
        {
            x = particles[j].x;
            y = particles[j].y;

            double prob (1.0);
            for (int i = 0; i < sizeof(landMarks) / sizeof(landMarks[0]); i++) 
            {
                dist = sqrt(pow((x - landMarks[i][0]), 2) + pow((y - landMarks[i][1]), 2));
                prob *= gaussian(dist, sense_noise_, measurement[i]);
            }
            weights_[j] = prob;
        }
    }

    double evaluation(std::pair<double,double> robotPose, std::vector<Particle> &p, int n)
    {
        double sum =0;
        // Calculate the mean error of the system
        for (int i = 0; i < n; i++) 
        {
            //the second part is because of world's cyclicity
            double dx = pf::mod((p[i].x - robotPose.first + (world_size/ 2.0)), world_size) - (world_size / 2.0);
            double dy = pf::mod((p[i].y - robotPose.second + (world_size / 2.0)), world_size) - (world_size / 2.0);
            double err = sqrt(pow(dx, 2) + pow(dy, 2));
            sum += err;
        }
        return sum / n;
    }
    
    void ParticleFilter::resampling_wheel(void)
    { 
        int index = gen_real_random() * particleNumber_;
        double beta = 0.0;
        double mw = max_weight();
        std::vector<Particle> weighted_sample(particleNumber_);
        for (int i = 0; i < particleNumber_; i++) 
        {
            beta += gen_real_random() * 2.0 * mw;
            while (beta > weights_[index]) 
            {
                beta -= weights_[index];
                index = mod((index + 1), particleNumber_);
            }

            weighted_sample[i] = particles[index];
        }
        particles = weighted_sample;
    }

    // Functions
    double gen_real_random()
    {
        // Generate real random between 0 and 1
        uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
        return real_dist(gen);
    }

    double mod(double first_term, double second_term)
    {
        // Compute the modulus
        return first_term - (second_term)*floor(first_term / (second_term));
    }

#ifndef USE_UNIT_TESTING
    void visualization(std::pair<double,double> robotPose,
                    std::vector<Particle> &p, 
                    std::vector<Particle> &pr,
                    int n, int step)
    {

        //Figure cfg
        std::string title("MCL step_"+ to_string(step));
        auto h = matplot::figure(true);
        h->title(title);
        matplot::axis({0, +100, 0, +100 });


        //Preparing the data
        std::vector<double> xp,yp,xr,yr,lx,ly;
        for(int i=0;i<n;i++)
        {
            xp.push_back(p[i].x);
            yp.push_back(p[i].y);

            xr.push_back(pr[i].x);
            yr.push_back(pr[i].y);
        }

        const int size(static_cast<int>(landMarks.size()));
        for(int i =0; i<size; i++)
        {
            lx.push_back(landMarks[i][0]);
            ly.push_back(landMarks[i][1]);
        }

        //Draw Particles in green
        matplot::plot(xp, yp, "og")->line_width(6.0).marker_size(6.0);
        matplot::hold(matplot::on);

        //Draw resampled particles in yellow
        matplot::plot(xr, yr, "oy")->line_width(6.0).marker_size(6.0);

        //Draw landmarks in red
        matplot::plot(lx, ly, "or")->line_width(15.0).marker_size(15.0);

        //Draw robot position in blue
        matplot::plot({robotPose.first} , {robotPose.second}, "*b")->line_width(20.0).marker_size(20.0);

        matplot::grid(matplot::on);
        matplot::gca()->minor_grid(true);
        matplot::hold(matplot::off);

        //Save the image and close the plot
        matplot:: save("../Images/"+ to_string(step)+ "Step.jpg");
    }
#endif
}