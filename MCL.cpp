// To compile g++ MCL.cpp -o app -std=c++11 //
//-I/usr/include/python2.7 -lpython2.7
//Format: http://format.krzaq.cc/

#include <iostream>
#include <string>
#include <math.h>
#include <random> //C++ 11 Random

using namespace std;

// Landmarks
double landmarks[8][2] = { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 }, { 50.0, 20.0 },
    { 50.0, 80.0 }, { 80.0, 80.0 }, { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters
double world_size = 100.0;

// Random Generators
random_device rd;
mt19937 gen(rd());

// Functions
double mod(double first_term, double second_term);
double gen_real_random();

class Robot
{
public:
    Robot()
    {
        x = gen_real_random() * world_size; // robot's x coordinate
        y = gen_real_random() * world_size; // robot's y coordinate
        orient = gen_real_random() * 2.0 * M_PI; // robot's orientation

        forward_noise = 0.0; // noise of the forward movement
        turn_noise = 0.0; // noise of the turn
        sense_noise = 0.0; // noise of the sensing
    }

    void set(double new_x, double new_y, double new_orient)
    {
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

    void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        forward_noise = new_forward_noise;
        turn_noise = new_turn_noise;
        sense_noise = new_sense_noise;
    }

    double* sense()
    {
        static double z[sizeof(landmarks) / sizeof(landmarks[0])];
        double dist;

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++)
        {
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));
            dist += gen_gauss_random(0.0, sense_noise);
            z[i] = dist;
        }
        return z;
    }

    Robot move(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backwards");

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

        // set particle
        Robot res;
        res.set(x, y, orient);
        res.set_noise(forward_noise, turn_noise, sense_noise);

        return res;
    }

    string show_pose()
    {
        return "[x=" + to_string(x) + " y=" + to_string(y) + " orient=" + to_string(orient) + "]";
    }

    string read_sensors()
    {
        double* z = sense();
        string readings = "[ ";
        for (int i = 0; i < sizeof(z); i++)
        {
            readings += to_string(z[i]) + " ";
        }
        readings += "]";

        return readings;
    }

    double measurement_prob(double measurement[])
    {
        double prob = 1.0;
        double dist;

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++)
        {
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));
            prob *= gaussian(dist, sense_noise, measurement[i]);
        }

        return prob;
    }

    double x, y, orient; // robot poses
    double forward_noise, turn_noise, sense_noise; // robot noises

private:
    double gen_gauss_random(double mean, double variance)
    { // Gaussian random
        normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    double gaussian(double mu, double sigma, double x)
    {
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0)
            / sqrt(2.0 * M_PI * (pow(sigma, 2)));
    }
};

// Functions
double gen_real_random()
{ // Generate real random between 0 and 1
    uniform_real_distribution<double> real_dist(0.0, 1.0); // Real
    return real_dist(gen);
}

double mod(double first_term, double second_term)
{ // modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}

double evaluation(Robot r, Robot p[], int n)
{ // Calculate the mean error of the system
    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        // the second part is because of world's cyclicity
        double dx = mod((p[i].x - r.x + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double dy = mod((p[i].y - r.y + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double err = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += err;
    }
    return sum / n;
}
double max(double arr[], int n)
{
    double max = 0;
    for (int i = 0; i < n; i++)
    {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}

int main()
{
    // create a robot
    Robot myrobot;
    // cout << myrobot.show_pose() << endl;

    // set noise parameters
    myrobot.set_noise(5.0, 0.1, 5.0);

    // set robot's initial position and orientation
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    // cout << myrobot.show_pose() << endl;

    // clockwise turn and move
    myrobot.move(-M_PI / 2.0, 15.0);
    // cout << myrobot.show_pose() << endl;

    // Robot Sense
    // cout << myrobot.read_sensors() << endl;

    // clockwise turn again and move
    myrobot.move(-M_PI / 2., 10.);
    // cout << myrobot.show_pose() << endl;

    // Robot Sense
    // cout << myrobot.read_sensors() << endl;

    // cout << endl;
    // cout << endl;

    // create a robot for the particle filter demo
    myrobot = Robot();
    myrobot.move(0.1, 5.0);
    double* z;
    z = myrobot.sense();
    // cout << "z= " ;
    for (int i = 0; i < sizeof(z); i++)
    {
        // cout << z[i] << " ";
    }
    // cout << "\n" << "myrobot= " << myrobot.show_pose() << endl;

    // create a set of particles
    int n = 1000;
    Robot p[n];

    for (int i = 0; i < n; i++)
    {
        Robot x;
        x.set_noise(0.05, 0.05, 5.0);
        p[i] = x;
    }

    int steps = 50; // particle filter steps should be 50 ////
    for (int t = 0; t < steps; t++)
    {
        // move the robot and sense the environment after that
        myrobot = myrobot.move(0.1, 5.0);
        z = myrobot.sense();

        // now we simulate a robot motion for each of these particles
        Robot p2[n];
        for (int i = 0; i < n; i++)
        {
            p2[i] = p[i].move(0.1, 5.0);
        }
        for (int i = 0; i < n; i++)
        {
            p[i] = p2[i];
        }

        // generate particle weights depending on robot's measurement
        double w[n];
        for (int i = 0; i < n; i++)
        {
            w[i] = p[i].measurement_prob(z);
            // cout << w[i] << endl;
        }

        // resampling with a sample probability proportional to the importance
        // weight
        Robot p3[n];
        int index = gen_real_random() * n;
        // cout << index << endl;
        double beta = 0.0;
        double mw = max(w, n);
        // cout << mw;
        for (int i = 0; i < n; i++)
        {
            beta += gen_real_random() * 2.0 * mw;
            while (beta > w[index])
            {
                beta -= w[index];
                index = mod((index + 1), n);
            }
            p3[i] = p[index];
        }
        // here we get a set of co-located particles
        for (int i = 0; i < n; i++)
        {
            p[i] = p3[i];
            // cout << p[i].show_pose() << endl;
        }

        cout << "Step = " << t << ", Evaluation = " << evaluation(myrobot, p, n) << endl;

        // visualization(myrobot, t, p2, p3, w)

    } // End of Steps loop

    return 0;
}
