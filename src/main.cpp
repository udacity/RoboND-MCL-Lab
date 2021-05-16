#include"robotMcl.hpp"

#ifndef USE_UNIT_TESTING
int main()
{
    pf::Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    myrobot.move(-M_PI / 2.0, 10.0);

    // Create a set of particles
    const int n = 1000;
    pf::ParticleFilter pf(n);
    pf.set_noise(0.05, 0.05, 5.0);

    //Re-initialize myrobot object and Initialize a measurment vector
    myrobot = pf::Robot();
    std::vector<double> z;

    //Iterating 50 times overW the set of particles
    int steps = 50;
    int counter = 0;
    for (int t = 0; t < steps; t++) 
    {
        //Move the robot and sense the environment afterwards
        myrobot.move(0.1, 5.0);
        z = myrobot.sense();
        auto p1(pf.get_particles());
        

        // Simulate a robot motion for each of these particles
        pf.move(0.1, 5.0);
        auto p2(pf.get_particles());

        //Generate particle weights depending on robot's measurement
        pf.update(z);
        auto p3(pf.get_particles());

        //Evaluate the Error
        std::cout << "Step = " << t << ", Evaluation = " << pf::evaluation(myrobot.get_robot_pose(), p3, n) << std::endl;

        //####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####
        pf::visualization(myrobot.get_robot_pose(), p2, p3, n, t);

    } //End of Steps loop

    return 0;
}
#endif

