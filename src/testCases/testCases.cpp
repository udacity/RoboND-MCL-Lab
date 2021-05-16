#include "testCases.hpp"
#include "robotMcl.hpp"

bool checkParticlesArrays(std::vector<pf::Particle> p1,
                          std::vector<pf::Particle> p2);


/**
 * @brief Test case 1 RobotClassTest
 *        Test The linear update step.
 *
 */
bool RobotClassTest(void)
{
    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    bool r= false;
    pf::Robot RobotMcl;

    /*******************************************************************************
     *  1st sub-test :Set robot new position to x=10.0, y=10.0 and orientation=0                                                        *
     *******************************************************************************/
    RobotMcl.set(10.0f, 10.0f, 0);
    std::string answerPose(RobotMcl.show_pose());
    std::string corrrectPose("[x=10.000000 y=10.000000 orient=0.000000]");
    r = corrrectPose.compare(answerPose) == 0;

    /*******************************************************************************
     * 2st sub-test :Rotate the robot by PI/2.0 and then move him forward by 10.0                                                 *
     *******************************************************************************/
    RobotMcl.move(M_PI/2.0f, 10.0f);
    answerPose = RobotMcl.show_pose();
    corrrectPose ="[x=10.000000 y=20.000000 orient=1.570796]";
    r = r && (corrrectPose.compare(answerPose) == 0);   

    /*******************************************************************************
     *3st sub-test :Printing the distance from the robot toward the eight landmarks                                                *
     *******************************************************************************/
    answerPose = RobotMcl.read_sensors();
    corrrectPose ="[10.000000 60.827625 31.622777 40.000000 72.111026 92.195445 70.000000 76.157731]";
    r = r && (corrrectPose.compare(answerPose) == 0);  

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    return r;
}

/**
 * @brief Test case 2 RobotClassTest
 *        Test The linear update step.
 *
 */
bool RobotClassTest2(void)
{
    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    bool r= false;
    pf::Robot RobotMcl;

    /*******************************************************************************
     *  1st sub-test : Set robot new position to x=30.0, y=50.0 and orientation=PI/2.0
     *  Turn clockwise by PI/2.0 and move by 15.0 meters                                                    *
     *******************************************************************************/
    RobotMcl.set(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move(-M_PI / 2.0f, 15.0f);
    std::string answerPose(RobotMcl.read_sensors());
    std::string corrrectPose("[39.051248 39.051248 25.000000 30.413813 30.413813 46.097722 46.097722 35.000000]");
    r = corrrectPose.compare(answerPose) == 0;

    /*******************************************************************************
     * 2st sub-test :Rotate the robot by PI/2.0 and then move him forward by 10.0                                                 *
     *******************************************************************************/
    RobotMcl.move(-M_PI / 2.0f, 10.0f);
    answerPose = RobotMcl.read_sensors();
    corrrectPose ="[32.015621 47.169906 26.925824 20.615528 40.311289 53.150729 40.311289 36.400549]";
    r = r && (corrrectPose.compare(answerPose) == 0);   

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    return r;
}

/**
 * @brief Test case 3 AddNoiseToMotion
 *        Test The linear update step.
 *
 */
bool AddNoiseToMotion(void)
{
    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    bool r= false;
    pf::Robot RobotMcl;
    RobotMcl.set_noise(0.05f, 0.05f, 5.0f);
    RobotMcl.set(5.0, 0.1, 5.0);

    /*******************************************************************************
     *  1st sub-test : Set robot new position to x=30.0, y=50.0 and orientation=PI/2.0
     *  Turn clockwise by PI/2.0 and move by 15.0 meters                                                    *
     *******************************************************************************/
    RobotMcl.set(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move(-M_PI / 2.0f, 15.0f);
    RobotMcl.read_sensors();
    auto z(RobotMcl.sense());
    double answerPose(RobotMcl.measurement_prob(z));
    r = answerPose< 0.1;

    /*******************************************************************************
     * 2st sub-test :Rotate the robot by PI/2.0 and then move him forward by 10.0                                                 *
     *******************************************************************************/
    RobotMcl.move(-M_PI / 2.0f, 10.0f);
    RobotMcl.read_sensors();
    z = RobotMcl.sense();
    answerPose = RobotMcl.measurement_prob(z);
    r = r && answerPose< 0.1; 

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    return r;
}

/**
 * @brief Test case 4 AddNoiseToMotion
 *        Test The linear update step.
 *
 */
bool particles(void)
{
    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    bool r= true;
    pf::Robot RobotMcl;
    RobotMcl.set(5.0f, 0.1f, 5.0f);


    /*******************************************************************************
     * create Particles and move robot with particles                        *
     *******************************************************************************/
    pf::ParticleFilter particleFilter(10);
    RobotMcl.move(0.1f, 5.0f);
    auto z = RobotMcl.sense();
    particleFilter.move(0.1f, 5.0f);
    auto pf1(particleFilter.get_particles());

    /*******************************************************************************
     * Add Noise to particle and move it                         *
     *******************************************************************************/
    particleFilter.set_noise(0.05f, 0.05f, 5.0f);
    particleFilter.move(0.1f, 5.0f);
    auto pf2(particleFilter.get_particles());

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    r = r && !checkParticlesArrays(pf1, pf2);

    return r;
}

/**
 * @brief Test case 5 updateParticlesWeights
 *        Test The linear update step.
 *
 */
bool updateParticlesWeights(void)
{
    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    bool r= true;
    pf::Robot RobotMcl;
    RobotMcl.set_noise(5.0f, 0.1f, 5.0f);
    RobotMcl.set(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move(-M_PI / 2.0f, 15.0f);
    RobotMcl.move(-M_PI / 2.0f, 10.0f);

    //Re-initialize myrobot object and Initialize a measurment vector
    RobotMcl = pf::Robot();
    std::vector<double> z;
    const int n = 3;

    /*******************************************************************************
     *  Move the robot and sense_robot the environment afterwards                         *
     *******************************************************************************/
    RobotMcl.move(0.1f, 5.0f);
    z = RobotMcl.sense();

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    pf::ParticleFilter particleFilter(n);
    auto pf1(particleFilter.get_particles());
    RobotMcl.set_noise(0.05f, 0.05f, 5.0f);
    RobotMcl.move(0.1f, 5.0f);
     auto pf2(particleFilter.get_particles());

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    RobotMcl.move(0.1f, 5.0f);
    z = RobotMcl.sense();
    RobotMcl.measurement_prob(z);
    auto pf3(particleFilter.get_particles());

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    r = r && checkParticlesArrays(pf1, pf2);
    r = r && checkParticlesArrays(pf2, pf3);

    return r;
}

/**
 * @brief Test case 6 resample_particles
 *        Test The linear update step.
 *
 */
bool resample_particles(void)
{
    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    bool r= true;
    pf::Robot RobotMcl;
    RobotMcl.set_noise(5.0f, 0.1f, 5.0f);
    RobotMcl.set(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move(-M_PI / 2.0f, 15.0f);
    RobotMcl.move(-M_PI / 2.0f, 10.0f);

    //Re-initialize myrobot object and Initialize a measurment vector
    RobotMcl = pf::Robot();
    std::vector<double> z;
    const int n = 10;

    /*******************************************************************************
     *  Move the robot and sense_robot the environment afterwards                         *
     *******************************************************************************/
    RobotMcl.move(0.1f, 5.0f);
    z = RobotMcl.sense();

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    pf::ParticleFilter particleFilter(n);
    auto pf1(particleFilter.get_particles());
    particleFilter.set_noise(0.05f, 0.05f, 5.0f);
    particleFilter.move(0.1f, 5.0f);
    auto pf2(particleFilter.get_particles());

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    z = RobotMcl.sense();
    RobotMcl.measurement_prob(z);
    auto pf3(particleFilter.get_particles());

    /*******************************************************************************
     * Simulate a robot motion for each of these particles       E                  *
     *******************************************************************************/
    particleFilter.update(z);
    auto pf4(particleFilter.get_particles());

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    r = r && !checkParticlesArrays(pf1, pf2);
    r = r && !checkParticlesArrays(pf1, pf2);
    r = r && !checkParticlesArrays(pf1, pf3);
    std::cout << "Evaluation = " << pf::evaluation(RobotMcl.get_robot_pose(), pf4,n) << std::endl;
    

    return r;
}

bool checkParticlesArrays(std::vector<pf::Particle> p1,
                          std::vector<pf::Particle> p2)
{
    bool answer = true;
    const int p1_size(static_cast<int>(p1.size()));
    const int p2_size(static_cast<int>(p2.size()));
    if(p1_size == p2_size)
    {
        for (int i = 0; i < p1_size; i++)
        {
            answer = answer && (p1[i].orient == p2[i].orient);
            answer = answer && (p1[i].x == p2[i].x);
            answer = answer && (p1[i].y == p2[i].y);
        }
    }
    else
    {
        answer = false;
    }
    return answer;
}


