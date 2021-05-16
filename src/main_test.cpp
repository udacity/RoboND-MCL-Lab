
#include <gtest/gtest.h>
#include "PfConfig.h"
#include "robotMcl.hpp"
#include "testCases.hpp"

#ifdef USE_UNIT_TESTING
TEST(Robot,RobotClassTest)
{
    EXPECT_EQ(RobotClassTest(),true);
}

TEST(Robot,RobotClassTest2)
{
    EXPECT_EQ(RobotClassTest2(),true);
}

TEST(Robot,AddNoiseToMotion)
{
    EXPECT_EQ(AddNoiseToMotion(),true);
}

TEST(ParticlesFilter,particles)
{
    EXPECT_EQ(particles(),true);
}

TEST(ParticlesFilter,updateParticlesWeights)
{
    EXPECT_EQ(updateParticlesWeights(),true);
}

TEST(ParticlesFilter,resample_particles)
{
    EXPECT_EQ(resample_particles(),true);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// TEST(ExampleTest, OnePlusOne) 
// {
//     pf::Robot RobotMcl;
//     RobotMcl.set(10,10,1);
//     EXPECT_EQ(2, 1 + 1);
// }

#endif