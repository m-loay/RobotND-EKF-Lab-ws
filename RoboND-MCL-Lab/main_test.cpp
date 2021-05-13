#include <gtest/gtest.h>
#include"testCases.hpp"

// TEST(Robot,RobotClassTest)
// {
//     EXPECT_EQ(RobotClassTest(),true);
// }

// TEST(Robot,RobotClassTest2)
// {
//     EXPECT_EQ(RobotClassTest2(),true);
// }

// TEST(Robot,AddNoiseToMotion)
// {
//     EXPECT_EQ(AddNoiseToMotion(),true);
// }

// TEST(ParticlesFilter,particles)
// {
//     EXPECT_EQ(particles(),true);
// }

// TEST(ParticlesFilter,updateParticlesWeights)
// {
//     EXPECT_EQ(updateParticlesWeights(),true);
// }

// TEST(ParticlesFilter,resample_particles)
// {
//     EXPECT_EQ(resample_particles(),true);
// }

TEST(ParticlesFilter,fullParticleFilter)
{
    EXPECT_EQ(fullParticleFilter(),true);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}