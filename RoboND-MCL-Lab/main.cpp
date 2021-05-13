#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers
#include <vector>
#include"robot_mcl.hpp"

// Random Generators
std::random_device rd;
std::mt19937 gen(rd());

// Map size in meters
double robot_mcl::RobotMcl::worldSize_ = 100.0;

// Landmarks
std::vector<std::vector<double>>  robot_mcl::RobotMcl::landMark_ { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
                                                                   { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
                                                                   { 80.0, 20.0 }, { 80.0, 50.0 } };

double evaluation(robot_mcl::RobotMcl robot, std::vector<robot_mcl::Particle> particles);

int main()
{

    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    robot_mcl::RobotMcl RobotMcl;
    RobotMcl.set_robot_noise(5.0f, 0.1f, 5.0f);
    RobotMcl.set_robot_pose(30.0f, 50.0f, M_PI / 2.0f);

    /*******************************************************************************
     *  Move the robot arround                                                     *
     *******************************************************************************/
    RobotMcl.move_robot(-M_PI / 2.0, 15.0);
    RobotMcl.move_robot(-M_PI / 2.0, 10.0);

    /*******************************************************************************
     *  Create Partices                                                            *
     *******************************************************************************/
    const int n (5);
    RobotMcl.Particles(n);
    RobotMcl.set_particle_noise(0.05f, 0.05f, 5.0f);

    /*******************************************************************************
     *  Re-Initialize Robot                                                        *
     *******************************************************************************/
    RobotMcl = robot_mcl::RobotMcl();

    /*******************************************************************************
     *  Create the loo[   ]                                                        *
     *******************************************************************************/
    const int steps(50);
    for (int i = 0; i < steps; i++)
    {
        //Move the robot and sense the environment afterwards
        RobotMcl.move_robot(0.1f, 5.0f);

        // Simulate a robot motion for each of these particles
        RobotMcl.move_particle(0.1f, 5.0f);
        std::vector<robot_mcl::Particle> move_particles(RobotMcl.debug_get_particles_move());

        //update Particles weight
        RobotMcl.measurement_prob_particle();
        std::vector<robot_mcl::Particle> weights_particles(RobotMcl.debug_get_particles_weight());

        //Resample the particles with a sample probability proportional to the importance weight
        RobotMcl.resampling_wheel();
        std::vector<robot_mcl::Particle> resample_particles(RobotMcl.debug_get_particles_resample());

        //Evaluate
        std::vector<robot_mcl::Particle> origin_particles(RobotMcl.debug_get_particles());
        std::cout << "Step = " << i << ", Evaluation = " << evaluation(RobotMcl, origin_particles) << std::endl;
        
    }

    return 0;
}

double evaluation(robot_mcl::RobotMcl r, std::vector<robot_mcl::Particle> p)
{
    //Calculate the mean error of the system
    double sum (0.0);
    const int n(static_cast<int>(p.size()));
    const double world_size(robot_mcl::RobotMcl::worldSize_);

    for (int i = 0; i < n; i++) 
    {
        //the second part is because of world's cyclicity
        double dx = r.mod((p[i].xPos - r.robotPose_.xPos + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double dy = r.mod((p[i].yPos - r.robotPose_.yPos + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double err = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += err;
    }
    return sum / n;    
}