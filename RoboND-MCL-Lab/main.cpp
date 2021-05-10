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

// Landmarks
std::vector<std::vector<double>> landmarks { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
                           { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
                           { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters
double world_size = 100.0;


int main()
{

    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    bool r= false;
    robot_mcl::RobotMcl RobotMcl;
    RobotMcl.setDist(gen);
    RobotMcl.set_robot_enviornment(landmarks, world_size);

    /*******************************************************************************
     *  1st sub-test :Set robot new position to x=10.0, y=10.0 and orientation=0                                                        *
     *******************************************************************************/
    RobotMcl.set_robot_pose(10.0f, 10.0,0.0f);
    std::string answerPose(RobotMcl.show_pose());
    std::cout<<answerPose<<std::endl;
    /*******************************************************************************
     * 2st sub-test :Rotate the robot by PI/2.0 and then move him forward by 10.0                                                 *
     *******************************************************************************/
    RobotMcl.move(M_PI/2.0f, 10.0f);
    answerPose = RobotMcl.show_pose();
    std::cout<<answerPose<<std::endl;


    /*******************************************************************************
     *3st sub-test :Printing the distance from the robot toward the eight landmarks                                                *
     *******************************************************************************/
    answerPose = RobotMcl.read_sensors();
    std::cout<<answerPose<<std::endl;

    return 0;
}
