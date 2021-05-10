
#include "testCases.hpp"
#include "robot_mcl.hpp"
// Landmarks
std::vector<std::vector<double>> landmarks { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
                        { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
                        { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters
double world_size = 100.0;

// Random Generators
std::random_device rd;
std::mt19937 gen(rd());
double robot_mcl::RobotMcl::worldSize_ = world_size;
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
    robot_mcl::RobotMcl RobotMcl;

    /*******************************************************************************
     *  1st sub-test :Set robot new position to x=10.0, y=10.0 and orientation=0                                                        *
     *******************************************************************************/
    RobotMcl.set_robot_pose(10.0f, 10.0f, 0);
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
    std::cout<<answerPose<<std::endl;
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
    robot_mcl::RobotMcl RobotMcl;

    /*******************************************************************************
     *  1st sub-test : Set robot new position to x=30.0, y=50.0 and orientation=PI/2.0
     *  Turn clockwise by PI/2.0 and move by 15.0 meters                                                    *
     *******************************************************************************/
    RobotMcl.set_robot_pose(30.0f, 50.0f, M_PI / 2.0f);
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
    robot_mcl::RobotMcl RobotMcl;

    /*******************************************************************************
     *  1st sub-test : Set robot new position to x=30.0, y=50.0 and orientation=PI/2.0
     *  Turn clockwise by PI/2.0 and move by 15.0 meters                                                    *
     *******************************************************************************/
    RobotMcl.set_robot_pose(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move(-M_PI / 2.0f, 15.0f);
    RobotMcl.read_sensors();
    double answerPose(RobotMcl.measurement_prob());
    r = answerPose< 0.1;

    /*******************************************************************************
     * 2st sub-test :Rotate the robot by PI/2.0 and then move him forward by 10.0                                                 *
     *******************************************************************************/
    RobotMcl.move(-M_PI / 2.0f, 10.0f);
    RobotMcl.read_sensors();
    answerPose = RobotMcl.measurement_prob();
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
    robot_mcl::RobotMcl RobotMcl;
    RobotMcl.set_robot_noise(5.0f, 0.1f, 5.0f);
    RobotMcl.set_robot_pose(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move(-M_PI / 2.0f, 15.0f);
    RobotMcl.move(-M_PI / 2.0f, 10.0f);

    /*******************************************************************************
     *  create particles                                                   *
     *******************************************************************************/
    const int n = 10;
    robot_mcl::RobotMcl p[n];
    for (int i = 0; i < n; i++)
    {
        p[i].set_robot_noise(0.05,0.05,5.0);
        std::cout<<p[i].show_pose()<<std::endl;
    }

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    return r;
}