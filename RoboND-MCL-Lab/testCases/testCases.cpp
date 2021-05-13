
#include "testCases.hpp"
#include "robot_mcl.hpp"

// Random Generators
std::random_device rd;
std::mt19937 gen(rd());

// Map size in meters
double robot_mcl::RobotMcl::worldSize_ = 100.0;

// Landmarks
std::vector<std::vector<double>>  robot_mcl::RobotMcl::landMark_ { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
                                                                   { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
                                                                   { 80.0, 20.0 }, { 80.0, 50.0 } };

bool checkParticlesArrays(std::vector<robot_mcl::Particle> p1,
                          std::vector<robot_mcl::Particle> p2);

double evaluation(robot_mcl::RobotMcl robot, std::vector<robot_mcl::Particle> particles);

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
    RobotMcl.move_robot(M_PI/2.0f, 10.0f);
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
    robot_mcl::RobotMcl RobotMcl;

    /*******************************************************************************
     *  1st sub-test : Set robot new position to x=30.0, y=50.0 and orientation=PI/2.0
     *  Turn clockwise by PI/2.0 and move by 15.0 meters                                                    *
     *******************************************************************************/
    RobotMcl.set_robot_pose(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move_robot(-M_PI / 2.0f, 15.0f);
    std::string answerPose(RobotMcl.read_sensors());
    std::string corrrectPose("[39.051248 39.051248 25.000000 30.413813 30.413813 46.097722 46.097722 35.000000]");
    r = corrrectPose.compare(answerPose) == 0;

    /*******************************************************************************
     * 2st sub-test :Rotate the robot by PI/2.0 and then move him forward by 10.0                                                 *
     *******************************************************************************/
    RobotMcl.move_robot(-M_PI / 2.0f, 10.0f);
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
    RobotMcl.set_robot_noise(5.0, 0.1, 5.0);

    /*******************************************************************************
     *  1st sub-test : Set robot new position to x=30.0, y=50.0 and orientation=PI/2.0
     *  Turn clockwise by PI/2.0 and move by 15.0 meters                                                    *
     *******************************************************************************/
    RobotMcl.set_robot_pose(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move_robot(-M_PI / 2.0f, 15.0f);
    RobotMcl.read_sensors();
    double answerPose(RobotMcl.measurement_prob_robot());
    r = answerPose< 0.1;

    /*******************************************************************************
     * 2st sub-test :Rotate the robot by PI/2.0 and then move him forward by 10.0                                                 *
     *******************************************************************************/
    RobotMcl.move_robot(-M_PI / 2.0f, 10.0f);
    RobotMcl.read_sensors();
    answerPose = RobotMcl.measurement_prob_robot();
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
    RobotMcl.move_robot(-M_PI / 2.0f, 15.0f);
    RobotMcl.move_robot(-M_PI / 2.0f, 10.0f);

    //Re-initialize myrobot object and Initialize a measurment vector
    RobotMcl = robot_mcl::RobotMcl();
    std::vector<double> z;
    const int n = 3;

    /*******************************************************************************
     *  Move the robot and sense_robot the environment afterwards                         *
     *******************************************************************************/
    RobotMcl.move_robot(0.1f, 5.0f);
    z = RobotMcl.sense_robot_landMark();

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    RobotMcl.Particles(n);
    RobotMcl.set_particle_noise(0.05f, 0.05f, 5.0f);
    std::vector<robot_mcl::Particle> origin_particles(RobotMcl.debug_get_particles());
    RobotMcl.move_particle(0.1f, 5.0f);
    std::vector<robot_mcl::Particle> move_particles(RobotMcl.debug_get_particles());

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    r = r && checkParticlesArrays(origin_particles, move_particles);


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
    robot_mcl::RobotMcl RobotMcl;
    RobotMcl.set_robot_noise(5.0f, 0.1f, 5.0f);
    RobotMcl.set_robot_pose(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move_robot(-M_PI / 2.0f, 15.0f);
    RobotMcl.move_robot(-M_PI / 2.0f, 10.0f);

    //Re-initialize myrobot object and Initialize a measurment vector
    RobotMcl = robot_mcl::RobotMcl();
    std::vector<double> z;
    const int n = 3;

    /*******************************************************************************
     *  Move the robot and sense_robot the environment afterwards                         *
     *******************************************************************************/
    RobotMcl.move_robot(0.1f, 5.0f);
    z = RobotMcl.sense_robot_landMark();

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    RobotMcl.Particles(n);
    std::vector<robot_mcl::Particle> origin_particles(RobotMcl.debug_get_particles());
    RobotMcl.set_particle_noise(0.05f, 0.05f, 5.0f);
    RobotMcl.move_particle(0.1f, 5.0f);
    std::vector<robot_mcl::Particle> move_particles(RobotMcl.debug_get_particles());

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    RobotMcl.measurement_prob_particle();
    std::vector<robot_mcl::Particle> weights_particles(RobotMcl.debug_get_particles());

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    r = r && !checkParticlesArrays(origin_particles, move_particles);
    r = r && checkParticlesArrays(origin_particles, weights_particles);

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
    robot_mcl::RobotMcl RobotMcl;
    RobotMcl.set_robot_noise(5.0f, 0.1f, 5.0f);
    RobotMcl.set_robot_pose(30.0f, 50.0f, M_PI / 2.0f);
    RobotMcl.move_robot(-M_PI / 2.0f, 15.0f);
    RobotMcl.move_robot(-M_PI / 2.0f, 10.0f);

    //Re-initialize myrobot object and Initialize a measurment vector
    RobotMcl = robot_mcl::RobotMcl();
    std::vector<double> z;
    const int n = 10;

    /*******************************************************************************
     *  Move the robot and sense_robot the environment afterwards                         *
     *******************************************************************************/
    RobotMcl.move_robot(0.1f, 5.0f);
    z = RobotMcl.sense_robot_landMark();

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    RobotMcl.Particles(n);
    std::vector<robot_mcl::Particle> origin_particles(RobotMcl.debug_get_particles());
    RobotMcl.set_particle_noise(0.05f, 0.05f, 5.0f);
    RobotMcl.move_particle(0.1f, 5.0f);
    std::vector<robot_mcl::Particle> move_particles(RobotMcl.debug_get_particles());

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    RobotMcl.measurement_prob_particle();
    std::vector<robot_mcl::Particle> weights_particles(RobotMcl.debug_get_particles());

    /*******************************************************************************
     * Simulate a robot motion for each of these particles                         *
     *******************************************************************************/
    RobotMcl.resampling_wheel();
    std::vector<robot_mcl::Particle> resample_particles(RobotMcl.debug_get_particles());

    /*******************************************************************************
     *  Evaluation                                                    *
     *******************************************************************************/
    r = r && !checkParticlesArrays(origin_particles, move_particles);
    r = r && !checkParticlesArrays(origin_particles, weights_particles);
    r = r && checkParticlesArrays(origin_particles, resample_particles);
    std::cout << "Evaluation = " << evaluation(RobotMcl, origin_particles) << std::endl;
    

    return r;
}

/**
 * @brief Test case 7 fullParticleFilter
 *        Test The linear update step.
 *
 */
bool fullParticleFilter(void)
{
    /*******************************************************************************
     *  Initialization                                                             *
     *******************************************************************************/
    bool r = true;
    robot_mcl::RobotMcl RobotMcl;
    RobotMcl.set_robot_noise(5.0f, 0.1f, 5.0f);
    RobotMcl.set_robot_pose(30.0f, 50.0f, M_PI / 2.0f);
    double x,y,orient,counter = 0;

    /*******************************************************************************
     *  Move the robot arround                                                     *
     *******************************************************************************/
    RobotMcl.move_robot(-M_PI / 2.0, 15.0);
    RobotMcl.move_robot(-M_PI / 2.0, 10.0);

    /*******************************************************************************
     *  Re-Initialize Robot                                                        *
     *******************************************************************************/
    RobotMcl = robot_mcl::RobotMcl();
    x=40.0, y=40.0, orient=M_PI;
    RobotMcl.robotPose_.xPos = x;
    RobotMcl.robotPose_.yPos = y;
    RobotMcl.robotPose_.orientRad = orient;

    /*******************************************************************************
     *  Create Partices                                                            *
     *******************************************************************************/
    const int n = 10;
    RobotMcl.Particles(n);
    RobotMcl.set_particle_noise(0.05f, 0.05f, 5.0f);

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

        //update Particles weight
        RobotMcl.measurement_prob_particle();

        //Resample the particles with a sample probability proportional to the importance weight
        RobotMcl.resampling_wheel();
        std::vector<robot_mcl::Particle> origin_particles(RobotMcl.debug_get_particles());

        std::cout << "Step = "<<i << ", Evaluation = " << evaluation(RobotMcl, origin_particles) << std::endl;
    }

    return r;
}

bool checkParticlesArrays(std::vector<robot_mcl::Particle> p1,
                          std::vector<robot_mcl::Particle> p2)
{
    bool answer = true;
    const int p1_size(static_cast<int>(p1.size()));
    const int p2_size(static_cast<int>(p2.size()));
    if(p1_size == p2_size)
    {
        for (int i = 0; i < p1_size; i++)
        {
            answer = answer && (p1[i].id == p2[i].id);
            answer = answer && (p1[i].orientRad == p2[i].orientRad);
            answer = answer && (p1[i].weight == p2[i].weight);
            answer = answer && (p1[i].xPos == p2[i].xPos);
            answer = answer && (p1[i].yPos == p2[i].yPos);
        }
    }
    else
    {
        answer = false;
    }
    return answer;
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


