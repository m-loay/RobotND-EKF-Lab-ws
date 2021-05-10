#include "robot_mcl.hpp"  

namespace robot_mcl
{
    std::random_device rd;
    std::mt19937 gen(rd());
    /*!
    * Constructor.
    */
    RobotMcl::RobotMcl()
    {
        robotPose_.xPos = gen_real_random() * worldSize_;// robot's x coordinate
        robotPose_.yPos = gen_real_random() * worldSize_;// robot's y coordinate
        robotPose_.orientRad =gen_real_random() * 2.0f * M_PI;// robot's orientation

        robotNoise_.forward_noise = 0.0; //noise of the forward movement
        robotNoise_.turn_noise = 0.0; //noise of the turn
        robotNoise_.sense_noise = 0.0; //noise of the sensing

    }

    /*!
    * set_robot_pose set Robot Position.
    * @param new_x Robot new x-position.
    * @param new_y Robot new y-position.
    * @param new_orient Robot new orientation.
    * @return void.
    */
    void RobotMcl::set_robot_pose(double new_x, double new_y, double new_orient)
    {
                // Set robot new position and orientation
        if (new_x < 0 || new_x >= worldSize_) throw std::invalid_argument("X coordinate out of bound");
        if (new_y < 0 || new_y >= worldSize_) throw std::invalid_argument("Y coordinate out of bound");
        if (new_orient < 0 || new_orient >= 2 * M_PI) throw std::invalid_argument("Orientation must be in [0..2pi]");

        robotPose_.xPos = new_x;
        robotPose_.yPos = new_y;
        robotPose_.orientRad = new_orient;
    }

    /*!
    * set_robot_pose set Robot Noise.
    * @param new_forward_noise Robot new forward move noise.
    * @param new_turn_noise Robot new turn move noise.
    * @param new_sense_noise Robot senosr.
    * @return void.
    */
    void RobotMcl::set_robot_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // Simulate noise, often useful in particle filters
        robotNoise_.forward_noise = new_forward_noise;
        robotNoise_.turn_noise = new_turn_noise;
        robotNoise_.sense_noise = new_sense_noise;
    }

    /*!
    * sense  Measure the distances from the robot toward the landmarks.
    * @return std::vector<double> sensor measurements.
    */
    std::vector<double> RobotMcl::sense(void)
    {
        // Measure the distances from the robot toward the landmarks
        const int size(landMark_.size());
        std::vector<double> z(size);
        double dist;

        for (int i = 0; i < size; i++) 
        {
            dist = sqrt(pow((robotPose_.xPos - landMark_[i][0]), 2.0f) + pow((robotPose_.yPos - landMark_[i][1]), 2.0f));
            dist += gen_gauss_random(0.0, robotNoise_.sense_noise);
            z[i] = dist;
        }
        return z;
    }

    /*!
    * read_sensors Returns all the distances from the robot toward the landmarks.
    * @return std::string sensor readings.
    */
    std::string RobotMcl::read_sensors(void)
    {
            // Returns all the distances from the robot toward the landmarks
        distancesToLandMarks_ = sense();
        std::string readings = "[";
        for (int i = 0; i < distancesToLandMarks_.size(); i++) 
        {
            readings += std::to_string(distancesToLandMarks_[i]) + " ";
        }
        readings[readings.size() - 1] = ']';

        return readings;           
    }

    /*!
    * measurement_prob Calculates how likely a measurement should be.
    * @param measurement measurements collected from sensor std::vector<double>.
    * @return double.
    */
    double RobotMcl::measurement_prob()
    {
        // Calculates how likely a measurement should be
        double prob = 1.0;
        double dist;
        const int size(landMark_.size());

        for (int i = 0; i < size ; i++) 
        {
            dist = sqrt(pow((robotPose_.xPos - landMark_[i][0]), 2.0f) + pow((robotPose_.yPos - landMark_[i][1]), 2.0f));
            prob *= gaussian(dist, robotNoise_.sense_noise, distancesToLandMarks_[i]);
        }

        return prob;    
    }


    /*!
    * move It moves Robot and Particles.
    * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
    * @param forward distance to be moved.
    * @return RobotMcl object.
    */
    RobotMcl RobotMcl::move(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");

        // turn, and add randomness to the turning command
        robotPose_.orientRad = robotPose_.orientRad + turn + gen_gauss_random(0.0, robotNoise_.turn_noise);
        robotPose_.orientRad = mod(robotPose_.orientRad, 2 * M_PI);

        // move, and add randomness to the motion command
        double dist = forward + gen_gauss_random(0.0,  robotNoise_.forward_noise);
        robotPose_.xPos = robotPose_.xPos + (cos(robotPose_.orientRad) * dist);
        robotPose_.yPos = robotPose_.yPos + (sin(robotPose_.orientRad) * dist);

        // cyclic truncate
        robotPose_.xPos = mod(robotPose_.xPos, worldSize_);
        robotPose_.yPos = mod(robotPose_.yPos, worldSize_);

        // set particle
        RobotMcl res;
        res.set_robot_pose(robotPose_.xPos, robotPose_.yPos, robotPose_.orientRad);
        res.set_robot_noise(robotNoise_.forward_noise, robotNoise_.turn_noise, robotNoise_.sense_noise);

        return res;
    }

    /*!
    * show_pose Returns the robot current position and orientation in a string format.
    * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
    * @param forward distance to be moved.
    * @return RobotMcl object.
    */
    std::string RobotMcl::show_pose(void)
    {
        // Returns the robot current position and orientation in a string format
        return "[x=" + std::to_string(robotPose_.xPos) + " y=" + std::to_string(robotPose_.yPos) +
                    " orient=" + std::to_string(robotPose_.orientRad) + "]";           
    }

    /*!
    * gen_gauss_random generate random numbers based on normal disribution.
    * @param mean mean value.
    * @param variance vvariance of the mean value.
    * @return void.
    */
    double RobotMcl::gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        std::normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    /*!
    * gaussian generate gaussiam distrbition function to calculate the probability based on mu, sigma..
    * @param mu mean value.
    * @param sigma variance of the mean value.
    * @param x calculate the probablity of x, p(x).Based on mu and sigma.
    * @return void.
    */
    double RobotMcl::gaussian (double mu, double sigma, double x)
    {
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));            
    }

    /*!
    * gen_real_random // Generate real random between 0 and 1.
    * @return double.
    */
    double RobotMcl::gen_real_random(void)
    {
        // Generate real random between 0 and 1
        std::uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
        return real_dist(gen);
    }


    /*!
    * mod Calculate the modulus for cyclic truncate.
    * @param first_term first position.
    * @param second_term second position.
    * @return void.
    */
    //double RobotMCL::mod(double first_term, double second_term)
    double RobotMcl::mod(double first_term, double second_term)
    {
        // Compute the modulus
        return first_term - (second_term)*floor(first_term / (second_term));
    }
}