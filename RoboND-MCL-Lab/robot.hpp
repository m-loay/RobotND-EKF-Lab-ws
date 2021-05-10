#include <iostream>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers
#include <vector>
using namespace std;
class Robot;


class Robot 
{
    private:
        //! robot pose
        double x_, y_, orient_;

        //!robot noise
        double forward_noise_, turn_noise_, sense_noise_;

        //! landmark.
        std::vector<std::vector<double>> landMark_;

        //distance to landmarks 
        std::vector<double>distancesToLandMarks_;

        //! world size.
        double worldSize_;

        /*!
        * gen_gauss_random generate random numbers based on normal disribution.
        * @param mean mean value.
        * @param variance vvariance of the mean value.
        * @return void.
        */
        double gen_gauss_random(double mean, double variance);

        /*!
        * gaussian generate gaussiam distrbition function to calculate the probability based on mu, sigma.
        * @param mu mean value.
        * @param sigma variance of the mean value.
        * @param x calculate the probablity of x, p(x).Based on mu and sigma.
        * @return double .
        */        
        double gaussian(double mu, double sigma, double x);

        /*!
        * gen_real_random // Generate real random between 0 and 1.
        * @return double.
        */
        double gen_real_random();
        
        /*!
        * mod Calculate the modulus for cyclic truncate.
        * @param first_term first position.
        * @param second_term second position.
        * @return void.
        */
        double mod(double first_term, double second_term);

    public:
        /*!
        * Constructor.
        */
        Robot();

         /*!
        * set_robot_pose set Robot Position.
        * @param new_x Robot new x-position.
        * @param new_y Robot new y-position.
        * @param new_orient Robot new orientation.
        * @return void.
        */       
        void set_robot_pose(double new_x, double new_y, double new_orient);

        /*!
        * set_robot_noise set Robot Noise.
        * @param new_forward_noise Robot new forward move noise.
        * @param new_turn_noise Robot new turn move noise.
        * @param new_sense_noise Robot senosr.
        * @return void.
        */
        void set_robot_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise);

        /*!
        * set_robot_pose set Robot Noise.
        * @param landMark Robot new forward move noise.
        * @param worldsize Robot new turn move noise.
        * @return void.
        */
        void set_robot_enviornment(std::vector<std::vector<double>> landMark, double worldsize);

        /*!
        * sense  Measure the distances from the robot toward the landmarks.
        * @return std::vector<double> snenor measurements.
        */        
        vector<double> sense();

        /*!
        * read_sensors Returns all the distances from the robot toward the landmarks.
        * @return void.
        */
        string read_sensors();

        /*!
        * measurement_prob Calculates how likely a measurement should be.
        * @param measurement measurements collected from sensor std::vector<double>.
        * @return void.
        */
        double measurement_prob(vector<double> measurement);

         /*!
        * move It moves Robot and Particles.
        * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
        * @param forward distance to be moved.
        * @return RobotMcl object.
        */       
        Robot move(double turn, double forward);

        /*!
        * show_pose Returns the robot current position and orientation in a string format.
        * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
        * @param forward distance to be moved.
        * @return RobotMcl object.
        */        
        string show_pose();

};
