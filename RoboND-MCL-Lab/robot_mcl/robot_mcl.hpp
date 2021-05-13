#ifndef ROBOT_MCL_H
#define ROBOT_MCL_H
#pragma once


#define DEBUG_ROBOT_MCL

#ifdef DEBUG_ROBOT_MCL
#define private public
#include<algorithm> // for copy() and assign()
#include<iterator> // for back_inserter
#endif

#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers
#include <vector>
#include<algorithm> // for copy() and assign()
#include<iterator> // for back_inserter



namespace robot_mcl
{

    struct Particle
    {
        int id;
        double xPos;
        double yPos;
        double orientRad;
        double weight;
    };

    struct RobotPose
    {
        double xPos;
        double yPos;
        double orientRad;
    };

    struct ParticleNoise
    {
        double forward_noise;
        double turn_noise;
        double sense_noise;
    };

    struct RobotNoise
    {
        double forward_noise;
        double turn_noise;
        double sense_noise;
    };

    class RobotMcl
    {
        private:
        /*data*/

        //distance to landmarks 
        std::vector<double>weights_;

        // robot pose
        struct RobotPose robotPose_;

        // robot noise
        struct RobotNoise robotNoise_;

        //distance to landmarks 
        std::vector<Particle>robotParticles_;

        // particle noise
        struct ParticleNoise particleNoise_;

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
        double gen_real_random(void);

       /*!
        * mod max_weight find the maximum weights in particles.
        * @param particles std::vector<Particle> which contains all vectors.
        * @return max the maximum weight.
        */
        double max_weight(std::vector<Particle> &particles);


        public:
        //! world size.
        static double worldSize_;

        //! landmark.
        static std::vector<std::vector<double>> landMark_;
        /*!
        * Constructor.
        */
        RobotMcl();

        /*!
        * Destructor.
        */
        ~RobotMcl() = default;

        /*!
        * createParticles create robot particles.
        * @param numOfParticles number of particles.
        * @return void.
        */
        void Particles(int numOfParticles);

        /*!
        * set_robot_pose set Robot Position.
        * @param new_x Robot new x-position.
        * @param new_y Robot new y-position.
        * @param new_orient Robot new orientation.
        * @return void.
        */
        void set_robot_pose(double new_x, double new_y, double new_orient);

        /*!
        * set_robot_pose set Robot Noise.
        * @param new_forward_noise Robot new forward move noise.
        * @param new_turn_noise Robot new turn move noise.
        * @param new_sense_noise Robot senosr.
        * @return void.
        */
        void set_robot_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise);


        /*!
        * set_particle_noise set particle Noise.
        * @param new_forward_noise Robot new forward move noise.
        * @param new_turn_noise Robot new turn move noise.
        * @param new_sense_noise Robot senosr.
        * @return void.
        */
        void set_particle_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise);

        /*!
        * sense_robot_landMark  Measure the distances from the robot toward the landmarks.
        * @return std::vector<double> snenor measurements.
        */
        std::vector<double> sense_robot_landMark(void);

        /*!
        * sense_particles_landMark  Measure the distances from the robot toward the landmarks.
        * @return std::vector<double> sensor measurements.
        */
        std::vector<double> sense_particles_landMark(const struct Particle &p);

        /*!
        * read_sensors Returns all the distances from the robot toward the landmarks.
        * @return void.
        */
        std::string read_sensors(void);

        /*!
        * measurement_prob Calculates how likely a measurement should be.
        * @return void.
        */
        double measurement_prob_robot();

        /*!
        * measurement_prob Calculates how likely a measurement should be.
        * @return void.
        */
        void measurement_prob_particle();


        /*!
        * move_robot It moves Robot and Particles.
        * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
        * @param forward distance to be moved.
        * @return RobotMcl object.
        */
        void move_robot(double turn, double forward);

        /*!
        * move_particle It moves Robot and Particles.
        * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
        * @param forward distance to be moved.
        * @return RobotMcl object.
        */
        void move_particle(double turn, double forward);

        /*!
        * show_pose Returns the robot current position and orientation in a string format.
        * @return RobotMcl object.
        */
        std::string show_pose(void);

        /*!
        * show_pose Returns the robot current position and orientation in a string format.
        * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
        * @param forward distance to be moved.
        * @return RobotMcl object.
        */
        std::string show_pose(const struct Particle &p);

        /*!
        * resampling_wheel resample the weights of particles.
        * @return RobotMcl object.
        */
        void resampling_wheel();

        /*!
        * mod Calculate the modulus for cyclic truncate.
        * @param first_term first position.
        * @param second_term second position.
        * @return void.
        */
        double mod(double first_term, double second_term);

#ifdef DEBUG_ROBOT_MCL
        std::vector<Particle> debug_get_particles(void);
#endif

    };
}
#endif