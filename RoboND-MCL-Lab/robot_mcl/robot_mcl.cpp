#include "robot_mcl.hpp"  
#include <algorithm>
#include <iterator>
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
    * Particles create robot particles.
    * @param numOfParticles number of particles.
    * @return void.
    */
    void RobotMcl::Particles(int numOfParticles)
    {
        robotParticles_.clear();
        robotParticles_.empty();
        for (int i = 0; i < numOfParticles; i++)
        {
            Particle particle;
            particle.id = i;
            particle.xPos = gen_real_random() * worldSize_;// robot's x coordinate
            particle.yPos = gen_real_random() * worldSize_;// robot's y coordinate
            particle.orientRad =gen_real_random() * 2.0f * M_PI;// robot's orientation

            particleNoise_.forward_noise = 0.0; //noise of the forward movement
            particleNoise_.turn_noise = 0.0; //noise of the turn
            particleNoise_.sense_noise = 0.0; //noise of the sensing
            robotParticles_.push_back(particle);
        }
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
    * set_particle_noise set particle Noise.
    * @param new_forward_noise Robot new forward move noise.
    * @param new_turn_noise Robot new turn move noise.
    * @param new_sense_noise Robot senosr.
    * @return void.
    */
    void RobotMcl::set_particle_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // Simulate noise, often useful in particle filters
        particleNoise_.forward_noise = new_forward_noise;
        particleNoise_.turn_noise = new_turn_noise;
        particleNoise_.sense_noise = new_sense_noise;
    }

    /*!
    * sense_robot_landMark  Measure the distances from the robot toward the landmarks.
    * @return std::vector<double> sensor measurements.
    */
    std::vector<double> RobotMcl::sense_robot_landMark(void)
    {
        //initialize
        const double x(robotPose_.xPos);
        const double y(robotPose_.yPos);
        const double sensorNoise(robotNoise_.sense_noise);

        // Measure the distances from the robot toward the landmarks
        const int size(landMark_.size());
        std::vector<double> z(size);
        double dist;

        for (int i = 0; i < size; i++) 
        {
            dist = sqrt(pow((x - landMark_[i][0]), 2.0f) + pow((y - landMark_[i][1]), 2.0f));
            dist += gen_gauss_random(0.0, sensorNoise);
            z[i] = dist;
        }
        return z;
    }

    /*!
    * sense_particles_landMark  Measure the distances from the robot toward the landmarks.
    * @param index particle index.
    * @return std::vector<double> sensor measurements.
    */
    std::vector<double> RobotMcl::sense_particles_landMark(const struct Particle &p)
    {
        //initialize
        double x(p.xPos);
        double y(p.yPos);
        const double sensorNoise(particleNoise_.sense_noise);

        // Measure the distances from the robot toward the landmarks
        const int size(landMark_.size());
        std::vector<double> z(size);
        double dist;

        for (int i = 0; i < size; i++) 
        {
            dist = sqrt(pow((x - landMark_[i][0]), 2.0f) + pow((y - landMark_[i][1]), 2.0f));
            dist += gen_gauss_random(0.0, sensorNoise);
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
        const std::vector<double>distancesToLandMarks (sense_robot_landMark());

        std::string readings = "[";
        for (int i = 0; i < distancesToLandMarks.size(); i++) 
        {
            readings += std::to_string(distancesToLandMarks[i]) + " ";
        }
        readings[readings.size() - 1] = ']';

        return readings;           
    }

    /*!
    * measurement_prob Calculates how likely a measurement should be.
    * @return double.
    */
    double RobotMcl::measurement_prob_robot()
    {
        //initialize
        const double x(robotPose_.xPos);
        const double y(robotPose_.yPos);
        const double sensorNoise(robotNoise_.sense_noise);
        double prob = 1.0;
        double dist;
        const int size(landMark_.size());

        // Calculates how likely a measurement should be
        const std::vector<double>distancesToLandMarks (sense_robot_landMark()); 

        for (int i = 0; i < size ; i++) 
        {
            dist = sqrt(pow((x - landMark_[i][0]), 2.0f) + pow((y - landMark_[i][1]), 2.0f));
            prob *= gaussian(dist, sensorNoise, distancesToLandMarks[i]);
        }

        return prob;    
    }


    /*!
    * measurement_prob_particle Calculates how likely a measurement should be.
    * @param index particle index.
    * @return double.
    */
    void RobotMcl::measurement_prob_particle()
    {
        for(auto & p:robotParticles_)
        {
            //initialize
            const double x(p.xPos);
            const double y(p.yPos);
            const double sensorNoise(particleNoise_.sense_noise);
            double prob = 1.0;
            double dist;
            const int size(landMark_.size());

            // Calculates how likely a measurement should be
            const std::vector<double>distancesToLandMarks (sense_robot_landMark()); 

            for (int i = 0; i < size ; i++) 
            {
                dist = sqrt(pow((x - landMark_[i][0]), 2.0f) + pow((y - landMark_[i][1]), 2.0f));
                prob *= gaussian(dist, sensorNoise, distancesToLandMarks[i]);
            }
            p.weight = prob;
            weights_.push_back(prob);

        }
    }


    /*!
    * move_robot It moves Robot and Particles.
    * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
    * @param forward distance to be moved.
    * @return RobotMcl object.
    */
    void RobotMcl::move_robot(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");

        //initialize
        double x(robotPose_.xPos);
        double y(robotPose_.yPos);
        double orient(robotPose_.orientRad);
        const double sense_noise(robotNoise_.sense_noise);
        const double turn_noise(robotNoise_.turn_noise);
        const double forward_noise(robotNoise_.forward_noise);

        // turn, and add randomness to the turning command
        orient = orient + turn + gen_gauss_random(0.0, turn_noise);
        orient = mod(orient, 2 * M_PI);

        // move, and add randomness to the motion command
        double dist = forward + gen_gauss_random(0.0, forward_noise);
        x = x + (cos(orient) * dist);
        y = y + (sin(orient) * dist);

        // cyclic truncate
        x = mod(x, worldSize_);
        y = mod(y, worldSize_);

        // set particle
        robotPose_.xPos = x;
        robotPose_.yPos = y;
        robotPose_.orientRad = orient;
    }

    /*!
    * move_particle It moves Robot and Particles.
    * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
    * @param forward distance to be moved.
    * @return RobotMcl object.
    */
    void RobotMcl::move_particle(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");


        Particles(robotParticles_.size());

        for(auto &p:robotParticles_)
        {
            //initialize
            double x(p.xPos);
            double y(p.yPos);
            double orient(p.orientRad);
            const double sense_noise(particleNoise_.sense_noise);
            const double turn_noise(particleNoise_.turn_noise);
            const double forward_noise(particleNoise_.forward_noise);

            // turn, and add randomness to the turning command
            orient = orient + turn + gen_gauss_random(0.0, turn_noise);
            orient = mod(orient, 2 * M_PI);

            // move, and add randomness to the motion command
            double dist = forward + gen_gauss_random(0.0, forward_noise);
            x = x + (cos(orient) * dist);
            y = y + (sin(orient) * dist);

            // cyclic truncate
            x = mod(x, worldSize_);
            y = mod(y, worldSize_);

            // set particle
            p.xPos = x;
            p.yPos = y;
            p.orientRad = orient;
        }
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
    * show_pose Returns the robot current position and orientation in a string format.
    * @param turn Angle of the new orientation(clockwise --> -ve, +ve anti clockwise).
    * @param forward distance to be moved.
    * @return RobotMcl object.
    */
    std::string RobotMcl::show_pose(const struct Particle &p)
    {
        
        // Returns the robot current position and orientation in a string format
        return "[id="+std::to_string(p.id) + " x=" + std::to_string(p.xPos) + " y=" + std::to_string(p.yPos) +
                    " orient=" + std::to_string(p.orientRad) + "]";           
    }
    /*!
    * resampling_wheel Returns the robot particles weights.
    * @param weights weights of the particles.
    * @return RobotMcl object.
    */
    void RobotMcl::resampling_wheel(void)
    { 
        const int size(static_cast<int>(robotParticles_.size()));
        Particles(size);
        int index (gen_real_random()*size);
        double beta(0.0f);
        double mw(*std::max_element(weights_.begin(), weights_.end()));
        std::vector<Particle> weighted_sample(size);

        for (int i = 0; i < size; i++) 
	    {
            const double weight(robotParticles_[index].weight);
            beta += gen_real_random() * 2.0 * mw;
            while (beta > weight); 
            {
                beta -=weight;
                index = mod((index + 1), size);
            }
            weighted_sample[i] = robotParticles_[index];
        }
        robotParticles_ = weighted_sample;
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
    double RobotMcl::mod(double first_term, double second_term)
    {
        // Compute the modulus
        return first_term - (second_term)*floor(first_term / (second_term));
    }

    /*!
    * mod max_weight find the maximum weights in particles.
    * @param particles std::vector<Particle> which contains all vectors.
    * @return max the maximum weight.
    */
    double RobotMcl::max_weight(std::vector<Particle> &particles)
    {
        //initialize
        const int size(particles.size());

        // Identify the max element in an array
        double max = 0;

        for (int i = 0; i < i; i++) 
        {
            if (particles[i].weight > max)
            {
                max = particles[i].weight;
            }
        }
        return max;
    }
#ifdef DEBUG_ROBOT_MCL
        std::vector<Particle> RobotMcl::debug_get_particles(void)
        {
            return robotParticles_;
        }
#endif

}