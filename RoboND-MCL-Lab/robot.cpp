#include"robot.hpp"
using namespace std;

// Global Functions
// Random Generators
random_device rd;
mt19937 gen(rd());

Robot::Robot()
{
    // Constructor
    x_ = gen_real_random() * worldSize_; // robot's x coordinate
    y_ = gen_real_random() * worldSize_; // robot's y coordinate
    orient_ = gen_real_random() * 2.0 * M_PI; // robot's orientation

    forward_noise_ = 0.0; //noise of the forward movement
    turn_noise_ = 0.0; //noise of the turn
    sense_noise_ = 0.0; //noise of the sensing
}

void Robot::set_robot_pose(double new_x, double new_y, double new_orient)
{
    // Set robot new position and orientation
    if (new_x < 0 || new_x >= worldSize_)
        throw std::invalid_argument("X coordinate out of bound");
    if (new_y < 0 || new_y >= worldSize_)
        throw std::invalid_argument("Y coordinate out of bound");
    if (new_orient < 0 || new_orient >= 2 * M_PI)
        throw std::invalid_argument("Orientation must be in [0..2pi]");

    x_ = new_x;
    y_ = new_y;
    orient_ = new_orient;
}

void Robot::set_robot_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
{
    // Simulate noise, often useful in particle filters
    forward_noise_ = new_forward_noise;
    turn_noise_ = new_turn_noise;
    sense_noise_ = new_sense_noise;
}

/*!
* set_robot_pose set Robot Noise.
* @param landMark Robot new forward move noise.
* @param worldsize Robot new turn move noise.
* @return void.
*/
void Robot::set_robot_enviornment(std::vector<std::vector<double>> landMark, double worldsize)
{
    std::copy(landMark.begin(), landMark.end(), std::back_inserter(landMark_));
    worldSize_ = worldsize;
}

vector<double> Robot::sense()
{
    // Measure the distances from the robot toward the landmarks
    const int size(landMark_.size());
    std::vector<double> z(size);
    double dist;

    for (int i = 0; i < size; i++) 
    {
        dist = sqrt(pow((x_ - landMark_[i][0]), 2.0f) + pow((y_ - landMark_[i][1]), 2.0f));
        dist += gen_gauss_random(0.0, sense_noise_);
        z[i] = dist;
    }
    return z;
}

Robot Robot::move(double turn, double forward)
{
    if (forward < 0)
        throw std::invalid_argument("Robot cannot move backward");

    // turn, and add randomness to the turning command
    orient_ = orient_ + turn + gen_gauss_random(0.0, turn_noise_);
    orient_ = mod(orient_, 2 * M_PI);

    // move, and add randomness to the motion command
    double dist = forward + gen_gauss_random(0.0, forward_noise_);
    x_ = x_ + (cos(orient_) * dist);
    y_ = y_ + (sin(orient_) * dist);

    // cyclic truncate
    x_ = mod(x_, worldSize_);
    y_ = mod(y_, worldSize_);

    // set particle
    Robot res;
    res.set_robot_enviornment(landMark_, worldSize_);
    res.set_robot_pose(x_, y_, orient_);
    res.set_robot_noise(forward_noise_, turn_noise_, sense_noise_);

    return res;
}

string Robot::show_pose()
{
    // Returns the robot current position and orientation in a string format
    return "[x=" + to_string(x_) + " y=" + to_string(y_) + " orient=" + to_string(orient_) + "]";
}

string Robot::read_sensors()
{
    // Returns all the distances from the robot toward the landmarks
    distancesToLandMarks_ = sense();
    string readings = "[";
    for (int i = 0; i < distancesToLandMarks_.size(); i++) {
        readings += to_string(distancesToLandMarks_[i]) + " ";
    }
    readings[readings.size() - 1] = ']';

    return readings;
}

double Robot::measurement_prob()
{
    // Calculates how likely a measurement should be
    double prob = 1.0;
    double dist;
    const int size(landMark_.size());

    for (int i = 0; i < size; i++) {
        dist = sqrt(pow((x_ - landMark_[i][0]), 2) + pow((y_ - landMark_[i][1]), 2));
        prob *= gaussian(dist, sense_noise_, distancesToLandMarks_[i]);
    }

    return prob;
}

double Robot::gen_gauss_random(double mean, double variance)
{
    // Gaussian random
    normal_distribution<double> gauss_dist(mean, variance);
    return gauss_dist(gen);
}

double Robot::gaussian(double mu, double sigma, double x)
{
    // Probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
}

// Functions
double Robot::gen_real_random()
{
    // Generate real random between 0 and 1
    uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
    return real_dist(gen);
}

double Robot::mod(double first_term, double second_term)
{
    // Compute the modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}
