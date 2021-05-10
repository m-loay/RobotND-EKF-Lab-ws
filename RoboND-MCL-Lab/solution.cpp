//Compile with: g++ solution.cpp -o app -std=c++11 -I/usr/include/python2.7 -lpython2.7
// #include "matplotlibcpp.h" //Graph Library

#include"robot.hpp"
#include "robot_mcl.hpp"
using namespace std;
using namespace robot_mcl ;
// Landmarks
std::vector<std::vector<double>> landMark{ { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
    { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
    { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters
double world_size = 100.0;
int main()
{
    //Practice Interfacing with Robot Class
    // Instantiating a robot object from the Robot class
    Robot myrobot;
    myrobot.set_robot_enviornment(landMark, world_size);
    

    // TODO: Set robot new position to x=10.0, y=10.0 and orientation=0
    // Fill in the position and orientation values in myrobot.set() function
    myrobot.set_robot_pose(10.0f, 10.0,0.0f);
    //myrobot.set_robot_pose(10.0f, 10.0,0.0f);

    // Printing out the new robot position and orientation
    // cout << myrobot.show_pose() << endl;
    cout << myrobot.show_pose() << endl;

    // TODO: Rotate the robot by PI/2.0 and then move him forward by 10.0
    // Use M_PI for the pi value
    myrobot.move(M_PI/2.0f, 10.0f);

    // TODO: Print out the new robot position and orientation
    cout << myrobot.show_pose() << endl;

    // Printing the distance from the robot toward the eight landmarks
    cout << myrobot.read_sensors() << endl;

    // // Create a set of particles
    // const int n = 1000;
    // Robot p[n];

    // for (int i = 0; i < n; i++) {
    //     p[i].set_noise(0.05, 0.05, 5.0);
    //     //cout << p[i].show_pose() << endl;
    // }

    // //Re-initialize myrobot object and Initialize a measurment vector
    // myrobot = Robot();
    // vector<double> z;

    // //Iterating 50 times over the set of particles
    // int steps = 50;
    // for (int t = 0; t < steps; t++) {

    //     //Move the robot and sense the environment afterwards
    //     myrobot = myrobot.move(0.1, 5.0);
    //     z = myrobot.sense();

    //     // Simulate a robot motion for each of these particles
    //     Robot p2[n];
    //     for (int i = 0; i < n; i++) {
    //         p2[i] = p[i].move(0.1, 5.0);
    //         p[i] = p2[i];
    //     }

    //     //Generate particle weights depending on robot's measurement
    //     double w[n];
    //     for (int i = 0; i < n; i++) {
    //         w[i] = p[i].measurement_prob(z);
    //         //cout << w[i] << endl;
    //     }

    //     //Resample the particles with a sample probability proportional to the importance weight
    //     Robot p3[n];
    //     int index = static_cast<int>(gen_real_random() * n);
    //     //cout << index << endl;
    //     double beta = 0.0;
    //     double mw = max(w, n);
    //     //cout << mw;
    //     for (int i = 0; i < n; i++) {
    //         beta += gen_real_random() * 2.0 * mw;
    //         while (beta > w[index]) {
    //             beta -= w[index];
    //             index = static_cast<int>(mod((index + 1), n));
    //         }
    //         p3[i] = p[index];
    //     }
    //     for (int k = 0; k < n; k++) {
    //         p[k] = p3[k];
    //         //cout << p[k].show_pose() << endl;
    //     }

    //     //Evaluate the Error
    //     cout << "Step = " << t << ", Evaluation = " << evaluation(myrobot, p, n) << endl;

    //     //####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

    //     //Graph the position of the robot and the particles at each step
    //     visualization(n, myrobot, t, p2, p3);

    //} //End of Steps loop

    return 0;
}
