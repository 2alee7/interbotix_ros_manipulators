#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cmath>

using namespace std;

// It's hard to accurately model the robot's dynamics using geometry for the purpose of gravity compenstion.
// Although it's a brute-force method, we will functionally map all joint positions to a corresponding "empirical" (simulated) torque value for each joint affected by gravity
// This script iterates through a nested loop of every joint position combination and reads the absolute torque values (given that the screw axis of the joints are orthogonal to the link and the gravity vector)
// interbotix_ws_gazebo should already be running, with self-collision disabled and physics/gravity enabled

sensor_msgs::JointState joint_states;

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "gravcomp_torques");
    ros::NodeHandle n;

    ros::Subscriber robot_state = n.subscribe("wx250s/joint_states", 1, joint_state_cb);         // read absolute torque sensors
    ros::Publisher shoulder_pos_cmd = n.advertise<std_msgs::Float64>("wx250s/shoulder_controller/command", 1);
    ros::Publisher elbow_pos_cmd = n.advertise<std_msgs::Float64>("wx250s/elbow_controller/command", 1);
    ros::Publisher wrist_angle_pos_cmd = n.advertise<std_msgs::Float64>("wx250s/wrist_angle_controller/command", 1);

    ros::Rate loop_rate(100);

    const double s_pos_min = -108;
    const double s_pos_max = 114;

    const double e_pos_min = -123;
    const double e_pos_max = 92;

    const double wa_pos_min = -100;
    const double wa_pos_max = 123;

    ofstream outputFile("wx250grav.csv");
    if (!outputFile.is_open()) {
        cerr << "Error: Unable to open file for writing" << endl;
        return 1; // Return an error code
    }

    while (ros::ok()) {
        std_msgs::Float64 s_msg, e_msg, wa_msg;

        for (double s_pos = s_pos_min; s_pos <= s_pos_max; s_pos += 100) {
            s_msg.data = s_pos * M_PI / 180.0;
            shoulder_pos_cmd.publish(s_msg);
            
            for (double e_pos = e_pos_min; e_pos <= e_pos_max; e_pos += 100) {
                e_msg.data = e_pos * M_PI / 180.0;
                elbow_pos_cmd.publish(e_msg);

                for (double wa_pos = wa_pos_min; wa_pos <= wa_pos_max; wa_pos += 100) {
                    wa_msg.data = wa_pos * M_PI / 180.0;
                    wrist_angle_pos_cmd.publish(wa_msg);

                    ros::spinOnce();
                
                    double s_eff = joint_states.effort[5];
                    double e_eff = joint_states.effort[0];
                    double wa_eff = joint_states.effort[7];

                    outputFile << std::fixed << std::setprecision(3) << s_pos << "," << e_pos << "," << wa_pos << "," << s_eff << "," << e_eff << "," << wa_eff << endl;

                }
            }
        }
    
    }
    outputFile.close();
    return 0;
}
