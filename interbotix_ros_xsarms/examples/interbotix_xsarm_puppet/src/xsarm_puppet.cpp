#include <ros/ros.h>
#include <ros/console.h>
#include "interbotix_xs_msgs/JointGroupCommand.h"
#include "interbotix_xs_msgs/JointSingleCommand.h"
#include "interbotix_xs_msgs/RobotInfo.h"
#include <sensor_msgs/JointState.h>


// Define joint states for robot A and robot B
sensor_msgs::JointState joint_states_A;
sensor_msgs::JointState joint_states_B;

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_state_cb_A(const sensor_msgs::JointState &msg)
{
  joint_states_A = msg;
}

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_state_cb_B(const sensor_msgs::JointState &msg)
{
  joint_states_B = msg;
}
// GRAVITY COMPENSATION CALLBACK FUNCTIONS
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  // *** Robot specific m and l will be moved to the config file ***

  const double g = 9.81;  // Acceleration due to gravity (m/s^2)
  const double m1 = 0.360;      // mass of limbs (kg)
  const double m2 = 0.350;
  const double m3 = 0.223;

  const double l1 = 0.120; // 0.207;      // length of limbs (m)
  const double l2 = 0.134; // 0.200;
  const double l3 = 0.176;

std::vector<double> calculateGravity(double m1, double m2, double m3, double l1, double l2, double l3, double p_s, double p_e, double Kt) {

    // converting relative motor positions to absolute theta (relative to vertical) (rad)
    double theta_s = p_s;
    double theta_e = theta_s + (p_e + 1.5708);
    // double theta_wa = theta_e + (p_wa + 1.5708);
    double x = 1 / Kt;

    // respective torque approximation (N-m)
    double sTorque = x * m1 * g * l1 * sin(theta_s);
    double eTorque = x * m2 * g * l2  * sin(theta_e);
    // double waTorque = 0 * m3 * g * l3 * cos(theta_wa);

    std::vector<double> torques = {sTorque, eTorque}; //waTorque

    return torques;
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsarm_puppet");
  ros::NodeHandle n;

  std::string robot_name_master, robot_name_puppet;
  ros::param::get("~robot_name_master", robot_name_master);
  ros::param::get("~robot_name_puppet", robot_name_puppet);

  // Subscribe to A joint states and publish to B joint commands
  ros::Subscriber sub_positions_A = n.subscribe(robot_name_master + "/joint_states", 1, joint_state_cb_A);
  ros::Publisher pub_group_B = n.advertise<interbotix_xs_msgs::JointGroupCommand>(robot_name_puppet + "/commands/joint_group", 1);
  // ros::Publisher pub_single_B = n.advertise<interbotix_xs_msgs::JointSingleCommand>(robot_name_puppet + "/commands/joint_single", 1);

  // Subscribe to B joint states and publish to A joint commands
  ros::Subscriber sub_positions_B = n.subscribe(robot_name_puppet + "/joint_states", 1, joint_state_cb_B);
  ros::Publisher pub_group_A = n.advertise<interbotix_xs_msgs::JointGroupCommand>(robot_name_master + "/commands/joint_group", 1);
  // ros::Publisher pub_single_A = n.advertise<interbotix_xs_msgs::JointSingleCommand>(robot_name_master + "/commands/joint_single", 1);

  // Get some robot info to figure out how many joints the robot has
  ros::ServiceClient srv_robot_info = n.serviceClient<interbotix_xs_msgs::RobotInfo>(robot_name_master + "/get_robot_info");

  ros::Rate loop_rate(1000);
  bool success;

  // Wait for the 'arm_node' to finish initializing
  while ((pub_group_A.getNumSubscribers() < 1 || joint_states_A.position.size() < 1 || pub_group_B.getNumSubscribers() < 1 || joint_states_B.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  
  // Get info about the 'arm' group from the first robot
  interbotix_xs_msgs::RobotInfo arm_info_srv;
  arm_info_srv.request.cmd_type = "group";
  arm_info_srv.request.name = "arm";
  success = srv_robot_info.call(arm_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get info on the 'arm' group.");
    return 1;
  }

  // Get gripper info from the first robot
  interbotix_xs_msgs::RobotInfo gripper_info_srv;
  gripper_info_srv.request.cmd_type = "group";
  gripper_info_srv.request.name = "gripper";
  success = srv_robot_info.call(gripper_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get info on the 'gripper' joint.");
    return 1;
  }

  // retrieve gain values from gains.yaml, which is customizable for tuning
  const double Kt = 1.1555; // see below for explanation
  
  std::string arm = "arm";

  double arm_K_p; // proportional gain, Dynamixel default 900
  ros::param::get(arm + "/K_p", arm_K_p);
  double arm_K_d; // derivative gain, Dynamxiel default 0
  ros::param::get(arm + "/K_d", arm_K_d);
  double arm_K; // 'damping term,' should increase with K_p to maintain stability
  ros::param::get(arm + "/K", arm_K);

  std::string gripper = "gripper";

  double gripper_K_p; // proportional gain, Dynamixel default 900
  ros::param::get(gripper + "/K_p", gripper_K_p);
  double gripper_K_d; // derivative gain, Dynamxiel default 0
  ros::param::get(gripper + "/K_d", gripper_K_d);
  double gripper_K; // 'damping term,' should increase with K_p to maintain stability
  ros::param::get(gripper + "/K", gripper_K);


  // TODO: What do the tabular Dynamixel gain values represent? i.e. if K_p and K_d = 0? Will just adjust manually for now
  // K_D = K_d(TBL) / 16
  // K_P = K_p(TBL) / 128

  // ros::ServiceClient srv_motor_gains_A = n.serviceClient<interbotix_xs_msgs::MotorGains>(robot_name_master + "/set_motor_pid_gains");
  // srv_motor_gains_A.waitForExistence();

  // ros::ServiceClient srv_motor_gains_B = n.serviceClient<interbotix_xs_msgs::MotorGains>(robot_name_puppet + "/set_motor_pid_gains");
  // srv_motor_gains_B.waitForExistence();

  // interbotix_xs_msgs::MotorGains pd_gains;
  // pd_gains.request.cmd_type = "group"; // Commanding a joint group
  // pd_gains.request.name = "arm";       // Name of the joint group

  // pd_gains.request.kp_pos = 0 * 128;
  // pd_gains.request.kd_pos = 0 * 128;
  // pd_gains.request.ki_pos = 0 * 128;

  // srv_motor_gains_A.call(pd_gains);
  // srv_motor_gains_B.call(pd_gains);   // IRRELEVANT; ONLY AFFECTS POS. CONTROL

// MAIN LOOP
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  while (ros::ok())
  {
    // control law for effort/torque tau for each joint pair 1 and 2 (by Ben Katz): tau_1 = K_p*(p_2 - p_1) + K_d(v_2 - v_1) - K(v_1)
    // converting to effort (mA) = ((K_p*(p_2 - p_1) + K_d(v_2 - v_1) - K(v_1))/(Kt * 1000); where Kt = motor torque constant
    // Kt for different motors (estimated): XL 430-250: 0.8780; XM 430-350: 1.6273; XM 430-210-R: 1.1555
    // "In general, positions are given in radians, velocities are given in radians per second, and effort is given in milliamps."
    
    interbotix_xs_msgs::JointGroupCommand pos_msg_A;
    pos_msg_A.name = "arm";

    interbotix_xs_msgs::JointGroupCommand gripper_pos_msg_A;
    gripper_pos_msg_A.name = "gripper";

    interbotix_xs_msgs::JointGroupCommand pos_msg_B;
    pos_msg_B.name = "arm";

    interbotix_xs_msgs::JointGroupCommand gripper_pos_msg_B;
    gripper_pos_msg_B.name = "gripper";

    // initializing and defining the p_s, p_e, p_wa for both robots...
    // see calculateGravity for my gravity compensation code
    // double p_s_A, p_e_A;
    // double p_s_B, p_e_B;

    // p_s_A = joint_states_A.position[1];
    // p_e_A = joint_states_A.position[2];
    // std::vector<double> torques_A = calculateGravity(m1, m2, m3, l1, l2, l3, p_s_A, p_e_A, Kt);

    // p_s_B = joint_states_B.position[1];
    // p_e_B = joint_states_B.position[2];
    // std::vector<double> torques_B = calculateGravity(m1, m2, m3, l1, l2, l3, p_s_B, p_e_B, Kt);

    // calculate respective efforts

    for (auto const& index : arm_info_srv.response.joint_state_indices)
    {
      double p_A, v_A; // p, v for robot A
      double p_B, v_B; // p, v for robot B

      p_A = joint_states_A.position[index]; // Position in rad
      v_A = joint_states_A.velocity[index]; // Velocity in rad
      p_B = joint_states_B.position[index]; 
      v_B = joint_states_B.velocity[index]; 
      
      double effort_A = ((180.0 / M_PI)*(arm_K_p * (p_B - p_A) + arm_K_d * (v_B - v_A) - arm_K * v_A) / (Kt));  // pos., vel. converted to deg, deg/s
      double effort_B = ((180.0 / M_PI)*(arm_K_p * (p_A - p_B) + arm_K_d * (v_A - v_B) - arm_K * v_B) / (Kt));
      
      // Add gravity compensation term only for joints 2, 3, and 4
      // if (index == 1) {
      //     effort_A += torques_A[0]; // Add gravity compensation term for shoulder joint for robot A
      //     effort_B += torques_B[0]; // Add gravity compensation term for shoulder joint for robot B
      // } else if (index == 2) {
      //     effort_A += torques_A[1]; // Add gravity compensation term for elbow joint for robot A
      //     effort_B += torques_B[1]; // Add gravity compensation term for elbow joint for robot B
      // }
      
      pos_msg_A.cmd.push_back(effort_A);
      pos_msg_B.cmd.push_back(effort_B);

    }

    for (auto const& index : gripper_info_srv.response.joint_state_indices)
    {
      double p_A, v_A; // p, v for robot A
      double p_B, v_B; // p, v for robot B

      p_A = joint_states_A.position[index]; // Position in rad
      v_A = joint_states_A.velocity[index]; // Velocity in rad
      p_B = joint_states_B.position[index]; 
      v_B = joint_states_B.velocity[index]; 
      
      double effort_A = (gripper_K_p * (p_B - p_A) + gripper_K_d * (v_B - v_A) - gripper_K * v_A) / (Kt);  // pos., vel. converted to deg, deg/s
      double effort_B = (gripper_K_p * (p_A - p_B) + gripper_K_d * (v_A - v_B) - gripper_K * v_B) / (Kt);
      
      gripper_pos_msg_A.cmd.push_back(effort_A);
      gripper_pos_msg_B.cmd.push_back(effort_B);

    }

    // publish efforts

    pub_group_A.publish(gripper_pos_msg_A);
    pub_group_B.publish(gripper_pos_msg_B);
    pub_group_A.publish(pos_msg_A);
    pub_group_B.publish(pos_msg_B);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
