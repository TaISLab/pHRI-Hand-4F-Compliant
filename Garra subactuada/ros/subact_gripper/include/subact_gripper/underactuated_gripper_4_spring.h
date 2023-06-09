/* Class definition to interact with the 4 fingers 2 phalanges underactuated gripper with elastic link
    Date: 15-02-2022
    Author: Francisco J. Ruiz-Ruiz
*/

#pragma once

// Libraries for Serial communication
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <termio.h>
#include <unistd.h>
// Libraries for ROS
#include <Eigen/Dense>
#include <math.h>
#include <ros/node_handle.h>
#include <ros/time.h>
// msgs
#include <subact_gripper/pwm_control.h>
#include <subact_gripper/sub_gripper.h>
#include <subact_gripper/force_meas_dev.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

//namespace underactuated_gripper {

class UnderactuatedGripper4Spring{ 
  public:
    // Functions
    //UnderactuatedGripper4Spring();
    UnderactuatedGripper4Spring(ros::NodeHandle* node_handle);
    ~UnderactuatedGripper4Spring();

    //void init(ros::NodeHandle* node_handle);
    //float* getGripperState(); // Funcion para leer valores directamente del puerto serie [Not used]
    void actuateGripper(int pwm1, int pwm2, int pwm3, int pwm4);
    void update();
    bool setGraspingForce(float force);
    bool setGraspingForceTDC(float force);
    void inverseModel(double force);
    

    // Variables
    /*float finger1[3]; float finger3[3];
    float finger2[3]; float finger4[3];*/
    float theta_a_[4]; 
    float theta_1_[4]; 
    float theta_2_[4];
    float tau_a_[4];
    float forces_1_x_[4];
    float forces_1_z_[4];
    float forces_2_x_[4];
    float forces_2_z_[4];
    //float grasp_force_x_, grasp_force_z_;
    Eigen::Vector2d grasp_force_;
    Eigen::Vector3d interaction_forces_;
    Eigen::Vector4i pid_pwm_;
    Eigen::Vector4d Kp_, Ki_, Kd_;
    float ts_;
    Eigen::Matrix<double, 4,3> error_;
    Eigen::Matrix<double, 4,1> finger_force_;
    float fsensor1_, fsensor2_;
    
    Eigen::Matrix<double, 6,1> ffranka, franka_vel;    
    Eigen::Matrix<double, 7,1> franka_pose;

  private:
    // Functions
    void gripper_state_callback(const subact_gripper::sub_gripper& msg);
    void force_sensor_callback(const subact_gripper::force_meas_dev& msg);
//    void ffranka_callback(const std_msgs::Float64MultiArray& msg);
    void fpose_callback(const geometry_msgs::PoseStamped& msg);
    void ffranka_callback(const geometry_msgs::WrenchStamped& msg);
    void fvelocity_callback(const geometry_msgs::TwistStamped& msg);
    bool checkContact(int finger);
    void getFingersCartesianForce();
    void getGraspingForce();
    void getFingerForce();
    void getInteractionForces();

    // Publishers
    ros::Publisher garra_msg_pub_;
    ros::Publisher interaction_forces_pub_;

    // Subscribers
    ros::Subscriber garra_state_sub_;
    ros::Subscriber force_sensor_sub_;
    ros::Subscriber ffranka_sub, ffranka_for, franka_pose_sub, franka_vel_sub;

    // Variables
    int serial_port_;
    struct termios tty_;
    float gripper_state_[12];
    // Gripper params
    float a_; // Phalanx length
    float b_, psi_; // distal phalanx base and angle
    float gamma_; // servo origin angle
    float e_;
    float d_;
    float w_; // phalanx width
    float D_; // distance between parallel fingers
    float ks1_, ks2_, ls1r_, ls2r_; // springs params
    float contact_point1_, contact_point2_; // contact points
    int pwm_saturation_;
    double tara_z_;
    int tara_timer_;

    // variables para el modelo inverso
    Eigen::Vector4d tau_0_;
};

//}
