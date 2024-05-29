/* Class implementation to interact with the 4 fingers 2 phalanges underactuated gripper with elastic link
    Date: 15-02-2022
    Author: Francisco J. Ruiz-Ruiz
*/

#include <subact_gripper/underactuated_gripper_4_spring.h>
#include <ros/ros.h>

#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>




UnderactuatedGripper4Spring::UnderactuatedGripper4Spring(ros::NodeHandle* node_handle){
std::cout << "STARTING" << std::endl;
    // PUBLISHERS 
    garra_msg_pub_ = node_handle->advertise<subact_gripper::pwm_control>("/robot_subactuated_gripper/pwm", 1);
    interaction_forces_pub_ = node_handle->advertise<geometry_msgs::WrenchStamped>("/robot_subactuated_gripper/interaction_force", 1);
    // SUBSCRIBERS
    garra_state_sub_ = node_handle->subscribe("/robot_subactuated_gripper/gripper_state",1, &UnderactuatedGripper4Spring::gripper_state_callback, this);
    force_sensor_sub_ = node_handle->subscribe("/robot_subactuated_gripper/ground_truth_grasping_force",1, &UnderactuatedGripper4Spring::force_sensor_callback, this);

    franka_pose_sub = node_handle->subscribe("/robot_vel_ctrl/pose",1, &UnderactuatedGripper4Spring::fpose_callback, this);
    franka_vel_sub = node_handle->subscribe("/robot_vel_ctrl/velocity_filtered",1, &UnderactuatedGripper4Spring::fvelocity_callback, this);

//    ffranka_sub = node_handle->subscribe("/franka_state_controller/F_ext", 1, &UnderactuatedGripper4Spring::ffranka_callback, this);

    ffranka_for = node_handle->subscribe("/franka_state_controller/F_ext",1, &UnderactuatedGripper4Spring::ffranka_callback, this);


    // VARIABLES INITIALIZATION
    for(int i=0; i<4; i++){
        /*if (i<3){
            error_[i] = 0;
        }*/
        theta_a_[i] = 0;
        theta_1_[i] = 0;
        theta_2_[i] = 0;
        forces_1_x_[i] = 0;
        forces_2_x_[i] = 0;
        forces_1_z_[i] = 0;
        forces_2_z_[i] = 0;
    }

    grasp_force_.setZero();
    interaction_forces_.setZero();

    Kp_ << 55.0, 58.0, 58.0, 55.0; // kp = {36.5, 60.5, 40.5, 50.5}                             // <------------------
    Ki_ << 20.0, 22.0, 22.0, 20.0; // ki = {3.80, 6.80, 3.80, 4.80}  
    ts_ = 0.01;
    Kd_.setZero();
    pwm_saturation_ = 500;
    pid_pwm_.setZero();
    error_.setZero();
    

    a_ = 0.04;
    b_ = 0.02;
    psi_ = M_PI_2;
    contact_point1_ = 0.02;
    contact_point2_ = 0.02;
    gamma_ = 0.9879; // 56.6 degrees
    e_ = 0.028;
    d_ = 0.05;
    w_ = 0.01; // phalanx width
    D_ = 0.08;// distance between parallel fingers
    ks1_ = 60;
    ks2_ = 675;
    ls1r_ = 0.016; 
    ls2r_ = 0.057;

    tara_z_ = 0;
    tara_timer_ = 0;

    fsensor1_ = 0;
    fsensor2_ = 0;
    
    ffranka.setZero();

    std::cout << "----- CLASS INITIALIZED -----" << std::endl;

/*    // open port
    //const char[] port = "/dev/ttyUSB0";
    serial_port_ = open("/dev/ttyUSB0", O_RDWR);
    // check errors
    if(serial_port_ < 0) {
        std::cout << "Error " << errno << "opening /dev/ttyUSB0: " << strerror(errno) << std::endl;
    }

    if(tcgetattr(serial_port_, &tty_) != 0){
        std::cout << "Error " << errno << "from tcgetattr: " << strerror(errno) << std::endl;
    }

    // Configure serial port
    tty_.c_cflag &= ~PARENB; // clear parity bit
    tty_.c_cflag &= ~CSTOPB; // one stop bit (set this bit for two stop bits)
    tty_.c_cflag |= CS8; // messages of 8 bits per byte
    tty_.c_cflag &= ~CRTSCTS; // RTS/CTS hardware flow control
    tty_.c_cflag |= CREAD | CLOCAL; // Turn on read and ignore ctrl lines (clocal = 1)
    tty_.c_lflag = 0; // no signaling chars, no echo, no canonical processing
    /*tty_.c_lflag &= ~ICANON;
    tty_.c_lflag &= ~ECHO;
    tty_.c_lflag &= ~ECHOE;
    tty_.c_lflag &= ~ECHONL;
    tty_.c_lflag &= ~ISIG;*/
/*    tty_.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow ctrl
    tty_.c_iflag &= ~(IGNBRK | BRKi_NT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); //Disable any special handling of received bytes 
    tty_.c_oflag &= ~(OPOST | ONLCR);
    tty_.c_cc[VTIME] = 1; // Timeout of 0.1 seconds
    tty_.c_cc[VMIN] = 1; // Or read 1 bytes
    // set baudrate
    cfsetispeed(&tty_, B500000);
    cfsetospeed(&tty_, B500000);

    // save tty settings
    if(tcsetattr(serial_port_, TCSANOW, &tty_) != 0){
        std::cout << "Error " << errno << "from tcsetattr: " << strerror(errno) << std::endl;
    }
*/    
}

UnderactuatedGripper4Spring::~UnderactuatedGripper4Spring(){
    //close(serial_port_);
}

/*float* UnderactuatedGripper4Spring::getGripperState(){
    char msg_read[10];
    char buffer;
    int n = 0; int index = 0; int global_index = 0;
    bool tab_init = false;

    do {
        n = read(serial_port_, &buffer, 1);
        if(buffer != '\t'){
            //tab_init = false;
            msg_read[index] = buffer;
            index++;
        } else {
            index = 0;
            std::string data_s(msg_read);
            std::stringstream ss;
            ss << data_s;
            ss >> gripper_state_[global_index];
            global_index++;
            //gripper_state_[global_index] = std::stof(data_s);
        }

    }while(global_index < 12);//while(buffer != '\r' && n > 0);

    if(n<0){
        std::cout << "Error reading serial port, error code " << n << std::endl;
    }
    else{
        for(int i=0;i<12;i++){
            std::cout << gripper_state_[i] << "  "; 
        }
        std::cout << "indexes: " << index << std::endl;
    }
    return &gripper_state_;
}*/

void UnderactuatedGripper4Spring::gripper_state_callback(const subact_gripper::sub_gripper& msg){
    /*for(int i=0; i<3; i++){
        finger1[i] = msg.dedo_1[i];
        finger2[i] = msg.dedo_2[i];
        finger3[i] = msg.dedo_3[i];
        finger4[i] = msg.dedo_4[i];
    }*/
    theta_a_[0] = msg.dedo_1[0]; theta_a_[1] = msg.dedo_2[0]; theta_a_[2] = msg.dedo_3[0]; theta_a_[3] = msg.dedo_4[0];
    theta_1_[0] = msg.dedo_1[1]; theta_1_[1] = msg.dedo_2[1]; theta_1_[2] = msg.dedo_3[1]; theta_1_[3] = msg.dedo_4[1];
    theta_2_[0] = msg.dedo_1[2]; theta_2_[1] = msg.dedo_2[2]; theta_2_[2] = msg.dedo_3[2]; theta_2_[3] = msg.dedo_4[2];
    // Transform from deg to rad
    for(int i=0; i<4; i++){
        theta_a_[i] = theta_a_[i] * M_PI/180;
        theta_1_[i] = theta_1_[i] * M_PI/180;
        theta_2_[i] = theta_2_[i] * M_PI/180;
    }
     
}

void UnderactuatedGripper4Spring::force_sensor_callback(const subact_gripper::force_meas_dev& msg){
    fsensor1_ = msg.sensor1;
    fsensor2_ = msg.sensor2;
    //std::cout << "Sensor msg: " << fsensor1_ << "  " << fsensor2_ << std::endl;
}


//void UnderactuatedGripper4Spring::ffranka_callback(const std_msgs::Float64MultiArray& msg){
void UnderactuatedGripper4Spring::ffranka_callback(const geometry_msgs::WrenchStamped& msg){
//    std::cout << "-------------:::FFranka msg received -------" << std::endl;
    /*for(int in = 0; in<6; in++){
        ffranka(in) = msg.data[in];
    }*/
    ffranka(0) = msg.wrench.force.x;
    ffranka(1) = msg.wrench.force.y;
    ffranka(2) = msg.wrench.force.z;
    ffranka(3) = msg.wrench.torque.x;
    ffranka(4) = msg.wrench.torque.y;
    ffranka(5) = msg.wrench.torque.z;
}

void UnderactuatedGripper4Spring::fpose_callback(const geometry_msgs::PoseStamped& msg){
    franka_pose(0) = msg.pose.position.x;
    franka_pose(1) = msg.pose.position.y;
    franka_pose(2) = msg.pose.position.z;
    franka_pose(3) = msg.pose.orientation.x;
    franka_pose(4) = msg.pose.orientation.y;
    franka_pose(5) = msg.pose.orientation.z;
    franka_pose(6) = msg.pose.orientation.w;
}

void UnderactuatedGripper4Spring::fvelocity_callback(const geometry_msgs::TwistStamped& msg){
    franka_vel(0) = msg.twist.linear.x;
    franka_vel(1) = msg.twist.linear.y;
    franka_vel(2) = msg.twist.linear.z;
    franka_vel(3) = msg.twist.angular.x;
    franka_vel(4) = msg.twist.angular.y;
    franka_vel(5) = msg.twist.angular.z;
}

void UnderactuatedGripper4Spring::actuateGripper(int pwm1, int pwm2, int pwm3, int pwm4){
    subact_gripper::pwm_control msg;
    msg.header.stamp = ros::Time::now();
    msg.c1 = pwm1;
    msg.c2 = pwm2;
    msg.c3 = pwm3;
    msg.c4 = pwm4;
    garra_msg_pub_.publish(msg);
}

bool UnderactuatedGripper4Spring::checkContact(int finger){
    bool contact_found = true;

    /*for(int i=0; i<4; i++){
        // if any theta 2 is 0, no contact is made
        if(theta_2_[i] < 0.035){
            contact_found = false;
        }
    }*/
    if(theta_2_[finger] < 0.035){
            contact_found = false;
    }



    return contact_found;
}

void UnderactuatedGripper4Spring::getFingersCartesianForce(){
    // Variables
    float fj, fs, c, theta_c;
    float g, j, theta_0, theta_j, theta_3;
    float tau_0, alfa, beta;
    float tau_2_prima, xs1, contact_point2_prima, theta_2_prima;
    float P2y, P2x;
    float h;
    Eigen::Vector2d t, F, P1, P2, P3;
    Eigen::Matrix2d T, J, Fcart, Transform;
    Eigen::Matrix2d Jpn, Cp, A, B;//, C;
    Eigen::Matrix<double, 1, 2> Ja, J2;
    Eigen::Matrix<double, 2, 4> Tn, Jp_transpose;
    Eigen::Matrix<double, 3, 2> Kind, C;
    Eigen::Matrix3d Kdep;
    int lambda;

//    std::cout << "tau_0 measured: ";
    // Repeat for each finger
    for(int i=0; i<4; i++){
         //std::cout << "Finger " << i << " theta2: " << theta_2_[i];
        if(checkContact(i)){

            // Compute common params to both methods
            P1 << a_*cos(theta_1_[i]), a_*sin(theta_1_[i]);
            P2 << a_*cos(theta_1_[i]) + b_*cos(theta_1_[i] + theta_2_[i] - psi_) , a_*sin(theta_1_[i]) + b_*sin(theta_1_[i] + theta_2_[i] - psi_);
            P3 << e_*cos(-gamma_) + d_*cos(theta_a_[i]), e_*sin(-gamma_) + d_*sin(theta_a_[i]);
            c = (P2-P3).norm();
            theta_c = atan2(P2(1)-P3(1), P2(0)-P3(0));
            g = P3.norm();
            theta_0 = atan2(P3(1),P3(0));
            j = P2.norm();
            theta_j = atan2(P2(1),P2(0));
            theta_3 = atan2(P1(1)-P2(1), P1(0)-P2(0));
            //std::cout << " g: " << g << " theta_0: " << theta_0 << " j: " << j << " theta_j: " << theta_j;

            // Compute spring forces
            fs = ks2_ * (ls2r_ - c);
            fj = ks1_ * (ls1r_ - j);
            // std::cout << " c: " << c ;
            
            /* //// BIRGLEN APPROACH
            // Compute tau_0
            alfa = theta_c - (theta_0 + M_PI_2); // MAL: theta_c está calculado con respecto a la horizontal, con esta formula hay que obtenerlo con respecto a g
            tau_0 = fs * g * cos(alfa);

            // Compute tau_a
            beta = theta_c - (theta_a_[i] + M_PI_2);
            tau_a_[i] = (d_*cos(beta))/(g*cos(alfa)) * tau_0;
        
            // Compute tau_2_prima            
            xs1 = a_ * sin(theta_1_[i] - theta_j); 
            tau_2_prima = ks1_ * (ls1r_ - j) * xs1;

            // Compose torque vector
            t << tau_0, tau_2_prima;
            // Compose transmision matrix
            h = b_ * sin(M_PI + theta_1_[i] + theta_2_[i] - psi_ - theta_c)/sin(theta_c - theta_1_[i]);
            T(0,0) = 1;  T(0,1) = -h/(h+a_);
            T(1,0) = 0;  T(1,1) = 1;
            // correccion del punto de contacto y angulo theta2, espesor de las falanges no despreciable
            theta_2_prima = theta_2_[i] + atan(w_/contact_point2_);
            contact_point2_prima = sqrt(pow(contact_point2_,2) + pow(w_,2));
            // Compose Jacobian matrix
            J(0,0) = contact_point1_;                                   J(0,1) = 0;
            J(1,0) = contact_point2_prima + a_ * cos(theta_2_prima);    J(1,1) = contact_point2_prima;
            // Compute normal forces
            F = J.inverse().transpose() * T.inverse().transpose() * t;  
            */ ////END OF BIRGLEN APPROACH


            //// PRINCIPLE OF VIRTUAL WORK (PTV) APPROACH
            // Jacobian of the contact points
            Kind(0,0) = 1; Kind(0,1) = 1; 
            Kind(1,0) = -(d_*sin(theta_a_[i])+c*sin(theta_c)+b_*sin(theta_3)+a_*sin(M_PI+theta_1_[i])); Kind(1,1) = -a_*sin(M_PI+theta_1_[i]); 
            Kind(2,0) =   d_*cos(theta_a_[i])+c*cos(theta_c)+b_*cos(theta_3)+a_*cos(M_PI+theta_1_[i]);  Kind(2,1) =  a_*cos(M_PI+theta_1_[i]); 

            Kdep(0,0) = 1; Kdep(0,1) = 1; Kdep(0,2) = 1;
            Kdep(1,0) = 0; Kdep(1,1) = -(c*sin(theta_c)+b_*sin(theta_3)+a_*sin(M_PI+theta_1_[i])); Kdep(1,2) = -(b_*sin(theta_3)+a_*sin(M_PI+theta_1_[i]));
            Kdep(2,0) = 0; Kdep(2,1) =   c*cos(theta_c)+b_*cos(theta_3)+a_*cos(M_PI+theta_1_[i]);  Kdep(2,2) =   b_*cos(theta_3)+a_*cos(M_PI+theta_1_[i]);

            C = -Kdep.inverse() * Kind;
            /*A(0,0) = d_*sin(theta_a_[i]);  A(0,1) = -b_*sin(theta_1_[i]+theta_2_[i]-psi_); 
            A(1,0) = -d_*cos(theta_a_[i]); A(1,1) = b_*cos(theta_1_[i]+theta_2_[i]-psi_);
            B(0,0) = -a_*sin(theta_1_[i])-b_*sin(theta_1_[i]+theta_2_[i]-psi_); B(0,1) = c*sin(theta_c); 
            B(1,0) = a_*cos(theta_1_[i])+b_*cos(theta_1_[i]+theta_2_[i]-psi_);  B(1,1) = -c*cos(theta_c);
            C = B.inverse()*A;*/


            Tn.setZero();
            Tn(0,0) = -sin(theta_1_[i]);              Tn(0,1) = cos(theta_1_[i]);
            Tn(1,2) = -sin(theta_1_[i]+theta_2_[i]);  Tn(1,3) = cos(theta_1_[i]+theta_2_[i]);

            Jp_transpose(0,0) = -contact_point1_*sin(theta_1_[i]);      Jp_transpose(1,0) = 0;
            Jp_transpose(0,1) = contact_point1_*cos(theta_1_[i]);       Jp_transpose(1,1) = 0;
            Jp_transpose(0,2) = -a_*sin(theta_1_[i])-contact_point2_*sin(theta_1_[i]+theta_2_[i]);  Jp_transpose(1,2) = -contact_point2_*sin(theta_1_[i]+theta_2_[i]);
            Jp_transpose(0,3) = a_*cos(theta_1_[i])+contact_point2_*cos(theta_1_[i]+theta_2_[i]);   Jp_transpose(1,3) = contact_point2_*cos(theta_1_[i]+theta_2_[i]);

            Cp(0,0) = C(0,0);   Cp(0,1) = C(0,1);
            Cp(1,0) = 0;        Cp(1,1) = 1;

            Jpn = Tn * Jp_transpose.transpose() * Cp;

            // Jacobian of the return spring
            J2 << 0, a_*b_*cos(theta_2_[i])/j;

            // Jacobian of the actuator
            Ja << 1, 0;
            tau_a_[i] = fs*d_*cos(theta_c - (theta_a_[i] + M_PI/2));

            // Computation of the normal forces
            F = -Jpn.transpose().inverse() * (Ja.transpose() * tau_a_[i] + J2.transpose() * fj);
            //// END PTV APPROACH


            //std::cout << "Force " << i << ": " << F(0) << " " << F(1) << std::endl;


            // Compute cartesian forces
            if (i==0 || i==3){
                lambda = 1;
            } else {
                lambda = -1;
            }
            Transform(0,0) = lambda * cos(M_PI_2 + theta_1_[i]);    Transform(0,1) = lambda * cos(theta_1_[i] + theta_2_[i]  + M_PI_2);
            Transform(1,0) = sin(M_PI_2 + theta_1_[i]);             Transform(1,1) = sin(theta_1_[i] + theta_2_[i] + M_PI_2);
            Fcart = Transform * F.asDiagonal();

            // Save results
            forces_1_x_[i] = Fcart(0,0); forces_2_x_[i] = Fcart(0,1);
            forces_1_z_[i] = Fcart(1,0); forces_2_z_[i] = Fcart(1,1); 


            // std::cout << F.transpose() << std::endl;
            //std::cout << " CONTACT" << std::endl;
//            std::cout << tau_0 << "  ";
        } else {
            // si no se esta agarrando nada 
            forces_1_x_[i] = 0; forces_2_x_[i] = 0;
            forces_1_z_[i] = 0; forces_2_z_[i] = 0;
            //std::cout  << " NO CONTACT" << std::endl;
//            std::cout << "0  ";
        }
       
    }
//     std::cout << std::endl;

    // std::cout << "End computation" << std::endl;
    
}

void UnderactuatedGripper4Spring::getGraspingForce(){
    grasp_force_(0) = 0.5 * (forces_1_x_[0] + forces_1_x_[3] + forces_2_x_[0] + forces_2_x_[3] - (forces_1_x_[2] + forces_1_x_[1] + forces_2_x_[2] + forces_2_x_[1]));
    grasp_force_(1) = 0.5 * (forces_1_z_[0] + forces_1_z_[1] + forces_1_z_[2] + forces_1_z_[3] - (forces_2_z_[0] + forces_2_z_[1] + forces_2_z_[2] + forces_2_z_[3]));
}

void UnderactuatedGripper4Spring::getFingerForce(){
    for(int i=0;i<4; i++){
        finger_force_(i,0) = sqrt( pow(forces_1_x_[i] + forces_2_x_[i],2) + pow(forces_1_z_[i] + forces_2_z_[i],2) );
    }
}

void UnderactuatedGripper4Spring::getInteractionForces(){
    double fh1, fh2, fh3, fh4;
    double fv1, fv2, fv3, fv4;
    double raw_z;

    fh1 = forces_1_x_[0] + forces_2_x_[0]; fv1 = forces_1_z_[0] + forces_2_z_[0]; 
    fh2 = forces_1_x_[1] + forces_2_x_[1]; fv2 = forces_1_z_[1] + forces_2_z_[1];
    fh3 = forces_1_x_[2] + forces_2_x_[2]; fv3 = forces_1_z_[2] + forces_2_z_[2];
    fh4 = forces_1_x_[3] + forces_2_x_[3]; fv4 = forces_1_z_[3] + forces_2_z_[3]; 
    
    //std::cout << " fh1: " << fh1 <<  " fh2: " << fh2 << " fh3: " << fh3 << " fh4: " << fh4 << std::endl; 
    /*raw_z = -(fv1 + fv2 + fv3 + fv4);

    if(raw_z == 0){
        tara_z_ = 0;
        tara_timer_ = 0;
    }else if(tara_z_ == 0){
        tara_timer_++;
    }
    if(tara_timer_ == 200){
        tara_z_ = raw_z;
        tara_timer_ = 0;
    }

    interaction_forces_(0) = fh1 + fh2 + fh3 + fh4;//f_x
    interaction_forces_(1) = raw_z - tara_z_;//f_z
    interaction_forces_(2) = D_ * (fh1 + fh2) -D_ * (fh3 + fh4);//tau_z */
    interaction_forces_(0) = ((forces_1_x_[0]+forces_1_x_[3]+forces_2_x_[0]+forces_2_x_[3]) + (forces_1_x_[1]+forces_1_x_[2]+forces_2_x_[1]+forces_2_x_[2]));
    
    interaction_forces_(1) = ((forces_1_z_[0]+forces_1_z_[1]+forces_1_z_[2]+forces_1_z_[3]) - (forces_2_z_[0]+forces_2_z_[1]+forces_2_z_[2]+forces_2_z_[3]));
    
    interaction_forces_(2) = -(-forces_1_x_[0]-forces_2_x_[0]+forces_1_x_[2]+forces_2_x_[2])*D_/2 + (forces_1_x_[1]+forces_2_x_[1]-forces_1_x_[3]-forces_2_x_[3])*D_/2;

}

    

bool UnderactuatedGripper4Spring::setGraspingForce(float force){
    Eigen::Vector4d a0,a1,a2;
    bool achieved = false;
    //Eigen::Matrix<double, 4,1> measured_force;
    Eigen::Matrix<double, 4,1> identity_vector = {1,1,1,1};

    a0 = Kp_ + ts_ * Ki_;// + Kd_/ts_;
    a1 = Kp_;// + 2*Kd_/ts_;
    a2.setZero();//Kd_/ts_;

    // Compute cartesian forces for each finger and grasping force
    /*getFingersCartesianForce();
    getGraspingForce();*/

    // Compute the norm of the grasping force
    // measured_force = grasp_force_.norm();
    /*for(int i=0;i<4; i++){
        measured_force(i,0) = sqrt( pow(forces_1_x_[i] + forces_2_x_[i],2) + pow(forces_1_z_[i] + forces_2_z_[i],2) );
    }*/

    // Compute error
    error_.block<4,1>(0,2) = error_.block<4,1>(0,1);
    error_.block<4,1>(0,1) = error_.block<4,1>(0,0);
    error_.block<4,1>(0,0) = force/2*identity_vector - finger_force_; //measured_force;

    //if(abs(error_[0]) < 0.2){
    //    achieved = true;
    //} else{
        // Use PID controller
        for(int i=0; i<4; i++){
            pid_pwm_(i) += (int)(a0(i) * error_(i,0) - a1(i) * error_(i,1) + a2(i) * error_(i,2));

            if(pid_pwm_(i) > pwm_saturation_){
                pid_pwm_(i) = pwm_saturation_;
            } else if(pid_pwm_(i) < -pwm_saturation_){
                pid_pwm_(i) = -pwm_saturation_;
            }

            std::cout << i << " Output: " << pid_pwm_(i) << " error " << error_(i,0) << std::endl;
        }

        actuateGripper(pid_pwm_(0),pid_pwm_(1),pid_pwm_(2),pid_pwm_(3));
        //actuateGripper(-10,-10,-10,pid_pwm_(3));
        //actuateGripper(0,0,0,0);
    //}

    return achieved;
}


bool UnderactuatedGripper4Spring::setGraspingForceTDC(float force){
    // chi = 1, omega_n = 4 => kd = 8, kp = 16, 
    double L, chi, omega_n, kp, kd, g_inv; // L = sampling time
    double Fcut;
    Eigen::Matrix4d K, Td, Ti; // derivative and integral time of the PID
    Eigen::Matrix<double, 4,1> identity_vector = {1,1,1,1};
    Eigen::Matrix<double, 4,1> control_action;
    Eigen::Matrix<double, 4,4> I4;
    
    Fcut = 10;

    // Behavioral constants
    I4 << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    L = 0.01; chi = 1; omega_n = 1;
    g_inv = 0.00025;
    // TDC parameters
    kp = pow(omega_n,2);
    kd = 2*chi*omega_n;
    // Equivalent PID parameters
    Td = 1/kd * I4;
    Ti = kd/kp * I4;
    K = g_inv*kd/L * I4;
    //K = Fcut*g_inv*kd/(1+Fcut*L) * I4; // with lowpass filter

    // Compute error
    error_.block<4,1>(0,2) = error_.block<4,1>(0,1);
    error_.block<4,1>(0,1) = error_.block<4,1>(0,0);
    error_.block<4,1>(0,0) = force/2*identity_vector - finger_force_; //measured_force;

    control_action = L*K*(Td*(error_.block<4,1>(0,0) - 2*error_.block<4,1>(0,1) + error_.block<4,1>(0,2))/pow(L,2) + (error_.block<4,1>(0,0) - error_.block<4,1>(0,1))/L + Ti.inverse()*error_.block<4,1>(0,0));

    for(int i=0; i<4; i++){
        pid_pwm_(i) += (int)control_action(i);

        if(pid_pwm_(i) > pwm_saturation_){
                pid_pwm_(i) = pwm_saturation_;
            } else if(pid_pwm_(i) < -pwm_saturation_){
                pid_pwm_(i) = -pwm_saturation_;
            }

        std::cout << i << " Output: " << pid_pwm_(i) << " error " << error_(i,0) << std::endl;
    }
    actuateGripper(pid_pwm_(0),pid_pwm_(1),pid_pwm_(2),pid_pwm_(3));

    return false;
}

void UnderactuatedGripper4Spring::inverseModel(double ft){
    double a, b, c;
    double K1,K2,K3;
    Eigen::Matrix2d frel, T, J;
    Eigen::Vector2d P2, P3;
    double theta_c, j, xs1, h;
    double theta_2_prima, contact_point2_prima, tau_2_prima;
    double f1, f2;

    ft /= 2;
    frel.setZero();

    for(int i=0; i<4; i++){
        // Compute finger state
        P2 << a_*cos(theta_1_[i]) + b_*cos(theta_1_[i] + theta_2_[i] - psi_) , a_*sin(theta_1_[i]) + b_*sin(theta_1_[i] + theta_2_[i] - psi_);
        P3 << e_*cos(-gamma_) + d_*cos(theta_a_[i]), e_*sin(-gamma_) + d_*sin(theta_a_[i]);
        theta_c = atan2(P2(1)-P3(1), P2(0)-P3(0));
        j = P2.norm();
        //j = sqrt(pow(a_,2) + pow(b_,2) + 2 * a_ * b_ * sin(theta_2_[i]));
    
        // Compute tau_2_prima            
        xs1 = a_ * sin(theta_1_[i] - atan2(P2(1),P2(0))); 
        tau_2_prima = ks1_ * (ls1r_ - j) * xs1;
        // Compose transmision matrix
        h = b_ * sin(M_PI + theta_1_[i] + theta_2_[i] - psi_ - theta_c)/sin(theta_c - theta_1_[i]);
        T(0,0) = 1;  T(0,1) = -h/(h+a_);
        T(1,0) = 0;  T(1,1) = 1;
        // correccion del punto de contacto y angulo theta2, espesor de las falanges no despreciable
        //theta_2_prima = theta_2_[i] + atan(w_/contact_point2_);
        //contact_point2_prima = sqrt(pow(contact_point2_,2) + pow(w_,2));
        J(0,0) = contact_point1_;                           J(0,1) = 0;
        J(1,0) = contact_point2_ + a_ * cos(theta_2_[i]);   J(1,1) = contact_point2_;


        // Computation of the forces at phalanx 1 and 2
        frel = (T*J).transpose();
        K1 = frel(1,0); K2 = frel(1,1);
        K3 = cos(-theta_2_[i]);

        //std::cout << "frel: " << frel << std::endl;

        std::cout << "K1: " << K1 << "  K2: " << K2 << std::endl;

        a = pow(K2,2)/pow(K1,2) + 1;
        b = 2*K3*(tau_2_prima-K2)/K1 - 2*tau_2_prima*K2/pow(K1,2);
        c = pow(tau_2_prima,2)/pow(K1,2) - pow(ft,2);

        f2 = (-b + sqrt(pow(b,2) - 4*a*c))/(2*a);
        f1 = (tau_2_prima - f2*K2)/K1;
        tau_0_(i) = frel(1,1)*f1 + frel(1,2)*f2;
    }

    std::cout << "tau_0 Desired: " << tau_0_.transpose() << std::endl;
}

void UnderactuatedGripper4Spring::update(){

    getFingersCartesianForce();
    getGraspingForce();
    getInteractionForces();
    getFingerForce();

    //std::cout << interaction_forces_.transpose() << std::endl;

    // Publish data
    geometry_msgs::WrenchStamped int_msg;
    int_msg.header.stamp = ros::Time::now();
    int_msg.header.frame_id = "gripperCenterPoint";
    int_msg.wrench.force.x = 0;
    int_msg.wrench.force.y = interaction_forces_(0);
    int_msg.wrench.force.z = interaction_forces_(1);
    int_msg.wrench.torque.x = 0;
    int_msg.wrench.torque.y = 0;
    int_msg.wrench.torque.z = interaction_forces_(2);
    interaction_forces_pub_.publish(int_msg);

}

// Main for standalone execution
/*int main(int argc, char **argv){

    ros::init(argc, argv, "TEST");
    ros::NodeHandle node_handle;
    double initial_time;
    ros::Rate rate(100);

    // Matlogger params
    XBot::MatLogger2::Ptr logger_;
    XBot::MatAppender::Ptr appender_;

    // Initialize matlogger
    std::string log_path;
    const char* home_path = getenv("HOME");
    log_path.append(home_path);
    log_path.append("/rosWorkspace/catkin_ws/log/");
    log_path.append("stepTestOpenLoop");

    logger_ = XBot::MatLogger2::MakeLogger(log_path);
    appender_ = XBot::MatAppender::MakeInstance();
    appender_->add_logger(logger_);
    appender_->start_flush_thread();

    UnderactuatedGripper4Spring gripper(&node_handle);

    int timer_test = 0;

    initial_time = ros::Time::now().toSec();
    int pwmtest = 200;
    bool bef = false;
    double ref = 20;
    while(ros::ok()){

        /*if(timer_test == 400){
            timer_test = 0;
            bef = !bef;
            if(bef){
                pwmtest = 350;
            }else{
                pwmtest = 100;
            }
            gripper.actuateGripper(pwmtest,pwmtest,pwmtest,pwmtest);
        }*/ /*

        gripper.getFingersCartesianForce();
        gripper.getGraspingForce();
        gripper.getInteractionForces();
        gripper.getFingerForce();

        //gripper.setGraspingForce(ref);
        gripper.actuateGripper(pwmtest,pwmtest,pwmtest,pwmtest);

        logger_->add("t",ros::Time::now().toSec()-initial_time);
        logger_->add("y",gripper.grasp_force_.norm());
        //logger_->add("pid_out",gripper.pid_pwm_);
        logger_->add("pwmtest",pwmtest);
        //logger_->add("ref",ref);
        logger_->add("finger_force",gripper.finger_force_);
        for(int i=0;i<4;i++){
            logger_->add("theta_a_"+std::to_string(i),gripper.theta_a_[i]);
            logger_->add("theta_1_"+std::to_string(i),gripper.theta_1_[i]);
            logger_->add("theta_2_"+std::to_string(i),gripper.theta_2_[i]);
        }


        //std::cout << "ref: " << ref << " gforce: " << gripper.grasp_force_.norm() << std::endl;
        std::cout << "Interaction Forces: " << gripper.interaction_forces_.transpose() << std::endl;
        timer_test++;

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}*/


// Main for shared control velocity
/*int main(int argc, char **argv){

    ros::init(argc, argv, "TEST");
    ros::NodeHandle node_handle;
    double initial_time;
    ros::Rate rate(100);
    float ref = 15;
    ros::Publisher ffranka_pub_;

    ffranka_pub_ = node_handle.advertise<geometry_msgs::TwistStamped>("/robot_vel_ctrl/vel_cmd",1);

    // Matlogger params
    XBot::MatLogger2::Ptr logger_;
    XBot::MatAppender::Ptr appender_;

    // Initialize matlogger
    std::string log_path;
    const char* home_path = getenv("HOME");
    log_path.append(home_path);
    log_path.append("/gitrepos/underactuated-gripper/data/");
    log_path.append("gripper_4_springs");

    logger_ = XBot::MatLogger2::MakeLogger(log_path);
    appender_ = XBot::MatAppender::MakeInstance();
    appender_->add_logger(logger_);
    appender_->start_flush_thread();

    UnderactuatedGripper4Spring gripper(&node_handle);
    initial_time = ros::Time::now().toSec();
    
    int openpwm = 0;
    int timer = 0; 
    double fv_cte = -0.8; // YZ:10, T:1

    while(ros::ok()){
        geometry_msgs::TwistStamped vmsg;
        vmsg.header.stamp = ros::Time::now();
        

        if(timer == 200){
            //timer = 0;
            openpwm=200;
            //if(openpwm > 350)
            //    openpwm = 50;
            gripper.actuateGripper(openpwm,openpwm,openpwm,openpwm);
        }
        
        if(timer > 500){
//            vmsg.twist.linear.y  = fv_cte * gripper.interaction_forces_(0);
//            vmsg.twist.linear.z  = -fv_cte * gripper.interaction_forces_(1);
            vmsg.twist.angular.z = fv_cte * gripper.interaction_forces_(2);
            ffranka_pub_.publish(vmsg);

        } else {
            timer++;
        }

        gripper.update();
//        gripper.setGraspingForce(ref);
//        gripper.inverseModel(ref);

        // Log gripper positions
        for(int i=0;i<4;i++){
            logger_->add("theta_a_"+std::to_string(i),gripper.theta_a_[i]);
            logger_->add("theta_1_"+std::to_string(i),gripper.theta_1_[i]);
            logger_->add("theta_2_"+std::to_string(i),gripper.theta_2_[i]);
        }
        // Log gripper interaction force
        logger_->add("interaction_force",gripper.interaction_forces_);
        logger_->add("franka_force", gripper.ffranka);
        logger_->add("franka_pose", gripper.franka_pose);
        logger_->add("franka_velocity", gripper.franka_vel);
/ *        logger_->add("grasping_force",gripper.grasp_force_.norm());
        logger_->add("finger_force",gripper.finger_force_);
        logger_->add("global_ref",ref);
        logger_->add("finger_ref",ref/2);
* /      logger_->add("time",ros::Time::now().toSec()-initial_time);
/ *        logger_->add("pid_out",gripper.pid_pwm_);
       // logger_->add("pwm_openloop",openpwm);
        logger_->add("error_k",gripper.error_.block<4,1>(0,0));
        logger_->add("fsensor",gripper.fsensor1_+gripper.fsensor2_);
* /
       // std::cout << "PWM: " << openpwm << "  timer: " << timer << std::endl;
       // std::cout << "sensor: " << gripper.fsensor1_+gripper.fsensor2_ << std::endl;

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}  */


// Main for shared control velocity V2
int main(int argc, char **argv){

    ros::init(argc, argv, "TEST");
    ros::NodeHandle node_handle;
    double initial_time;
    ros::Rate rate(100);
    float ref = 15;

    bool record = false;

    // Matlogger params
    XBot::MatLogger2::Ptr logger_;
    XBot::MatAppender::Ptr appender_;

    // Initialize matlogger
    std::string log_path;
    const char* home_path = getenv("HOME");
    log_path.append(home_path);
    log_path.append("/gitrepos/underactuated-gripper/data/");
    log_path.append("gripper_4_springs");

    if(record){
    logger_ = XBot::MatLogger2::MakeLogger(log_path);
    appender_ = XBot::MatAppender::MakeInstance();
    appender_->add_logger(logger_);
    appender_->start_flush_thread();
    }

    UnderactuatedGripper4Spring gripper(&node_handle);
    initial_time = ros::Time::now().toSec();
    
    int openpwm = 0;
    double fv_cte = -0.8; // YZ:10, T:1

    while(ros::ok()){

        gripper.update();
//        gripper.setGraspingForce(ref);
//        gripper.inverseModel(ref);

        if(record){
        // Log gripper positions
        for(int i=0;i<4;i++){
            logger_->add("theta_a_"+std::to_string(i),gripper.theta_a_[i]);
            logger_->add("theta_1_"+std::to_string(i),gripper.theta_1_[i]);
            logger_->add("theta_2_"+std::to_string(i),gripper.theta_2_[i]);
        }
        // Log gripper interaction force
        logger_->add("interaction_force",gripper.interaction_forces_);
//        logger_->add("franka_force", gripper.ffranka);
//        logger_->add("franka_pose", gripper.franka_pose);
//        logger_->add("franka_velocity", gripper.franka_vel);
//        logger_->add("grasping_force",gripper.grasp_force_.norm());
//        logger_->add("finger_force",gripper.finger_force_);
//        logger_->add("global_ref",ref);
//        logger_->add("finger_ref",ref/2);
//        logger_->add("time",ros::Time::now().toSec()-initial_time);
//        logger_->add("pid_out",gripper.pid_pwm_);
//        logger_->add("pwm_openloop",openpwm);
//        logger_->add("error_k",gripper.error_.block<4,1>(0,0));
//        logger_->add("fsensor",gripper.fsensor1_+gripper.fsensor2_);
        }

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
