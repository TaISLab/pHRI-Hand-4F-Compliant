
#include "underactuated_gripper_4_spring.cpp"
#include <ros/ros.h>

#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "TEST");
    ros::NodeHandle node_handle;
    double initial_time;
    ros::Rate rate(100);
    char unused;

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

    //UnderactuatedGripper4Spring gripper(&node_handle);
    UnderactuatedGripper4Spring gripper;
    gripper.init(&node_handle);

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
        }*/ 

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
}