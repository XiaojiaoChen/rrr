#include "kn800Driver.h"

namespace WISSON_ROBOTICS {

    std::string KN800Driver::joint_name_[KN800_DOF] = {"elongation","orientation","bending","grasp"};

    KN800Driver::KN800Driver(char * port_name)
    {
        joint_state_.name.resize(KN800_DOF);
        joint_state_.name[0]="elongation";
        joint_state_.name[1]="orientation";
        joint_state_.name[2]="bending";
        joint_state_.name[3]="grasp";
        joint_state_.position.resize(KN800_DOF);

    
        /** publish messages **/
        _pub = nh_.advertise<sensor_msgs::JointState>("kn800_joint_states", 300);
        // _pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 300);

        /** subscribe messages **/
        _kn800_service.setName(port_name);
        _sub = nh_.subscribe("kn800_command", 10, &KN800Driver::subCommandCallback, this);

        timer_ = nh_.createTimer(ros::Duration(1.0 / TIMER_SPAN_), &KN800Driver::pubTimerCallback, this);

        controller_connected_flag_  = false;
        ROS_INFO("Initializing KN800 Driver.");
    }

    KN800Driver::~KN800Driver()
    {

        ROS_INFO("Closing KN800Driver!");
        controller_connected_flag_  == false;
        _kn800_service.disconnectArm();
    }

    void KN800Driver::pubTimerCallback(const ros::TimerEvent& e)
    {
        if(controller_connected_flag_)
        {
            /** Query the states of robot joints **/
            int ret = _kn800_service.updateState();     

            joint_state_.header.stamp = ros::Time::now();
            joint_state_.position[0] = _kn800_service._state.elongation/1000.0;
            joint_state_.position[1] = -1*_kn800_service._state.orientation*3.14/100.0/180.0;
            joint_state_.position[2] = _kn800_service._state.bending*3.14/100.0/180.0;
            joint_state_.position[3] = 1.26+(-0.19-1.26)/(3.14*130/180)*(_kn800_service._state.grasp*3.14/100.0/180.0);
        
            _pub.publish(joint_state_);
    
        }
    }

    
    void KN800Driver::subCommandCallback(const wisson_kn800::kn800Command::ConstPtr &msg)
    {
        //ROS_INFO("Subcribed command:  elongate:%d bend_direction:%d   bend:%d  grasp:%d", msg->elongationCommandBinary, msg->graspingCommandBinary);
        if(controller_connected_flag_)
        {
            int ret = _kn800_service.updateCommand( msg->compositeCommand,
                                                    msg->elongationCommandBinary,
                                                    msg->orientationCommand,
                                                    msg->bendingCommandBinary,
                                                    msg->graspingCommandBinary);
        }

    }

    /**
     * return the last unconnected period in seconds.
    */
    void KN800Driver::autoConnect()
    {
 
        if(controller_connected_flag_  == false){
            
            if(timeout_%1000==0){
                ROS_INFO("Connecting to kn800 at %s",_kn800_service.port_name_);
                int ret = _kn800_service.connectArm();
                if(ret == 0)
                {
                    controller_connected_flag_  = true;
                    ROS_INFO("connection successful!");
                    timer_.start();
                    timeout_=0;
                }
                else
                {        
 
                    controller_connected_flag_  = false;
                    ROS_INFO("connection failed!");
                    timer_.stop();
                    timeout_+= 1000/UPDATE_RATE_;
                }
            }
            else{
                timeout_+= 1000/UPDATE_RATE_;
            }
        }
        else{
            timeout_=0;
        }
    }

    int KN800Driver::lostSeconds(){
        return timeout_/1000;
    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WISSON_ROBOTICS");
    const char* port_name_temp;
    if(argc>=1){
        port_name_temp = argv[1];
    }
    else{
        port_name_temp = "/dev/ttyUSB0";
    }

    WISSON_ROBOTICS::KN800Driver kn800_driver((char *)port_name_temp);
 
    ros::AsyncSpinner spinner(6);
    spinner.start();
    ros::Rate loop_rate(kn800_driver.UPDATE_RATE_);
    while(ros::ok())
    {
 
        //No connection for 10 s would terminate this node
        kn800_driver.autoConnect();
        if(kn800_driver.lostSeconds()>10){
            break;
        };
 
        loop_rate.sleep();
        ros::spinOnce();
    }

    ROS_WARN("Exiting kn800_driver");
    return(0);
}