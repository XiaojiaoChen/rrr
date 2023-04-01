#ifndef __KN800DRIVER_H_
#define __KN800DRIVER_H_

#include <thread>
#include <string>
#include <queue>
#include <ros/ros.h>

#include "kn800Interface.h"
#include <wisson_kn800/kn800Command.h>
#include "sensor_msgs/JointState.h"
 
  
namespace WISSON_ROBOTICS
{
    #define KN800_DOF 4 
    class KN800Driver
    {
        public:
            KN800Driver(char * port_name);
            ~KN800Driver();

            void autoConnect();
            int lostSeconds();
 
            const int UPDATE_RATE_ = 500;
            const int TIMER_SPAN_ = 50;

        public:
            int buffer_size_;
            KN800Arm _kn800_service;

            sensor_msgs::JointState joint_state_;
        private:
            
            void subCommandCallback(const wisson_kn800::kn800Command::ConstPtr &msg);

            void pubTimerCallback(const ros::TimerEvent& e);


            bool controller_connected_flag_;
 
            static std::string joint_name_[KN800_DOF];

            ros::NodeHandle nh_;
            ros::Publisher _pub;
            ros::Subscriber  _sub; 
            ros::Timer timer_;
            int timeout_;

    };
}

#endif 
