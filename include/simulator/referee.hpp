#ifndef REFEREE_HPP
#define REFEREE_HPP

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <cstdlib>
#include <fstream>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/UInt8.h>

#include <logger/logger.hpp>

logger::Logger logger_sim;

using namespace gazebo;
using namespace ignition;
using namespace std;

namespace gazebo
{
    class RefereeBox : public WorldPlugin
    {
    public:
        RefereeBox();
        ~RefereeBox();
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
        void OnUpdate();

        void QueueThread();

        ros::Publisher pub_command_referee;

    private:
        event::ConnectionPtr update_connection;
        std::unique_ptr<ros::NodeHandle> ros_node;
        std::thread ros_queue_thread;
        ros::CallbackQueue ros_queue;

        uint8_t command_referee;

        int Kbhit();
        void KeyboardHandler();
        void PublishData();
    };
};

#endif