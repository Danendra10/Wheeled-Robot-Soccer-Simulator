#ifndef BALL_GAZEBO_HH
#define BALL_GAZEBO_HH

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Gazebo versi 9
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include <angles/angles.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Char.h>
#include <iostream>

#include <geometry_msgs/Pose2D.h>

//-----Team
#define TEAM_CYAN 'c'
#define TEAM_MAGENTA 'm'

//-----Basestation protocol
#define COMMAND_KICKOFF_HOME 'K'
#define COMMAND_KICKOFF_AWAY 'k'
#define COMMAND_FREEKICK_HOME 'F'
#define COMMAND_FREEKICK_AWAY 'f'
#define COMMAND_GOALKICK_HOME 'G'
#define COMMAND_GOALKICK_AWAY 'g'
#define COMMAND_THROWIN_HOME 'T'
#define COMMAND_THROWIN_AWAY 't'
#define COMMAND_CORNER_HOME 'C'
#define COMMAND_CORNER_AWAY 'c'
#define COMMAND_PENALTY_HOME 'P'
#define COMMAND_PENALTY_AWAY 'p'
#define COMMAND_DROPBALL 'N'
#define COMMAND_START 's'
#define COMMAND_STOP 'S'

int robot_selected;

namespace gazebo
{
    class BallGazebo : public ModelPlugin
    {

    public:
        BallGazebo();
        ~BallGazebo();
        //---Gazebo
        void Load(physics::ModelPtr _model, sdf::ElementPtr);
        void OnUpdate();

        void callbackSubCommandBasestation(const std_msgs::CharConstPtr &msg);
        void queueThread();

    private:
        void deselerasiBola();
        void setPosisiBola(char _command, bool _side);

        // Keyboard handler
        int kbhit();

        //---Gazebo
        physics::WorldPtr world;
        physics::ModelPtr ball_model;
        physics::LinkPtr ball_link;
        event::ConnectionPtr update_connection;

        //---ROS NodeHandle
        std::unique_ptr<ros::NodeHandle> ros_node;

        //---ROS Subscriber
        ros::Subscriber sub_command_basestation;

        char command;
        float bola_x;
        float bola_y;

        ros::CallbackQueue ros_queue;
        std::thread ros_queue_thread;
    };
} // namespace gazebo

#endif // BALL_GAZEBO_HH