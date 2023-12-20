#ifndef ROBOT_GAZEBO_NEW_CONTROL_HH
#define ROBOT_GAZEBO_NEW_CONTROL_HH

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
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread/mutex.hpp>
#include <std_msgs/Char.h>
#include <iostream>
#include "simulator/ball_gazebo.hh"
#include "iris_msgs/robot2sim.h"
#include "iris_msgs/robotdata.h"

#include <logger/logger.hpp>

#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780
#define DIV180 0.005555556

#define TOTAL_CYAN 3
#define TOTAL_MAGENTA 3

logger::Logger logger_sim;

using namespace gazebo;
using namespace ignition;
using namespace std;

//-----Model Name
#define MODEL_BALL "bola"
#define MODEL_CYAN1 "cyan1"
#define MODEL_CYAN2 "cyan2"
#define MODEL_CYAN3 "cyan3"
#define MODEL_CYAN4 "cyan4"
#define MODEL_CYAN5 "cyan5"
#define MODEL_MAGENTA1 "magenta1"
#define MODEL_MAGENTA2 "magenta2"
#define MODEL_MAGENTA3 "magenta3"
#define MODEL_MAGENTA4 "magenta4"
#define MODEL_MAGENTA5 "magenta5"

//-----Team Identifier
#define CYAN_TEAM 'c'
#define MAGENTA_TEAM 'm'

#define BALL_DIST 35

math::Pose3d cyan_pose[6];
math::Pose3d magenta_pose[6];

typedef struct
{
    float x;
    float y;
    float th;
} Vect3;

namespace gazebo
{
    class RobotGazeboSecond : public ModelPlugin
    {
    public:
        RobotGazeboSecond();
        ~RobotGazeboSecond();
        void Load(physics::ModelPtr _model, sdf::ElementPtr);
        void OnUpdate();

        //---Ros Subscribers
        void CllbckSubGazeboModel(const gazebo_msgs::ModelStates::ConstPtr &msg);
        void CllbckSubVelocity(const iris_msgs::robot2sim::ConstPtr &msg);
        void CllbckSubKickerPower(const std_msgs::Float32ConstPtr &msg);

        //---Utils
        /**
         * @brief The RobotGazebo::QueueThread() function is intended to run in a separate
         * thread and manages the processing of incoming ROS messages.
         */
        void QueueThread();

    private:
        //---Gazebo variables
        physics::WorldPtr world;
        physics::ModelPtr robot_model;
        std::string robot_model_name;
        event::ConnectionPtr update_connection;
        physics::ModelPtr ball_model;
        unsigned int ball_index;

        //---ROS NodeHandle
        std::unique_ptr<ros::NodeHandle> ros_node;

        //---ROS Subscriber
        ros::Subscriber sub_gazebo_model;
        ros::Subscriber sub_kicker_power;
        ros::Subscriber sub_vel_cyan[4];
        ros::Subscriber sub_vel_magenta[4];

        //---ROS Publisher
        ros::Publisher pub_cyan_pose[4];
        ros::Publisher pub_cyan_ball_state[4];
        ros::Publisher pub_magenta_pose[4];
        ros::Publisher pub_magenta_ball_state[4];
        ros::Publisher pub_ball_pose_cyan;
        ros::Publisher pub_ball_pose_magenta;
        ros::Publisher pub_friend_poses;
        ros::Publisher pub_enemy_poses;
        ros::Publisher pub_total_active_cyan;
        ros::Publisher pub_total_active_magenta;

        //---Query
        ros::CallbackQueue ros_queue;
        std::thread ros_queue_thread;

        /**
         * @brief First array is the robot number
         * @brief Second array is the motor number
         * @param second_array is [left, right, rear]
         */
        Vect3 vel_cyan_motor[TOTAL_CYAN + 1];
        Vect3 final_vel_cyan[TOTAL_CYAN + 1];
        Vect3 cyan_pose[TOTAL_CYAN + 1];

        /**
         * @brief First array is the robot number
         * @brief Second array is the motor number
         * @param second_array is [left, right, rear]
         */
        Vect3 vel_magenta_motor[TOTAL_MAGENTA + 1];
        Vect3 final_vel_magenta[TOTAL_MAGENTA + 1];
        Vect3 magenta_pose[TOTAL_MAGENTA + 1];

        //---Status
        /**
         * @brief To decide the robot in that index is connected with robot code or not
         * if not, it would not give velocity to the robot
         * @param first_array is the robot number
         */
        uint8_t cyan_connection_stat[TOTAL_CYAN + 1];
        float cyan_connection_timer[TOTAL_CYAN + 1];
        /**
         * @brief To decide the robot in that index is connected with robot code or not
         * if not, it would not give velocity to the robot
         * @param first_array is the robot number
         */
        uint8_t magenta_connection_stat[TOTAL_MAGENTA + 1];
        float magenta_connection_timer[TOTAL_MAGENTA + 1];

        /**
         * Decide how much robot is active to tell workspace to decide between double and single
         */
        uint8_t active_cyan;
        uint8_t active_magenta;

        Vect3 ball_pose;
        Vect3 ball_pose_on_real;
        uint8_t ball_status;
        float kicker_power;

        //---Functions
        void RobotControl();
        void ComputeRobotPosition();
        /**
         * @brief Remember that X in our real world format is diffrent
         * from X in gazebo
         *
         * In gazebo
         *
         * y
         *  _
         * / \
         *  |
         *  |
         *  |
         *  |__________ x
         *
         * In robot
         *
         * x
         *  _
         * / \
         *  |
         *  |
         *  |
         *  |__________ y
         *
         */
        void ComputeRobotVelocity();
        void ComputeBallPosition();
        void ComputeBallVelocity();
        void PublishData();
        void MoveRobot(uint8_t _coderobot, float _vx, float _vy, float _vsudut);
        bool CheckBallStatus(Vect3 robot_pose);
        void BallDribbling(Vect3 robot_pose, string robot_model_name);
        void Kick(uint8_t mode_, float force, Vect3 robot_pose);
    };
}

#endif
