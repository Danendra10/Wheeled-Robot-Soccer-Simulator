#ifndef ROBOT_GAZEBO_HH
#define ROBOT_GAZEBO_HH

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

logger::Logger logger_sim;

using namespace gazebo;
using namespace ignition;
using namespace std;

// #include "iris_its/vision_ball.h"
// #include "iris_its/VisionBall.h"
// #include "iris_its/vision_obstacle.h"
// #include "iris_its/VisionObstacle.h"

//-----Nama Model
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

//-----Team
#define TEAM_CYAN 'c'
#define TEAM_MAGENTA 'm'

#define JARAK_BOLA 35

char _command;
extern int robot_selected;

math::Pose3d pose_cyan[6];
math::Pose3d pose_magenta[6];

namespace gazebo
{
    class RobotGazebo : public ModelPlugin
    {

    public:
        RobotGazebo();
        ~RobotGazebo();
        //---Gazebo
        void Load(physics::ModelPtr _model, sdf::ElementPtr);
        void OnUpdate();

        void callbackSubGazeboModel(const gazebo_msgs::ModelStates::ConstPtr &msg);
        void callbackSubVelocity(const iris_msgs::robot2sim::ConstPtr &msg);
        void callbackSubVelocity2(const iris_msgs::robot2sim::ConstPtr &msg);
        void callbackSubKicker(const std_msgs::Int16MultiArray::ConstPtr &msg);
        void callbackSubLidar(const sensor_msgs::LaserScan::ConstPtr &msg);
        void changeRobot(char _command);
        int kbhit();

        void queueThread();

    private:
        void olahDataPosisiRobot();
        void olahDataKecepatanRobot();

        void olahDataPosisiBola();
        void olahDataKecepatanBola();

        void jalanManual(int coderobot, float _vx, float _vy, float _vsudut);
        void jalanSemua(int coderobot);
        void controlRobot();
        bool cekStatusBola();
        void giringBola();
        void tendang(unsigned char _mode, unsigned int _kecepatan);

        void collisionHandling();
        void publishData();

        //---Gazebo
        physics::WorldPtr world;
        physics::ModelPtr robot_model;
        std::string robot_model_name;
        event::ConnectionPtr update_connection;

        physics::ModelPtr ball_model;
        unsigned int ball_index;

        //---ROS NodeHandle
        std::unique_ptr<ros::NodeHandle> ros_node;

        //---ROS Publisher
        ros::Publisher pub_odometry;
        ros::Publisher pub_ball_pose_cyan;
        ros::Publisher pub_ball_pose_magenta;
        ros::Publisher pub_handle_ball;
        ros::Publisher pub_cyan[6];
        ros::Publisher pub_magenta[6];

        //---ROS Subscriber
        ros::Subscriber sub_gazebo_model;
        ros::Subscriber sub_lidar;
        ros::Subscriber sub_velocity_cyan[6];
        ros::Subscriber sub_velocity_magenta[6];
        ros::Subscriber sub_kicker;

        ros::CallbackQueue ros_queue;
        std::thread ros_queue_thread;

        // ---Odometry
        float pos_x;
        float pos_y;
        float theta;

        // ODOMETRY
        float cyan[6][3];
        float magenta[6][3];

        //---Status bola
        unsigned char status_bola_dribble;
        unsigned char status_bola_vision;

        float bola_x_buffer;
        float bola_y_buffer;

        //---Ball on frame virtual
        float bola_x_pada_frame;
        float bola_y_pada_frame;
        float bola_theta_pada_frame;

        //---Ball on field
        float bola_x_pada_lapangan;
        float bola_y_pada_lapangan;
        float bola_theta_pada_lapangan;

        //---Obstacle
        unsigned short int halangan[144];

        //---Velocity

        float buffer_v_cyan[6][3];
        float v_cyan[6][3];

        float buffer_v_magenta[6][3];
        float v_magenta[6][3];

        float vx, vx_buffer = 0;
        float vy, vy_buffer = 0;
        float vsudut, vsudut_buffer = 0;

        //-----Penendang
        bool status_tendang = false;
        unsigned char mode_tendang = 0;
        unsigned int kecepatan_tendang = 1500;

        //---Status

        double timer_terima_cyan[6];
        bool status_terima_cyan[6] = {false, false, false, false, false, false};

        double timer_terima_magenta[6];
        bool status_terima_magenta[6] = {false, false, false, false, false, false};

        double timer_terima_pc;
        bool status_terima_pc = false;
    };
} // namespace gazebo

#endif // ROBOT_GAZEBO_HH