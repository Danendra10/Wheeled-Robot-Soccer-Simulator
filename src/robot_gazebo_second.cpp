#include <simulator/robot_gazebo_second.hpp>

unsigned int FIELD_X = 1200;
unsigned int FIELD_Y = 800;

GZ_REGISTER_MODEL_PLUGIN(RobotGazeboSecond)

RobotGazeboSecond::RobotGazeboSecond()
{
}

RobotGazeboSecond::~RobotGazeboSecond()
{
    update_connection.reset();
    ros_node->shutdown();
}

void RobotGazeboSecond::Load(physics::ModelPtr _model, sdf::ElementPtr)
{
    // Store the pointer to the model
    robot_model = _model;

    // Condition where the pointer is null
    // The gazebo will retry to load the model until it finds it
    if (!robot_model)
    {
        ROS_ERROR("Null pointer access: robot_model is null.");
        return;
    }

    // Get the world pointer
    world = _model->GetWorld();

    // Get the name of the model ex: cyan1
    robot_model_name = _model->GetName();

    // Get the ball model by the ball model name, "ball"
    ball_model = world->ModelByName(MODEL_BALL);

    // Condition where the ball model is null
    if (!ball_model)
    {
        ROS_FATAL("Robot %s can't detect the ball", robot_model_name.c_str());
        return;
    }

    // Iterate through all the model to find the ball
    // We do this so the index of the ball can be found
    // And it would help the post processing to be faster
    for (int i = 0; i < world->ModelCount(); i++)
    {
        // Getting the model by index
        // Reference: https://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1physics_1_1World.html#aa74c0e9b03a030b69bd5c475b1367b2e
        if (world->ModelByIndex(i) == ball_model)
        {
            ball_index = i;
            break;
        }
    }

    for (int i = 0; i < TOTAL_CYAN + 1; i++)
        cyan_connection_stat[i] = false;
    for (int i = 0; i < TOTAL_MAGENTA + 1; i++)
        magenta_connection_stat[i] = false;

    // This event is broadcast every simulation iteration
    // Reference: https://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1event_1_1Events.html#a0c8c7e6a33ad46f8be45b6a33d279fdc
    update_connection = event::Events::ConnectWorldUpdateBegin(
        // std::bind is used to bind the function to the class
        // Reference: https://en.cppreference.com/w/cpp/utility/functional/bind
        std::bind(&RobotGazeboSecond::OnUpdate, this));

    // Check if ROS is not being initialized
    // If not then initialize it
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        // Ros Init Option NoSigintHandler means that the program will not terminate when ctrl + c is pressed
        // Reference: https://docs.ros.org/en/api/roscpp/html/namespaceros_1_1init__options.html
        ros::init(argc, argv, "robot_gazebo", ros::init_options::NoSigintHandler);
    }

    /**
     * This line is NECESSARY for a gazebo plugins
     * Node Handle Creation: The ros::NodeHandle is an essential object in ROS that handles
     * the initialization and shutdown of the node within the ROS environment. It's used for
     * subscribing to topics, advertising services, and interacting with parameters on the parameter server.
     *
     * Scoped Node Handle: When you use reset on a boost::shared_ptr or std::shared_ptr (depending on your version of ROS),
     * you are effectively saying that you want to replace the managed object with a new one. In this case, it replaces the
     * current ros::NodeHandle object with a new instance of ros::NodeHandle. This is important for ensuring that the node handle
     * is properly scoped within the lifecycle of the plugin. When the boost::shared_ptr (or std::shared_ptr) goes out of scope,
     * it ensures the destruction of the ros::NodeHandle, which will cleanly shutdown all the ROS communications set up through this handle.
     *
     * Unique Namespace: By specifying "robot_gazebo" as the namespace for the node handle, you are ensuring that this plugin's node handle
     * interacts with ROS under a specific namespace, which can help in managing topics, services, and parameters, especially when you have
     * multiple instances of the same plugin or nodes in a larger system.
     *
     * Lazily Initialized ROS Node: The check if (!ros::isInitialized()) before initializing the ROS node is there to prevent re-initialization
     * if the ROS system is already up and running. This is a safety measure to ensure that you do not accidentally re-initialize ROS, which would
     * be an error. The ros::NodeHandle constructor does not initialize ROS; it assumes ROS has already been initialized, hence the check before
     * the node handle is created.
     *
     * Note: Explained by CHATGPT
     */
    ros_node.reset(new ros::NodeHandle("robot_gazebo"));

    //---Ros Subscribers

    ros::SubscribeOptions so_gazebo_model =
        ros::SubscribeOptions::create<gazebo_msgs::ModelStates>("/gazebo/model_states",
                                                                16,
                                                                boost::bind(&RobotGazeboSecond::CllbckSubGazeboModel, this, _1),
                                                                ros::VoidPtr(),
                                                                &ros_queue);

    sub_gazebo_model = ros_node->subscribe(so_gazebo_model);

    ros::SubscribeOptions so_kicker_power =
        ros::SubscribeOptions::create<std_msgs::Float32>("/pc2sim/kicker/power",
                                                         16,
                                                         boost::bind(&RobotGazeboSecond::CllbckSubKickerPower, this, _1),
                                                         ros::VoidPtr(),
                                                         &ros_queue);

    sub_kicker_power = ros_node->subscribe(so_kicker_power);

    for (uint8_t i = 1; i < TOTAL_CYAN + 1; i++)
    {
        string topic_name = "/cyan" + std::to_string(i) + "/pc2sim/velocity";
        ros::SubscribeOptions so_vel_cyan =
            ros::SubscribeOptions::create<iris_msgs::robot2sim>(topic_name,
                                                                16,
                                                                boost::bind(&RobotGazeboSecond::CllbckSubVelocity, this, _1),
                                                                ros::VoidPtr(),
                                                                &ros_queue);

        sub_vel_cyan[i] = ros_node->subscribe(so_vel_cyan);

        topic_name = "/cyan" + to_string(i) + "/odom";
        pub_cyan_pose[i] = ros_node->advertise<geometry_msgs::Pose2D>(topic_name, 16);

        topic_name = "/cyan" + to_string(i) + "/ball/state";
        pub_cyan_ball_state[i] = ros_node->advertise<std_msgs::UInt8>(topic_name, 16);
    }

    for (uint8_t i = 1; i < TOTAL_MAGENTA + 1; i++)
    {
        string topic_name = "/magenta" + std::to_string(i) + "/pc2sim/velocity";
        ros::SubscribeOptions so_vel_magenta =
            ros::SubscribeOptions::create<iris_msgs::robot2sim>(topic_name,
                                                                16,
                                                                boost::bind(&RobotGazeboSecond::CllbckSubVelocity, this, _1),
                                                                ros::VoidPtr(),
                                                                &ros_queue);

        sub_vel_magenta[i] = ros_node->subscribe(so_vel_magenta);

        topic_name = "magenta" + to_string(i) + "/odom";
        pub_magenta_pose[i] = ros_node->advertise<geometry_msgs::Pose2D>(topic_name, 16);

        topic_name = "/magenta" + to_string(i) + "/ball/state";
        pub_magenta_ball_state[i] = ros_node->advertise<std_msgs::UInt8>(topic_name, 16);
    }

    pub_ball_pose_cyan = ros_node->advertise<geometry_msgs::Pose2D>("/cyan/ball/pose", 16);
    pub_ball_pose_magenta = ros_node->advertise<geometry_msgs::Pose2D>("/magenta/ball/pose", 16);

    pub_friend_poses = ros_node->advertise<std_msgs::Float32MultiArray>("/cyan/poses", 16);
    pub_enemy_poses = ros_node->advertise<std_msgs::Float32MultiArray>("/magenta/poses", 16);

    pub_total_active_cyan = ros_node->advertise<std_msgs::UInt8>("/cyan/total/robot/active", 16);
    pub_total_active_magenta = ros_node->advertise<std_msgs::UInt8>("/magenta/total/robot/active", 16);

    /**
     * When std::bind(&RobotGazeboSecond::QueueThread, this) is passed to the std::thread constructor,
     * it tells the new thread to execute the QueueThread member function on the current instance of the
     * RobotGazeboSecond class. Essentially, it sets up a thread that will handle tasks defined within
     * the QueueThread function, which is often related to managing a queue of messages or events in the context of a ROS node.
     *
     * Reference to thread: https://en.cppreference.com/w/cpp/thread/thread
     */
    ros_queue_thread = std::thread(std::bind(&RobotGazeboSecond::QueueThread, this));
}

// Main Loop
void RobotGazeboSecond::OnUpdate()
{
    if (world->ModelCount() > 6)
    {
        RobotControl();

        for (int i = 1; i < TOTAL_CYAN + 1; i++)
            if (ros::Time::now().toSec() - cyan_connection_timer[i] > 1)
                cyan_connection_stat[i] = false;

        for (int i = 1; i < TOTAL_MAGENTA + 1; i++)
            if (ros::Time::now().toSec() - magenta_connection_timer[i] > 1)
                magenta_connection_stat[i] = false;
    }
}

// Thread
void RobotGazeboSecond::QueueThread()
{
    static const double timeout = 0.01;
    while (ros_node->ok())
    {
        ros_queue.callAvailable(ros::WallDuration(timeout));
    }
}

//---Ros Callbacks
void RobotGazeboSecond::CllbckSubGazeboModel(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    if (ball_index >= msg->pose.size())
        return;

    if (world->ModelCount())
    {
        // Virtual ball position
        geometry_msgs::Pose pose = msg->pose[ball_index];
        math::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
        ball_pose.x = position.X();
        ball_pose.y = position.Y();

        // Virtual velocity
        geometry_msgs::Twist twist = msg->twist[ball_index];
    }
}

void RobotGazeboSecond::CllbckSubVelocity(const iris_msgs::robot2sim::ConstPtr &msg)
{
    int motor_index = msg->coderobot - 10;
    if (motor_index >= 1 && motor_index <= 5)
    {
        vel_cyan_motor[motor_index].x = msg->vx;
        vel_cyan_motor[motor_index].y = msg->vy;
        vel_cyan_motor[motor_index].th = msg->vth;
        cyan_connection_stat[motor_index] = true;
        cyan_connection_timer[motor_index] = ros::Time::now().toSec();
    }
    else if (motor_index >= 11 && motor_index <= 15)
    {
        motor_index -= 10;
        vel_magenta_motor[motor_index].x = msg->vx;
        vel_magenta_motor[motor_index].y = msg->vy;
        vel_magenta_motor[motor_index].th = msg->vth;
        magenta_connection_stat[motor_index] = true;
        magenta_connection_timer[motor_index] = ros::Time::now().toSec();
    }
}

void RobotGazeboSecond::CllbckSubKickerPower(const std_msgs::Float32ConstPtr &msg)
{
    kicker_power = msg->data;
}

//---Robot Control
void RobotGazeboSecond::RobotControl()
{
    std_msgs::UInt8 total_robot_msg;
    uint8_t robot_code = 10;
    active_cyan = 0;
    active_magenta = 0;
    for (int i = 1; i <= TOTAL_CYAN; i++)
    {
        std_msgs::UInt8 ball_status_msg;
        robot_code++;
        ball_status = 1;
        if (cyan_connection_stat[i])
        {
            active_cyan++;
            ComputeRobotPosition();
            ComputeRobotVelocity();
            ComputeBallPosition();
            if (CheckBallStatus(cyan_pose[i]) && kicker_power == 0)
            {
                BallDribbling(cyan_pose[i], "cyan" + to_string(i));
                ball_status = 2;
            }
            if (kicker_power != 0)
            {
                Kick(1, kicker_power, cyan_pose[i]);
            }

            MoveRobot(robot_code, final_vel_cyan[i].x, final_vel_cyan[i].y, final_vel_cyan[i].th);

            PublishData();
        }
        ball_status_msg.data = ball_status;
        pub_cyan_ball_state[i].publish(ball_status_msg);
    }
    total_robot_msg.data = active_cyan;
    pub_total_active_cyan.publish(total_robot_msg);
    robot_code = 20;
    for (int i = 1; i <= TOTAL_MAGENTA; i++)
    {
        std_msgs::UInt8 ball_status_msg;
        robot_code++;
        ball_status = 1;
        if (magenta_connection_stat[i])
        {
            active_magenta++;
            ComputeRobotPosition();
            ComputeRobotVelocity();
            ComputeBallPosition();
            if (CheckBallStatus(magenta_pose[i]) && kicker_power == 0)
            {
                BallDribbling(magenta_pose[i], "magenta" + to_string(i));
                ball_status = 2;
            }
            if (kicker_power != 0)
            {
                Kick(1, kicker_power, magenta_pose[i]);
            }

            MoveRobot(robot_code, final_vel_magenta[i].x, final_vel_magenta[i].y, final_vel_magenta[i].th);

            PublishData();
        }
        ball_status_msg.data = ball_status;
        pub_magenta_ball_state[i].publish(ball_status_msg);
    }
    total_robot_msg.data = active_magenta;
    pub_total_active_magenta.publish(total_robot_msg);
}

void RobotGazeboSecond::ComputeRobotPosition()
{
    math::Pose3d pose_cyan[TOTAL_CYAN + 1];
    math::Pose3d pose_magenta[TOTAL_MAGENTA + 1];

    for (uint8_t i = 1; i < TOTAL_CYAN + 1; i++)
    {
        string what_robot = "cyan" + to_string(i);
        auto model = world->ModelByName(what_robot);
        if (!model)
            continue;

        pose_cyan[i] = model->RelativePose();
        cyan_pose[i].x = (pose_cyan[i].Pos().Y() * 100);
        cyan_pose[i].y = (pose_cyan[i].Pos().X() * 100);
        cyan_pose[i].th = -angles::to_degrees(pose_cyan[i].Rot().Yaw());

        while (cyan_pose[i].th > 180)
            cyan_pose[i].th -= 360;
        while (cyan_pose[i].th < -180)
            cyan_pose[i].th += 360;
    }
    for (uint8_t i = 1; i < TOTAL_MAGENTA + 1; i++)
    {
        string what_robot = "magenta" + to_string(i);
        auto model = world->ModelByName(what_robot);

        if (!model)
            continue;

        pose_magenta[i] = model->RelativePose();
        magenta_pose[i].x = (pose_magenta[i].Pos().Y() * 100);
        magenta_pose[i].y = (pose_magenta[i].Pos().X() * 100);
        magenta_pose[i].th = -angles::to_degrees(pose_magenta[i].Rot().Yaw());

        while (magenta_pose[i].th > 180)
            magenta_pose[i].th -= 360;
        while (magenta_pose[i].th < -180)
            magenta_pose[i].th += 360;
    }
}

void RobotGazeboSecond::PublishData()
{
    geometry_msgs::Pose2D robot_data;
    std_msgs::Float32MultiArray cyan_positions;
    std_msgs::Float32MultiArray magenta_positions;

    for (int i = 1; i <= TOTAL_CYAN; i++)
    {
        robot_data.x = cyan_pose[i].x;
        cyan_positions.data.push_back(cyan_pose[i].x);
        robot_data.y = cyan_pose[i].y;
        cyan_positions.data.push_back(cyan_pose[i].y);
        robot_data.theta = cyan_pose[i].th;
        pub_cyan_pose[i].publish(robot_data);
    }

    for (int i = 1; i <= TOTAL_MAGENTA; i++)
    {
        robot_data.x = magenta_pose[i].x;
        magenta_positions.data.push_back(magenta_pose[i].x);
        robot_data.y = magenta_pose[i].y;
        magenta_positions.data.push_back(magenta_pose[i].y);
        robot_data.theta = magenta_pose[i].th;
        pub_magenta_pose[i].publish(robot_data);
    }
    pub_friend_poses.publish(cyan_positions);
    pub_enemy_poses.publish(magenta_positions);

    geometry_msgs::Pose2D pose;
    pose.x = ball_pose_on_real.x;
    pose.y = ball_pose_on_real.y;
    pub_ball_pose_cyan.publish(pose);
}

void RobotGazeboSecond::ComputeRobotVelocity()
{
    for (int i = 1; i <= TOTAL_CYAN; i++)
    {
        final_vel_cyan[i].x = (vel_cyan_motor[i].y * 0.05 * cos(angles::from_degrees(cyan_pose[i].th)) +
                               vel_cyan_motor[i].x * 0.05 * sin(angles::from_degrees(cyan_pose[i].th)));
        final_vel_cyan[i].y = -(vel_cyan_motor[i].y * 0.05 * sin(angles::from_degrees(cyan_pose[i].th)) +
                                vel_cyan_motor[i].x * 0.05 * cos(angles::from_degrees(cyan_pose[i].th)));
        final_vel_cyan[i].th = -vel_cyan_motor[i].th * 0.05;
    }

    for (int i = 1; i <= TOTAL_MAGENTA; i++)
    {
        final_vel_magenta[i].x = -(vel_magenta_motor[i].y * 0.05 * cos(angles::from_degrees(magenta_pose[i].th)) +
                                   vel_magenta_motor[i].x * 0.05 * sin(angles::from_degrees(magenta_pose[i].th)));
        final_vel_magenta[i].y = (vel_magenta_motor[i].y * 0.05 * sin(angles::from_degrees(magenta_pose[i].th)) +
                                  vel_magenta_motor[i].x * 0.05 * cos(angles::from_degrees(magenta_pose[i].th)));
        final_vel_magenta[i].th = -vel_magenta_motor[i].th * 0.05;
    }
}

/**
 * @brief This system should be faster than the previous method.
 */
void RobotGazeboSecond::MoveRobot(uint8_t _code_robot, float _vx, float _vy, float _vsudut)
{
    std::string model_name;
    switch (_code_robot)
    {
    case 11:
        model_name = "cyan1";
        break;
    case 12:
        model_name = "cyan2";
        break;
    case 13:
        model_name = "cyan3";
        break;
    case 21:
        model_name = "magenta1";
        break;
    case 22:
        model_name = "magenta2";
        break;
    case 23:
        model_name = "magenta3";
        break;
    default:
        return;
    }

    gazebo::physics::ModelPtr model = world->ModelByName(model_name);
    if (model)
    {
        model->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
        model->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    }
}

void RobotGazeboSecond::ComputeBallPosition()
{
    ball_pose_on_real.x = ball_pose.y * 100;
    ball_pose_on_real.y = ball_pose.x * 100;
}

bool RobotGazeboSecond::CheckBallStatus(Vect3 robot_pose)
{
    float dist_to_ball = sqrt(pow(robot_pose.x - ball_pose_on_real.x, 2) + pow(robot_pose.y - ball_pose_on_real.y, 2));
    float robot_th = robot_pose.th + 90;

    while (robot_th > 180)
        robot_th -= 360;
    while (robot_th < -180)
        robot_th += 360;

    float theta_to_ball = atan2(ball_pose_on_real.y - robot_pose.y, ball_pose_on_real.x - robot_pose.x) * RAD2DEG;
    while (theta_to_ball > 180)
        theta_to_ball -= 360;
    while (theta_to_ball < -180)
        theta_to_ball += 360;

    float error_theta = theta_to_ball - robot_th;

    if (dist_to_ball < 45 && fabs(error_theta) < 10)
        return true;

    return false;
}

void RobotGazeboSecond::BallDribbling(Vect3 robot_pose, string robot_model_name)
{
    math::Pose3d pose;
    math::Quaterniond orientation;
    math::Vector3d position;
    pose = world->ModelByName(robot_model_name)->RelativePose();
    position = pose.Pos();
    orientation = pose.Rot();

    float target_x = 0.26 * cos(angles::from_degrees(robot_pose.th));
    float target_y = 0.26 * sin(angles::from_degrees(robot_pose.th));

    target_y = -target_y;

    math::Vector3d target_pos(target_x, target_y, 0.0);
    math::Pose3d target_pose(position + target_pos, orientation);
    ball_model->SetLinearVel(math::Vector3d::Zero);
    ball_model->SetRelativePose(target_pose);
}

void RobotGazeboSecond::Kick(uint8_t mode_, float force, Vect3 robot_pose)
{
    if (!ball_status)
        return;

    Vect3 ball_vel;
    float ball_velocity;

    float robot_th = robot_pose.th + 90;

    while (robot_th > 180)
        robot_th -= 360;
    while (robot_th < -180)
        robot_th += 360;

    float theta_to_ball = atan2(ball_pose_on_real.y - robot_pose.y, ball_pose_on_real.x - robot_pose.x) * RAD2DEG;
    while (theta_to_ball > 180)
        theta_to_ball -= 360;
    while (theta_to_ball < -180)
        theta_to_ball += 360;

    // Convert the force to velocity
    ball_velocity = force;

    ball_vel.x = ball_velocity * sin(angles::from_degrees(theta_to_ball));
    ball_vel.y = ball_velocity * cos(angles::from_degrees(theta_to_ball));

    math::Vector3d target_velocity;
    target_velocity.Set(ball_vel.x, ball_vel.y, 0.0);
    ball_model->SetLinearVel(target_velocity);
}