#include "simulator/robot_gazebo.hh"

// Define world SIZE : Regional 9x6 | Nasional 12x8
unsigned int X_FIELD = 1200;
unsigned int Y_FIELD = 800;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RobotGazebo)

RobotGazebo::RobotGazebo()
{
}

RobotGazebo::~RobotGazebo()
{
    update_connection.reset();
    ros_node->shutdown();
}

char data_bola;

void RobotGazebo::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    //   Store the pointer to the model
    robot_model = _model;
    if (!robot_model)
    {
        ROS_ERROR("Null pointer access: robot_model is null.");
        return;
    }
    world = _model->GetWorld();
    robot_model_name = _model->GetName();
    ball_model = world->ModelByName(MODEL_BALL);
    if (!ball_model)
    {
        ROS_FATAL("Model bola tidak terdeteksi pada robot %s", robot_model_name.c_str());
        return;
    }

    // Mendapatkan index robot diawal agar mempercepat iterasi dan pencarian
    for (int i = 0; i < world->ModelCount(); i++)
        if (world->ModelByIndex(i) == ball_model)
        {
            ball_index = i;
            break;
        }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RobotGazebo::OnUpdate, this));

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "robot_gazebo", ros::init_options::NoSigintHandler);
    }

    ros_node.reset(new ros::NodeHandle("robot_gazebo"));

    //---Publisher
    pub_handle_ball = ros_node->advertise<std_msgs::UInt8>("sim2pc/robot_handle_ball", 16);
    pub_odometry = ros_node->advertise<geometry_msgs::Pose2D>("simodom", 16);

    for (int i = 1; i <= 3; i++)
    {
        string odom_topic_cyan = "cyan" + to_string(i) + "/odom";
        string odom_topic_magenta = "magenta" + to_string(i) + "/odom";

        pub_cyan[i] = ros_node->advertise<geometry_msgs::Pose2D>(odom_topic_cyan, 16);
        pub_magenta[i] = ros_node->advertise<geometry_msgs::Pose2D>(odom_topic_magenta, 16);
    }
    pub_ball_pose_cyan = ros_node->advertise<geometry_msgs::Pose2D>("/cyan/ball/pose", 16);
    pub_ball_pose_magenta = ros_node->advertise<geometry_msgs::Pose2D>("/magenta/ball/pose", 16);

    //---Subscriber

    ros::SubscribeOptions so_gazebo_model =
        ros::SubscribeOptions::create<gazebo_msgs::ModelStates>("/gazebo/model_states",
                                                                16,
                                                                boost::bind(&RobotGazebo::callbackSubGazeboModel, this, _1),
                                                                ros::VoidPtr(), &ros_queue);
    sub_gazebo_model = ros_node->subscribe(so_gazebo_model);

    ros::SubscribeOptions so_lidar =
        ros::SubscribeOptions::create<sensor_msgs::LaserScan>("radial_scan",
                                                              16,
                                                              boost::bind(&RobotGazebo::callbackSubLidar, this, _1),
                                                              ros::VoidPtr(), &ros_queue);
    sub_lidar = ros_node->subscribe(so_lidar);

    for (int i = 1; i <= 3; i++)
    {
        string namatopic = "cyan" + to_string(i) + "/pc2sim/velocity";
        ros::SubscribeOptions so_velocity_cyan =
            ros::SubscribeOptions::create<iris_msgs::robot2sim>(namatopic,
                                                                16,
                                                                boost::bind(&RobotGazebo::callbackSubVelocity, this, _1),
                                                                ros::VoidPtr(), &ros_queue);

        sub_velocity_cyan[i] = ros_node->subscribe(so_velocity_cyan);

        string namatopicmagenta = "magenta" + to_string(i) + "/pc2sim/velocity";
        ros::SubscribeOptions so_velocity_magenta =
            ros::SubscribeOptions::create<iris_msgs::robot2sim>(namatopicmagenta,
                                                                16,
                                                                boost::bind(&RobotGazebo::callbackSubVelocity, this, _1),
                                                                ros::VoidPtr(), &ros_queue);

        sub_velocity_magenta[i] = ros_node->subscribe(so_velocity_magenta);
    }

    // sub_velocity = ros_node->subscribe("pc2sim/cyan1/velocity", 1000, RobotGazebo::callbackSubVelocity);

    ros::SubscribeOptions so_kicker =
        ros::SubscribeOptions::create<std_msgs::Int16MultiArray>("pc2sim/kicker",
                                                                 8,
                                                                 boost::bind(&RobotGazebo::callbackSubKicker, this, _1),
                                                                 ros::VoidPtr(), &ros_queue);
    sub_kicker = ros_node->subscribe(so_kicker);

    //---Timer
    timer_terima_pc = ros::Time::now().toSec();

    //---Get all the running topics
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    // Spin up the queue helper thread.
    ros_queue_thread = std::thread(std::bind(&RobotGazebo::queueThread, this));
}

void RobotGazebo::callbackSubGazeboModel(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    if (ball_index >= msg->pose.size())
    {
        ROS_ERROR("Ball index out of bounds in callbackSubGazeboModel.");
        return;
    }

    if (world->ModelCount())
    {
        // Posisi bola virtual
        geometry_msgs::Pose pose = msg->pose[ball_index];
        math::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
        bola_x_buffer = position.X();
        bola_y_buffer = position.Y();

        // Kecepatan bola virtual
        geometry_msgs::Twist twist = msg->twist[ball_index];
    }
}

void RobotGazebo::callbackSubLidar(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    for (int i = 0; i < 144; i++)
    {
        if (msg->ranges[i] == INFINITY)
            halangan[i] = 9999;
        else
            halangan[i] = msg->ranges[i] * 100 + 10;
        // ROS_INFO("%d %f", i*20, msg->ranges[i] * 100);
    }
}

void RobotGazebo::callbackSubVelocity(const iris_msgs::robot2sim::ConstPtr &msg)
{
    if (!msg)
    {
        ROS_ERROR("Received null pointer in callbackSubVelocity.");
        return;
    }
    if (msg->coderobot == 11)
    {
        ROS_WARN_ONCE("Connected to Workspace");
        // cout << "ROBOT CYAN 1" << endl;
        buffer_v_cyan[1][0] = msg->vx;
        buffer_v_cyan[1][1] = msg->vy;
        buffer_v_cyan[1][2] = msg->vth;
        // printf("%d || %d\n", msg->vx, msg->vy);

        timer_terima_cyan[1] = ros::Time::now().toSec();
        status_terima_cyan[1] = true;
    }
    else if (msg->coderobot == 12)
    {
        buffer_v_cyan[2][0] = msg->vx;
        buffer_v_cyan[2][1] = msg->vy;
        buffer_v_cyan[2][2] = msg->vth;

        timer_terima_cyan[2] = ros::Time::now().toSec();
        status_terima_cyan[2] = true;
    }

    else if (msg->coderobot == 13)
    {
        buffer_v_cyan[3][0] = msg->vx;
        buffer_v_cyan[3][1] = msg->vy;
        buffer_v_cyan[3][2] = msg->vth;

        timer_terima_cyan[3] = ros::Time::now().toSec();
        status_terima_cyan[3] = true;
    }

    else if (msg->coderobot == 14)
    {
        buffer_v_cyan[4][0] = msg->vx;
        buffer_v_cyan[4][1] = msg->vy;
        buffer_v_cyan[4][2] = msg->vth;

        timer_terima_cyan[4] = ros::Time::now().toSec();
        status_terima_cyan[4] = true;
    }

    else if (msg->coderobot == 15)
    {
        buffer_v_cyan[5][0] = msg->vx;
        buffer_v_cyan[5][1] = msg->vy;
        buffer_v_cyan[5][2] = msg->vth;

        timer_terima_cyan[5] = ros::Time::now().toSec();
        status_terima_cyan[5] = true;
    }

    else if (msg->coderobot == 21)
    {
        buffer_v_magenta[1][0] = msg->vx;
        buffer_v_magenta[1][1] = msg->vy;
        buffer_v_magenta[1][2] = msg->vth;

        timer_terima_magenta[1] = ros::Time::now().toSec();
        status_terima_magenta[1] = true;
    }
    else if (msg->coderobot == 22)
    {
        buffer_v_magenta[2][0] = msg->vx;
        buffer_v_magenta[2][1] = msg->vy;
        buffer_v_magenta[2][2] = msg->vth;

        timer_terima_magenta[2] = ros::Time::now().toSec();
        status_terima_magenta[2] = true;
    }
    else if (msg->coderobot == 23)
    {
        buffer_v_magenta[3][0] = msg->vx;
        buffer_v_magenta[3][1] = msg->vy;
        buffer_v_magenta[3][2] = msg->vth;

        timer_terima_magenta[3] = ros::Time::now().toSec();
        status_terima_magenta[3] = true;
    }
    else if (msg->coderobot == 24)
    {
        buffer_v_magenta[4][0] = msg->vx;
        buffer_v_magenta[4][1] = msg->vy;
        buffer_v_magenta[4][2] = msg->vth;

        timer_terima_magenta[4] = ros::Time::now().toSec();
        status_terima_magenta[4] = true;
    }
    else if (msg->coderobot == 25)
    {
        buffer_v_magenta[5][0] = msg->vx;
        buffer_v_magenta[5][1] = msg->vy;
        buffer_v_magenta[5][2] = msg->vth;

        timer_terima_magenta[5] = ros::Time::now().toSec();
        status_terima_magenta[5] = true;
    }

    // printf("testststtsts\n");
    // timer_terima_pc = ros::Time::now().toSec();
    // status_terima_pc = true;
    // vx_buffer = msg->linear.x;
    // vy_buffer = msg->linear.y;
    // vsudut_buffer = msg->angular.z;
}

void RobotGazebo::callbackSubKicker(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
    mode_tendang = msg->data[0];
    kecepatan_tendang = msg->data[1];
    status_tendang = true;
}

// Mirip seperti ros::spin(), tetapi versi Gazebo + ROS
// (mengikuti tutorial / contoh)
void RobotGazebo::queueThread()
{
    static const double timeout = 0.01;
    while (ros_node->ok())
    {
        ros_queue.callAvailable(ros::WallDuration(timeout));
    }
}

// Timer 50Hz
// Timer diatur dikonfigurasi .world
void RobotGazebo::OnUpdate()
{
    // Jika model sudah di-inisialisasi semua,
    // Baru mulai eksekusi
    // sprintf(robot_model_name);
    // cout << robot_model_name << endl;
    if (kbhit())
    {
        char c = std::cin.get();
        changeRobot(c);
        // sisi_letak_bola = !sisi_letak_bola;
    }
    // cout<<robot_selected<<endl;
    // printf("ROBOT NAME%s  \n",robot_model_name);
    if (world->ModelCount())
    {
        // status_terima_cyan[1] = true;
        // status_terima_cyan[2] = true;
        // status_terima_cyan[3] = true;
        // if (world->ModelByName("cyan2") == 0)
        // {
        //     cout<<"MODEL 2 UNDETECTED"<<endl;
        // }

        // cout<<world->ModelByName("cyan2")<<endl;;

        // for (int i = 1; i <= 3; i++)
        // {
        //     cout<<"ROBOT "<<"CYAN" <<i<<" " << status_terima_cyan[i] <<endl;
        //     cout<<"ROBOT "<<"MAGENTA" <<i<<" " << status_terima_magenta[i] <<endl;
        // }

        //     // Jika simulator menerima pesan
        // if(status_terima_cyan[1])
        // {
        //     publishData();
        //     // Mengolah data agar data yang dikirim
        //     // Sesuai dengan standarisasi pengiriman robot
        //     olahDataPosisiRobot();
        //     olahDataKecepatanRobot();
        //     olahDataPosisiBola();

        //     jalanManual(11,this->v_cyan[1][0], this->v_cyan[1][1], this->v_cyan[1][2]);
        if (cekStatusBola() && !status_tendang)
        {
            status_bola_dribble = 1;
            giringBola();
        }
        else if (status_tendang)
        {
            tendang(10, 8);
        }
        else
        {
            status_bola_dribble = 0;
            // Tendang mode.10 = lepasPakaiDrible
            tendang(10, 500);
        }

        //     // Simulator mengirim data ke PC

        // }
        // // Jika simulator tidak menerima pesan
        // // maka robot di-set diam, mengatasi stuck velocity
        // else
        // {
        //     jalanManual(11,0, 0, 0);
        //     status_bola_dribble = 0;

        // }

        // // Memanipulasi efek tabrakan
        // // collisionHandling();

        controlRobot();

        for (int i = 1; i <= 3; i++)
        {
            // CYAN
            if (ros::Time::now().toSec() - timer_terima_cyan[i] > 2)
                status_terima_cyan[i] = false;

            // MAGENTA
            if (ros::Time::now().toSec() - timer_terima_magenta[i] > 2)
                status_terima_magenta[i] = false;
        }
    }
}
void RobotGazebo::changeRobot(char _command)
{
    switch (_command)
    {
    case 't':
        robot_selected = 1;
        break;
    case 'y':
        robot_selected = 2;
        break;

    default:
        break;
    }

    printf("ROBOT SELECTED %d \n", robot_selected);
}

void RobotGazebo::jalanSemua(int _coderobot)
{
    if (_coderobot == 11 && world->ModelByName("cyan1"))
    {
        world->ModelByName("cyan1")->SetLinearVel(math::Vector3d(this->v_cyan[1][0], this->v_cyan[1][1], 0.0));
        world->ModelByName("cyan1")->SetAngularVel(math::Vector3d(0.0, 0.0, this->v_cyan[1][2]));
    }

    if (_coderobot == 12 && world->ModelByName("cyan2"))
    {
        world->ModelByName("cyan2")->SetLinearVel(math::Vector3d(this->v_cyan[2][0], this->v_cyan[2][1], 0.0));
        world->ModelByName("cyan2")->SetAngularVel(math::Vector3d(0.0, 0.0, this->v_cyan[2][2]));
    }

    // if (_coderobot == 13 && world->ModelByName("cyan3"))
    // {
    //     world->ModelByName("cyan3")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
    //     world->ModelByName("cyan3")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    // }

    // if (_coderobot == 21 && world->ModelByName("magenta1"))
    // {
    //     world->ModelByName("magenta1")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
    //     world->ModelByName("magenta1")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    // }

    // if (_coderobot == 22 && world->ModelByName("magenta2"))
    // {
    //     world->ModelByName("magenta2")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
    //     world->ModelByName("magenta2")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    // }

    // if (_coderobot == 23 && world->ModelByName("magenta3"))
    // {
    //     world->ModelByName("magenta3")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
    //     world->ModelByName("magenta3")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    // }
}

void RobotGazebo::controlRobot()
{

    for (int i = 1; i <= 3; i++)
    {
        /// FOR CYAN CONTROL
        int coderobotcyan = 10 + i;
        if (status_terima_cyan[i])
        {
            // Mengolah data agar data yang dikirim
            // Sesuai dengan standarisasi pengiriman robot
            try
            {
                olahDataPosisiRobot();
                olahDataKecepatanRobot();
                olahDataPosisiBola();

                jalanManual(coderobotcyan, this->v_cyan[i][0], this->v_cyan[i][1], this->v_cyan[i][2]);
                if (cekStatusBola() && !status_tendang)
                {
                    status_bola_dribble = 1;
                    giringBola();
                }
                else if (status_tendang)
                {
                    tendang(mode_tendang, kecepatan_tendang);
                }
                else
                {
                    status_bola_dribble = 0;
                    // Tendang mode.10 = lepasPakaiDrible
                    tendang(10, 500);
                }
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("Caught exception: %s", e.what());
            }
            publishData();
        }
        // Jika simulator tidak menerima pesan
        // maka robot di-set diam, mengatasi stuck velocity
        else
        {
            status_bola_dribble = 0;
            jalanManual(coderobotcyan, 0, 0, 0);
        }

        // FOR MAGENTA CONTROL
        int coderobotmagenta = 20 + i;
        if (status_terima_magenta[i])
        {
            publishData();
            // Mengolah data agar data yang dikirim
            // Sesuai dengan standarisasi pengiriman robot
            olahDataPosisiRobot();
            olahDataKecepatanRobot();
            olahDataPosisiBola();

            // jalanSemua(coderobotmagenta);

            jalanManual(coderobotmagenta, this->v_magenta[i][0], this->v_magenta[i][1], this->v_magenta[i][2]);
            if (cekStatusBola() && !status_tendang)
            {
                status_bola_dribble = 1;
                giringBola();
            }
            else if (status_tendang)
            {
                tendang(mode_tendang, kecepatan_tendang);
            }
            else
            {
                status_bola_dribble = 0;
                // Tendang mode.10 = lepasPakaiDrible
                tendang(10, 500);
            }

            // Simulator mengirim data ke PC
        }
        // Jika simulator tidak menerima pesan
        // maka robot di-set diam, mengatasi stuck velocity
        else
        {
            status_bola_dribble = 0;
            jalanManual(coderobotmagenta, 0, 0, 0);
        }
    }
}

int RobotGazebo::kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

void RobotGazebo::jalanManual(int _coderobot, float _vx, float _vy, float _vsudut)
{
    // cout << "X " << this->v_cyan[1][0] << "Y" << this->v_cyan[1][1] << "TH" << this->v_cyan[1][2] << endl;

    if (_coderobot == 11 && world->ModelByName("cyan1"))
    {
        world->ModelByName("cyan1")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
        world->ModelByName("cyan1")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    }

    if (_coderobot == 12 && world->ModelByName("cyan2"))
    {
        world->ModelByName("cyan2")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
        world->ModelByName("cyan2")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    }

    if (_coderobot == 13 && world->ModelByName("cyan3"))
    {
        world->ModelByName("cyan3")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
        world->ModelByName("cyan3")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    }

    if (_coderobot == 21 && world->ModelByName("magenta1"))
    {
        world->ModelByName("magenta1")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
        world->ModelByName("magenta1")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    }

    if (_coderobot == 22 && world->ModelByName("magenta2"))
    {
        world->ModelByName("magenta2")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
        world->ModelByName("magenta2")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    }

    if (_coderobot == 23 && world->ModelByName("magenta3"))
    {
        world->ModelByName("magenta3")->SetLinearVel(math::Vector3d(_vx, _vy, 0.0));
        world->ModelByName("magenta3")->SetAngularVel(math::Vector3d(0.0, 0.0, _vsudut));
    }
}

bool RobotGazebo::cekStatusBola()
{
    float jarak_robot_ke_bola = sqrt(pow(bola_x_pada_frame, 2) + pow(bola_y_pada_frame, 2));

    // Jika jarak robot memenuhi kondisi threshold dan bukan kiper
    // ROS_INFO("%f | %f", jarak_robot_ke_bola, bola_theta_pada_frame);

    if (jarak_robot_ke_bola <= JARAK_BOLA)
    {
        // Mengecek apakah bola berada tepat didepan robot

        if ((bola_theta_pada_frame < (90 + 5)) && (bola_theta_pada_frame > (90 - 5)))
            return true;
    }

    return false;
}

void RobotGazebo::giringBola()
{
    // Mendapatkan data robot (x, y, dan sudut) dari dunia simulator
    math::Pose3d pose;
    math::Quaterniond orientation;
    math::Vector3d position;
    pose = robot_model->RelativePose();
    position = pose.Pos();
    orientation = pose.Rot();

    // Membuat dribble virtual dengan cara
    // bola mengikuti pergerakan robot bagian depan
    float target_x = 0.26 * cos(angles::from_degrees(theta));
    float target_y = 0.26 * sin(angles::from_degrees(theta));
    if (robot_model_name[0] == TEAM_CYAN)
    {
        target_x = target_x;
        target_y = -target_y;
    }
    else if (robot_model_name[0] == TEAM_MAGENTA)
    {
        target_x = -target_x;
        target_y = target_y;
    }
    math::Vector3d target_pos(target_x, target_y, 0.0);
    math::Pose3d target_pose(position + target_pos, orientation);
    ball_model->SetLinearVel(math::Vector3d::Zero);
    ball_model->SetRelativePose(target_pose);
}

void RobotGazebo::tendang(unsigned char _mode, unsigned int _kecepatan)
{
    float kecepatan_bola, kecepatan_x_bola, kecepatan_y_bola;

    // Melakukan tendang jika sedang dalam keadaan menggiring bola
    if (status_bola_dribble == 1)
    {
        // Berhenti dulu
        jalanManual(1, 0, 0, 0);

        // Konversi kecepatan dari robot ke simulator
        if (_mode == 10)
            kecepatan_bola = 0.6;
        else
            kecepatan_bola = (_kecepatan * 100 * 1.0 / 1500.0) * 3.65; // Kali 100 only for robot segitiga

        if (robot_model_name[0] == TEAM_CYAN)
        {
            kecepatan_x_bola = kecepatan_bola * sin(angles::from_degrees(bola_theta_pada_lapangan));
            kecepatan_y_bola = -kecepatan_bola * cos(angles::from_degrees(bola_theta_pada_lapangan));
        }
        else if (robot_model_name[0] == TEAM_MAGENTA)
        {
            kecepatan_x_bola = -kecepatan_bola * sin(angles::from_degrees(bola_theta_pada_lapangan));
            kecepatan_y_bola = kecepatan_bola * cos(angles::from_degrees(bola_theta_pada_lapangan));
        }

        math::Vector3d target_velocity;
        switch (_mode)
        {
        case 10:
            target_velocity.Set(kecepatan_x_bola, kecepatan_y_bola, 0.0);
            ball_model->SetLinearVel(target_velocity);
            break;
        case 70:
            target_velocity.Set(kecepatan_x_bola, kecepatan_y_bola, 5.15);
            ball_model->SetLinearVel(target_velocity);
            break;
        case 80:
            target_velocity.Set(kecepatan_x_bola, kecepatan_y_bola, 4.15);
            ball_model->SetLinearVel(target_velocity);
            break;
        case 90:
            target_velocity.Set(kecepatan_x_bola, kecepatan_y_bola, 0.0);
            ball_model->SetLinearVel(target_velocity);
            break;
        default:
            break;
        }
    }
    if (!cekStatusBola())
        status_tendang = false;
}

void RobotGazebo::collisionHandling()
{
    math::Quaterniond target_rot = robot_model->RelativePose().Rot();
    math::Vector3d euler = target_rot.Euler();
    math::Pose3d current_pose;
    current_pose.Set(robot_model->RelativePose().Pos(), robot_model->RelativePose().Rot());

    if (fabs(current_pose.Pos().Z()) > 0.05 || fabs(euler.X()) > 0.05 || fabs(euler.Y()) > 0.05)
    {
        math::Vector3d target_position(current_pose.Pos().X(), current_pose.Pos().Y(), 0.01);
        math::Vector3d target_rotation(0, 0, euler.Z());
        math::Pose3d target_pose;
        target_pose.Set(target_position, target_rotation);
        robot_model->SetRelativePose(target_pose);
    }
}

void RobotGazebo::publishData()
{
    // Data virtual odometry
    geometry_msgs::Pose2D msg_odometry;
    msg_odometry.x = pos_x;
    msg_odometry.y = pos_y;
    msg_odometry.theta = theta;
    pub_odometry.publish(msg_odometry);

    std_msgs::UInt8 msg_handle_ball;
    msg_handle_ball.data = status_bola_dribble;
    pub_handle_ball.publish(msg_handle_ball);

    // // position = pose.Pos();
    // // orientation = pose.Rot();
    geometry_msgs::Pose2D data_robot;
    for (int i = 1; i <= 3; i++)
    {
        data_robot.x = cyan[i][0];
        data_robot.y = cyan[i][1];
        data_robot.theta = cyan[i][2];
        pub_cyan[i].publish(data_robot);

        data_robot.x = magenta[i][0];
        data_robot.y = magenta[i][1];
        data_robot.theta = magenta[i][2];

        pub_magenta[i].publish(data_robot);
    }
}

void RobotGazebo::olahDataKecepatanRobot()
{
    float kecepatan_cyan[6][3];
    float kecepatan_magenta[6][3];

    for (int i = 1; i <= 3; i++)
    {
        kecepatan_cyan[i][0] = buffer_v_cyan[i][1] * 0.015;
        kecepatan_cyan[i][1] = -(buffer_v_cyan[i][0]) * 0.015;
        // kecepatan_cyan[i][2] = buffer_v_cyan[i][2] * 0.2;
        // if (i == 1)
        //     printf("KECEPATAN CYAN %d : %.2f %.2f %.2f \n", i, kecepatan_cyan[i][0], kecepatan_cyan[i][1], kecepatan_cyan[i][2]);

        kecepatan_magenta[i][0] = -1 * (buffer_v_magenta[i][1]) * 0.075;
        kecepatan_magenta[i][1] = (buffer_v_magenta[i][0]) * 0.075;
        kecepatan_magenta[i][2] = buffer_v_magenta[i][2] * 0.2;

        // this->v_cyan[i][0] = (kecepatan_cyan[i][0] * sin(angles::from_degrees(cyan[i][2])) - kecepatan_cyan[i][1] * cos(angles::from_degrees(cyan[i][2])));
        // // this->v_cyan[i][0] = kecepatan_cyan[i][0] * sin(angles::from_degrees(cyan[i][2])) - kecepatan_cyan[i][1] * cos(angles::from_degrees(cyan[i][2]));
        // // this->v_cyan[i][1] = kecepatan_cyan[i][0] * cos(angles::from_degrees(cyan[i][2])) + kecepatan_cyan[i][1] * sin(angles::from_degrees(cyan[i][2]));
        // // this->v_cyan[i][0] = kecepatan_cyan[i][0];
        // this->v_cyan[i][1] = (kecepatan_cyan[i][0] * cos(angles::from_degrees(cyan[i][2])) + kecepatan_cyan[i][1] * sin(angles::from_degrees(cyan[i][2])));
        // // this->v_cyan[i][1] = kecepatan_cyan[i][1];
        // this->v_cyan[i][2] = -kecepatan_cyan[i][2];
        // this->v_cyan[i][0] = (kecepatan_cyan[i][0] * cos(angles::from_degrees(cyan[i][2])) + kecepatan_cyan[i][1] * sin(angles::from_degrees(cyan[i][2])));
        // this->v_cyan[i][1] = (-kecepatan_cyan[i][0] * sin(angles::from_degrees(cyan[i][2])) + kecepatan_cyan[i][1] * cos(angles::from_degrees(cyan[i][2])));
        this->v_cyan[i][0] = (kecepatan_cyan[i][0]);
        this->v_cyan[i][1] = (kecepatan_cyan[i][1]);
        this->v_cyan[i][2] = (kecepatan_cyan[i][2]);
        if (i == 1)
        {
            // printf("CYAN TH %f %f\n\n", cyan[i][2], angles::from_degrees(cyan[i][2]));
            printf("V CYAN %d : %.2f %.2f %.2f \n", i, this->v_cyan[i][0], this->v_cyan[i][1], this->v_cyan[i][2]);
        }

        this->v_magenta[i][0] = (int)(kecepatan_magenta[i][0] * cos(angles::from_degrees(magenta[i][2])) + kecepatan_magenta[i][1] * sin(angles::from_degrees(magenta[i][2])));
        this->v_magenta[i][1] = (int)(-kecepatan_magenta[i][0] * sin(angles::from_degrees(magenta[i][2])) + kecepatan_magenta[i][1] * cos(angles::from_degrees(magenta[i][2])));
        this->v_magenta[i][2] = (int)(kecepatan_magenta[i][2]);
    }
}

void RobotGazebo::olahDataPosisiRobot()
{
    math::Pose3d pose;
    math::Quaterniond orientation;
    math::Vector3d position;

    for (int i = 1; i <= 3; i++)
    {
        string what_robot = "cyan" + to_string(i);
        auto model = world->ModelByName(what_robot);
        if (!model)
        {
            // ROS_ERROR_ONCE("Model named %s does not exist.", what_robot.c_str());
            continue; // Skip this iteration
        }
        pose_cyan[i] = model->RelativePose();
        cyan[i][0] = -1 * (pose_cyan[i].Pos().Y() * 100) + Y_FIELD / 2;
        cyan[i][1] = pose_cyan[i].Pos().X() * 100 + X_FIELD / 2;
        cyan[i][2] = -angles::to_degrees(pose_cyan[i].Rot().Yaw()) + 90;

        while (cyan[i][2] > 180)
            cyan[i][2] -= 360;
        while (cyan[i][2] < -180)
            cyan[i][2] += 360;

        // printf("ROBOT %d : X :%.2f  || Y : %.2f  || THETA : %.2f %f", i, cyan[i][0], cyan[i][1], cyan[i][2], pose_cyan[i].Rot().Yaw());
    }

    for (int i = 1; i <= 3; i++)
    {
        string what_robot = "magenta" + to_string(i);
        auto model = world->ModelByName(what_robot);
        if (!model)
        {
            // ROS_ERROR_ONCE("Model named %s does not exist.", what_robot.c_str());
            continue; // Skip this iteration
        }
        pose_magenta[i] = model->RelativePose();
        magenta[i][0] = pose_magenta[i].Pos().Y() * 100 + Y_FIELD / 2;
        magenta[i][1] = -1 * (pose_magenta[i].Pos().X() * 100) + X_FIELD / 2;
        magenta[i][2] = -angles::to_degrees(pose_magenta[i].Rot().Yaw());
    }

    // LIST POSE BERBAGAI MODEL

    position = pose.Pos();
    orientation = pose.Rot();

    pos_x = position.Y() * 100;
    pos_y = position.X() * 100;

    if (robot_model_name[0] == TEAM_CYAN)
    {
        pos_x = -pos_x + Y_FIELD / 2;
        pos_y = pos_y + X_FIELD / 2;
        theta = -angles::to_degrees(orientation.Yaw());
    }
    else if (robot_model_name[0] == TEAM_MAGENTA)
    {
        pos_x = pos_x + Y_FIELD / 2;
        pos_y = -pos_y + X_FIELD / 2;
        theta = -angles::to_degrees(orientation.Yaw());
    }

    // printf("BEFORE: %f %f %f || %f %f\n", pos_x, pos_y, orientation.Yaw(), orientation.Roll(), orientation.Pitch());

    while (theta < -180)
        theta += 360;
    while (theta > 180)
        theta -= 360;

    // printf("AFTER: %f %f %f\n", pos_x, pos_y, theta);
}

void RobotGazebo::olahDataKecepatanBola()
{
}

void RobotGazebo::olahDataPosisiBola()
{
    // Calculate base positions
    bola_x_pada_lapangan = bola_y_buffer * 100;
    bola_y_pada_lapangan = bola_x_buffer * 100;

    // Adjust based on team
    if (robot_model_name[0] == TEAM_CYAN)
    {
        bola_x_pada_lapangan = -bola_x_pada_lapangan + Y_FIELD / 2;
        bola_y_pada_lapangan = bola_y_pada_lapangan + X_FIELD / 2;
    }
    else if (robot_model_name[0] == TEAM_MAGENTA)
    {
        bola_x_pada_lapangan = bola_x_pada_lapangan + Y_FIELD / 2;
        bola_y_pada_lapangan = -bola_y_pada_lapangan + X_FIELD / 2;
    }

    // Common calculations
    bola_theta_pada_lapangan = angles::to_degrees(atan2(bola_y_pada_lapangan - pos_y, bola_x_pada_lapangan - pos_x));
    bola_theta_pada_frame = bola_theta_pada_lapangan + theta;
    while (bola_theta_pada_frame < -180)
        bola_theta_pada_frame += 360;
    while (bola_theta_pada_frame > 180)
        bola_theta_pada_frame -= 360;

    // Calculate distance and update positions
    float jarak = sqrt(pow(bola_x_pada_lapangan - pos_x, 2) + pow(bola_y_pada_lapangan - pos_y, 2));
    bola_x_pada_frame = jarak * cos(angles::from_degrees(bola_theta_pada_frame));
    bola_y_pada_frame = jarak * sin(angles::from_degrees(bola_theta_pada_frame));

    // Determine ball status
    if (bola_x_pada_lapangan > Y_FIELD || bola_x_pada_lapangan < 0 || bola_y_pada_lapangan > X_FIELD || bola_y_pada_lapangan < 0)
    {
        status_bola_vision = 0;
    }
    else
    {
        status_bola_vision = (jarak < 100) ? 1 : data_bola;
    }

    // Publish ball pose
    geometry_msgs::Pose2D ball_pose;
    ball_pose.x = bola_x_pada_lapangan;
    ball_pose.y = bola_y_pada_lapangan;
    ball_pose.theta = bola_theta_pada_frame;

    if (robot_model_name[0] == TEAM_CYAN)
    {
        pub_ball_pose_cyan.publish(ball_pose);
    }
    else if (robot_model_name[0] == TEAM_MAGENTA)
    {
        pub_ball_pose_magenta.publish(ball_pose);
    }

    // ROS_INFO("vision bola simulator %d", status_bola_vision);
}