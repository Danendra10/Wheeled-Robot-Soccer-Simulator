#include "simulator/ball_gazebo.hh"

using namespace gazebo;
using namespace ignition;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BallGazebo)

BallGazebo::BallGazebo()
{
    command = 'S';
}

BallGazebo::~BallGazebo()
{
    update_connection.reset();
    ros_node->shutdown();
}

void BallGazebo::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    ball_model = _model;
    world = _model->GetWorld();
    ball_link = _model->GetLink("link");
    if (!ball_link)
        ROS_FATAL_ONCE("Ball Link tidak terdeteksi");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BallGazebo::OnUpdate, this));

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "ball_gazebo", ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    ros_node.reset(new ros::NodeHandle());

    //---Subscriber
    ros::SubscribeOptions so_command_basestation =
        ros::SubscribeOptions::create<std_msgs::Char>("referee2sim_command",
                                                      16,
                                                      boost::bind(&BallGazebo::callbackSubCommandBasestation, this, _1),
                                                      ros::VoidPtr(), &ros_queue);
    sub_command_basestation = ros_node->subscribe(so_command_basestation);

    // Spin up the queue helper thread.
    ros_queue_thread = std::thread(std::bind(&BallGazebo::queueThread, this));
}

void BallGazebo::callbackSubCommandBasestation(const std_msgs::CharConstPtr &msg)
{
    command = msg->data;
}

void BallGazebo::queueThread()
{
    static const double timeout = 0.01;
    while (ros_node->ok())
    {
        ros_queue.callAvailable(ros::WallDuration(timeout));
    }
}

// Update every 50 Hz
void BallGazebo::OnUpdate()
{
    static short int previous_bola_x, previous_bola_y;
    static char previous_command;
    static bool sisi_letak_bola;

    if (kbhit())
    {
        char c = std::cin.get();
        setPosisiBola(c, sisi_letak_bola);
        sisi_letak_bola = !sisi_letak_bola;
    }

    math::Pose3d ball_pose = ball_model->RelativePose();
    bola_x = ball_pose.Pos().X() * 100;
    bola_y = ball_pose.Pos().Y() * 100;

    if (previous_command != command)
    {
        sisi_letak_bola = !sisi_letak_bola;
        setPosisiBola(command, sisi_letak_bola);
        previous_bola_x = bola_x;
        previous_bola_y = bola_y;
    }
    else if (previous_command == command &&
             (previous_bola_x != bola_x && previous_bola_y != bola_y))
        setPosisiBola(command, sisi_letak_bola);

    if (command != COMMAND_START && command != COMMAND_STOP)
        previous_command = command;

    deselerasiBola();
}

void BallGazebo::deselerasiBola()
{
    math::Vector3d vektor_kecepatan_bola = ball_model->WorldLinearVel();
    math::Pose3d posisi_bola = ball_model->RelativePose();
    static double previous_kecepatan_bola = vektor_kecepatan_bola.Length();
    double kecepatan_bola = vektor_kecepatan_bola.Length();

    if (kecepatan_bola > 0.03)
    {
        if (!(previous_kecepatan_bola - kecepatan_bola) > 0 &&
            posisi_bola.Pos().Z() <= 0.121)
        {
            double force = 0.25 * -9.8 * 0.41;
            ball_link->AddForce(vektor_kecepatan_bola.Normalize() * force);
        }
    }
    else if (kecepatan_bola <= 0.03)
    {
        kecepatan_bola = 0.0;
        ball_model->SetLinearVel(math::Vector3d::Zero);
    }

    previous_kecepatan_bola = kecepatan_bola;
}

void BallGazebo::setPosisiBola(char _command, bool _side)
{
    math::Vector3d target_posisi;
    math::Vector3d translasi_posisi(-600, -400, 0);

    bool status_command = true;

    /*
     * @param:_side true: berarti bola diletakkan di sisi kanan lapangan
     * @param:_side false: berarti bola diletakkan di sisi kiri lapangan
     */
    switch (_command)
    {
    case COMMAND_KICKOFF_HOME:
        target_posisi.Set(600, 400, 0);
        break;
    case COMMAND_KICKOFF_AWAY:
        target_posisi.Set(600, 400, 0);
        break;
    case COMMAND_GOALKICK_HOME:
        if (_side)
            target_posisi.Set(200, 600, 0);
        else
            target_posisi.Set(200, 200, 0);
        break;
    case COMMAND_GOALKICK_AWAY:
        if (_side)
            target_posisi.Set(1000, 600, 0);
        else
            target_posisi.Set(1000, 200, 0);
        break;
    case COMMAND_FREEKICK_HOME:
        if (_side)
            target_posisi.Set(600, 600, 0);
        else
            target_posisi.Set(600, 200, 0);
        break;
    case COMMAND_FREEKICK_AWAY:
        if (_side)
            target_posisi.Set(600, 600, 0);
        else
            target_posisi.Set(600, 200, 0);
        break;
    case COMMAND_DROPBALL:
        if (_side)
            target_posisi.Set(600, 600, 0);
        else
            target_posisi.Set(600, 200, 0);
        break;
    case COMMAND_CORNER_HOME:
        if (_side)
            target_posisi.Set(25, 775);
        else
            target_posisi.Set(25, 25, 0);
        break;
    case COMMAND_CORNER_AWAY:
        if (_side)
            target_posisi.Set(1175, 775);
        else
            target_posisi.Set(1175, 25);
        break;
    case '1':
        target_posisi.Set(this->bola_x, this->bola_y, 0);
        robot_selected = 1;
        break;
    case '2':
        robot_selected = 2;
        break;
    case '3':
        robot_selected = 3;
        break;
    default:
        status_command = false;
        break;
    }

    if (status_command)
    {
        target_posisi = target_posisi + translasi_posisi;

        // Konversi ke skala dalam simulator
        target_posisi /= 100.0;

        math::Pose3d ball_pose = ball_model->RelativePose();
        math::Quaterniond ball_orientation = ball_pose.Rot();
        math::Pose3d target_pose(target_posisi, ball_orientation);

        ball_model->SetRelativePose(target_pose);
        ball_model->SetLinearVel(math::Vector3d::Zero);
        ball_model->SetAngularVel(math::Vector3d::Zero);
    }
}

int BallGazebo::kbhit()
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