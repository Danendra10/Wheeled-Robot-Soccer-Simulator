#include <simulator/referee.hpp>

GZ_REGISTER_WORLD_PLUGIN(RefereeBox)

RefereeBox::RefereeBox()
{
}

RefereeBox::~RefereeBox()
{
    update_connection.reset();
    ros_node->shutdown();
}

void RefereeBox::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    command_referee = 'S';

    update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RefereeBox::OnUpdate, this));

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        // Ros Init Option NoSigintHandler means that the program will not terminate when ctrl + c is pressed
        // Reference: https://docs.ros.org/en/api/roscpp/html/namespaceros_1_1init__options.html
        ros::init(argc, argv, "referee_box", ros::init_options::NoSigintHandler);
    }

    ros_node.reset(new ros::NodeHandle("referee_box"));

    pub_command_referee = ros_node->advertise<std_msgs::UInt8>("/referee/sim/command", 16);

    ros_queue_thread = std::thread(std::bind(&RefereeBox::QueueThread, this));
}

void RefereeBox::OnUpdate()
{
    KeyboardHandler();
    PublishData();
}

void RefereeBox::QueueThread()
{
    static const double timeout = 0.01;
    while (ros_node->ok())
    {
        ros_queue.callAvailable(ros::WallDuration(timeout));
    }
}

int RefereeBox::Kbhit()
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

    if (ioctl(STDIN, FIONREAD, &bytesWaiting) == -1)
        return 0;

    return bytesWaiting;
}

void RefereeBox::KeyboardHandler()
{
    if (Kbhit() > 0)
    {
        char key = std::cin.get();

        command_referee = key;
    }
}

void RefereeBox::PublishData()
{
    std_msgs::UInt8 msg;
    msg.data = command_referee;
    pub_command_referee.publish(msg);
}