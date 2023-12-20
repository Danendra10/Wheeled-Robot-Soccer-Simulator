#include <simulator/robot_gazebo_new_control.hpp>

unsigned int FIELD_X = 1200;
unsigned int FIELD_Y = 800;

GZ_REGISTER_MODEL_PLUGIN(RobotGazeboNewControl)

RobotGazeboNewControl::RobotGazeboNewControl()
{
}

RobotGazeboNewControl::~RobotGazeboNewControl()
{
    update_connection.reset();
    ros_node->shutdown();
}

void RobotGazeboNewControl::Load(physics::ModelPtr _model, sdf::ElementPtr)
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
        ROS_FATAL("Model bola tidak terdeteksi pada robot %s", robot_model_name.c_str());
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

    // This event is broadcast every simulation iteration
    // Reference: https://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1event_1_1Events.html#a0c8c7e6a33ad46f8be45b6a33d279fdc
    update_connection = event::Events::ConnectWorldUpdateBegin(
        // std::bind is used to bind the function to the class
        // Reference: https://en.cppreference.com/w/cpp/utility/functional/bind
        std::bind(&RobotGazeboNewControl::OnUpdate, this));

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
     *  you are effectively saying that you want to replace the managed object with a new one. In this case, it replaces the
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

    /**
     * When std::bind(&RobotGazeboNewControl::QueueThread, this) is passed to the std::thread constructor,
     * it tells the new thread to execute the QueueThread member function on the current instance of the
     * RobotGazeboNewControl class. Essentially, it sets up a thread that will handle tasks defined within
     * the QueueThread function, which is often related to managing a queue of messages or events in the context of a ROS node.
     *
     * Reference to thread: https://en.cppreference.com/w/cpp/thread/thread
     */
    ros_queue_thread = std::thread(std::bind(&RobotGazeboNewControl::QueueThread, this));
}

// Main Loop
void RobotGazeboNewControl::OnUpdate()
{
    ROS_WARN("HELLO IM THE FKING PKG THAT U CREATED");
}

// Thread
void RobotGazeboNewControl::QueueThread()
{
    static const double timeout = 0.01;
    while (ros_node->ok())
    {
        ros_queue.callAvailable(ros::WallDuration(timeout));
    }
}
