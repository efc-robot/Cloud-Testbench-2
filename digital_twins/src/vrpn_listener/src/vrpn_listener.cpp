#include "vrpn_listener/vrpn_listener.hpp"


VRPNListener::VRPNListener(std::string name) : Node(name)
{
    // load params
    loadParams();

    // init vrpn connection
    std::string host = _server + ":" + std::to_string(_port);
    _vrpn_connection = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host.c_str()));

    // create timers
    _mainloop_timer = this->create_wall_timer(std::chrono::seconds(1)/_mainloop_frequency, std::bind(&VRPNListener::mainloop, this));
    _trackers_refresh_timer = this->create_wall_timer(std::chrono::seconds(1)/_refresh_trackers_frequency, std::bind(&VRPNListener::refresh_trackers, this));

    RCLCPP_INFO(this->get_logger(), "VRPN listener initialized");
}

void VRPN_CALLBACK VRPNListener::handleTracker(void* userData, const vrpn_TRACKERCB trackerData)
{
    Synchronizer *synchronizer = static_cast<Synchronizer *>(userData);
    VRPNListener *listener = static_cast<VRPNListener *>(synchronizer->listener_ptr);

    RCLCPP_INFO(listener->get_logger(), "recv new tracker data from sender [%s]", synchronizer->sender_name.c_str());

    // set pos msg
    synchronizer->request->state.pose.position.x = trackerData.pos[0];
    synchronizer->request->state.pose.position.y = trackerData.pos[1];
    synchronizer->request->state.pose.position.z = trackerData.pos[2];
    synchronizer->request->state.pose.orientation.x = trackerData.quat[0];
    synchronizer->request->state.pose.orientation.y = trackerData.quat[1];
    synchronizer->request->state.pose.orientation.z = trackerData.quat[2];
    synchronizer->request->state.pose.orientation.w = trackerData.quat[3];

    // send request
    auto future = synchronizer->gazebo_client->async_send_request(synchronizer->request);
}

void VRPNListener::vrpnConnectionMainloop()
{
    _vrpn_connection->mainloop();

    if (!_vrpn_connection->doing_okay())
    {
        RCLCPP_WARN(this->get_logger(), "VRPN connection is not 'doing okay'");
    }
}

void VRPNListener::refresh_trackers()
{
    for (int i = 0; _vrpn_connection->sender_name(i) != NULL; i++) 
    {
        std::string sender_name = _vrpn_connection->sender_name(i);
        std::string &tracker_name = sender_name;
        std::string &entity_name = sender_name;
        std::string &synchronizer_name = sender_name;

        if (_synchronizers.count(sender_name) != 0)
        {
            continue;
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Found new sender: " << sender_name);

        // new synchronizer
        std::shared_ptr<Synchronizer> new_synchronizer = std::make_shared<Synchronizer>();
        new_synchronizer->sender_name = sender_name;
        new_synchronizer->listener_ptr = this;
        new_synchronizer->vrpn_tracker = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), _vrpn_connection.get());
        new_synchronizer->gazebo_client = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
        new_synchronizer->request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

        // init request
        new_synchronizer->request->state.name = entity_name;
        new_synchronizer->request->state.reference_frame = _frame_id;

        // register tracker change handler
        new_synchronizer->vrpn_tracker->register_change_handler(new_synchronizer.get(), &VRPNListener::handleTracker);

        // create new tracker mainloop timer
        this->create_wall_timer(
            std::chrono::seconds(1)/_tracker_mainloop_frequency, 
            std::bind(&vrpn_Tracker_Remote::mainloop, new_synchronizer->vrpn_tracker)
        );

        // register new synchronizer
        _synchronizers.insert(std::make_pair(synchronizer_name, new_synchronizer));

        RCLCPP_INFO_STREAM(this->get_logger(), "New synchronizer created: " << synchronizer_name);
    }
}

void VRPNListener::mainloop()
{
    vrpnConnectionMainloop();
}

template <class T> 
void VRPNListener::loadParam(std::string param_name, T default_value, T& param)
{
    // declare parameter
    this->declare_parameter(param_name, default_value);

    // load parameter
    bool success = this->get_parameter(param_name, param);

    if (!success)
    {
        RCLCPP_INFO(this->get_logger(), "load param [%s] failed", param_name.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "load param [%s] success", param_name.c_str());
    }
}

void VRPNListener::loadParams()
{
    loadParam(std::string("server"), std::string(""), _server);
    loadParam(std::string("port"), 3883, _port);
    loadParam(std::string("frame_id"), std::string("world"), _frame_id);
    loadParam(std::string("mainloop_frequency"), 100.0, _mainloop_frequency);
    loadParam(std::string("refresh_trackers_frequency"), 1.0, _refresh_trackers_frequency);
    loadParam(std::string("tracker_mainloop_frequency"), 100.0, _tracker_mainloop_frequency);
}


int main(int argc, char** argv)
{
    // init ROS2 node
    rclcpp::init(argc, argv);

    // create VRPNListener entity
    std::string name = "vrpn_listener";
    auto vrpn_listener = std::make_shared<VRPNListener>(name);

    // run ROS2 node
    rclcpp::spin(vrpn_listener);

    // shutdown ROS2 node
    rclcpp::shutdown();

    return 0;
}
