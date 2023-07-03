#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <unistd.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>



struct Synchronizer
{
    std::string sender_name;
    void *listener_ptr;
    std::shared_ptr<vrpn_Tracker_Remote> vrpn_tracker;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr gazebo_client;
    std::shared_ptr<gazebo_msgs::srv::SetEntityState::Request> request;
};



class VRPNListener : public rclcpp::Node
{
private:
    // params
    int _port;
    std::string _server;
    std::string _frame_id;
    double _mainloop_frequency;
    double _refresh_trackers_frequency;
    double _tracker_mainloop_frequency;

    // vrpn entitis
    std::shared_ptr<vrpn_Connection> _vrpn_connection;

    // synchronizer
    std::unordered_map<std::string, std::shared_ptr<Synchronizer>> _synchronizers;

    // timers
    rclcpp::TimerBase::SharedPtr _mainloop_timer;
    rclcpp::TimerBase::SharedPtr _trackers_refresh_timer;

    static void VRPN_CALLBACK handleTracker(void* userData, const vrpn_TRACKERCB trackerData);
    void vrpnConnectionMainloop();
    void refresh_trackers();
    void mainloop();
    template <class T> 
    void loadParam(std::string param_name, T default_value, T& param);
    void loadParams();

public:
    VRPNListener(std::string name);
};