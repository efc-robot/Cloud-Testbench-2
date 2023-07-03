#include <map>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include "entity_manager_interfaces/srv/spawn_robot.hpp"
#include "entity_manager_interfaces/srv/delete_robot.hpp"


class EntityManager : public rclcpp::Node
{
private:
    std::string _models_folder;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr _spawn_entity_client;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr _delete_entity_client;
    rclcpp::Service<entity_manager_interfaces::srv::SpawnRobot>::SharedPtr _spawn_robot_server;
    rclcpp::Service<entity_manager_interfaces::srv::DeleteRobot>::SharedPtr _delete_robot_server;
    
    template <class T> 
    void loadParam(std::string param_name, T default_value, T& param);
    void loadParams();
    void spawnEntity(std::string entity_name, std::string model_type, std::string robot_namespace, std::string reference_frame);
    void deleteEntity(std::string entity_name);
    void handleSpawnRobot(
        const std::shared_ptr<entity_manager_interfaces::srv::SpawnRobot::Request> request,
        std::shared_ptr<entity_manager_interfaces::srv::SpawnRobot::Response> response
    );
    void handleDeleteRobot(
        const std::shared_ptr<entity_manager_interfaces::srv::DeleteRobot::Request> request,
        std::shared_ptr<entity_manager_interfaces::srv::DeleteRobot::Response> response
    );

public:
    EntityManager(std::string name);
};

EntityManager::EntityManager(std::string name) : Node(name)
{
    // create ros client to gazebo_ros
    _spawn_entity_client = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    _delete_entity_client = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");

    // create ros server for receive command from remote bot system backend
    _spawn_robot_server = this->create_service<entity_manager_interfaces::srv::SpawnRobot>(
        "spawn_robot", std::bind(&EntityManager::handleSpawnRobot, 
        this, std::placeholders::_1, std::placeholders::_2)
    );
    _delete_robot_server = this->create_service<entity_manager_interfaces::srv::DeleteRobot>(
        "delete_robot", std::bind(&EntityManager::handleDeleteRobot, 
        this, std::placeholders::_1, std::placeholders::_2)
    );

    loadParams();
    
    RCLCPP_INFO(this->get_logger(), "Entity manager initialized");

    // spawnEntity("Rigid0", "xtark_r20_mec", "", "world");
    // deleteEntity("Rigid0");
}

template <class T> 
void EntityManager::loadParam(std::string param_name, T default_value, T& param)
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

void EntityManager::loadParams()
{
    loadParam(std::string("models_folder"), std::string(""), _models_folder);
}

void EntityManager::spawnEntity(
    std::string entity_name, std::string model_type, 
    std::string robot_namespace, std::string reference_frame
)
{
    // load model xml
    std::string model_path = _models_folder + "/" + model_type + ".urdf";
    RCLCPP_INFO(this->get_logger(), model_path.c_str());
    std::ifstream file(model_path);
    std::string model_content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // init request
    std::shared_ptr<gazebo_msgs::srv::SpawnEntity::Request> request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    
    // set entity info
    request->name = entity_name;
    request->xml = model_content;
    request->robot_namespace = robot_namespace;
    request->reference_frame = reference_frame;

    // send request
    auto future = _spawn_entity_client->async_send_request(request);
}

void EntityManager::deleteEntity(std::string entity_name)
{
    // init request
    std::shared_ptr<gazebo_msgs::srv::DeleteEntity::Request> request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    
    // set entity info
    request->name = entity_name;

    // send request
    auto future = _delete_entity_client->async_send_request(request);
}

void EntityManager::handleSpawnRobot(
    const std::shared_ptr<entity_manager_interfaces::srv::SpawnRobot::Request> request,
    std::shared_ptr<entity_manager_interfaces::srv::SpawnRobot::Response> response
)
{
    std::string entity_name = request->entity_name;
    std::string model_type = request->model_type;
    std::string robot_namespace = request->robot_namespace;
    std::string reference_frame = request->reference_frame;
    RCLCPP_INFO(
        this->get_logger(), 
        "get spawn robot request, entity_name:%s, model_type:%s, robot_namespace:%s, reference_frame:%s", 
        entity_name.c_str(), model_type.c_str(), robot_namespace.c_str(), reference_frame.c_str()
    );
    spawnEntity(entity_name, model_type, robot_namespace, reference_frame);
    response->success = true;
}

void EntityManager::handleDeleteRobot(
    const std::shared_ptr<entity_manager_interfaces::srv::DeleteRobot::Request> request,
    std::shared_ptr<entity_manager_interfaces::srv::DeleteRobot::Response> response
)
{
    std::string entity_name = request->entity_name;
    RCLCPP_INFO(
        this->get_logger(), 
        "get delete robot request, entity_name:%s", 
        entity_name.c_str()
    );
    deleteEntity(entity_name);
    response->success = true;
}


int main(int argc, char** argv)
{
    // init ROS2 node
    rclcpp::init(argc, argv);

    // create VRPNListener entity
    std::string name = "entity_manager";
    auto entity_manager = std::make_shared<EntityManager>(name);

    // run ROS2 node
    rclcpp::spin(entity_manager);

    // shutdown ROS2 node
    rclcpp::shutdown();

    return 0;
}