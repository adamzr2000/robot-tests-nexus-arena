#include <unistd.h>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "go2_srvs/srv/go2_modes.hpp" 
#include "go2_srvs/srv/go2_light.hpp"
#include "go2_srvs/srv/go2_volume.hpp"

#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "utils/colors.h"

using std::placeholders::_1;


class go2_highlevel_request : public rclcpp::Node
{
public:
    go2_highlevel_request() : Node("go2_highroscontrol")
    {   
        // Initialization
        RCLCPP_INFO(get_logger(), BOLD(FGRN("Initializing GO2 ROS Hardware Communication")));
        
        
        // Subscribers        
        sub_go2_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&go2_highlevel_request::callback_go2_cmd, this, _1));

        // Publishers
        pub_go2_api = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        pub_go2_vui = this->create_publisher<unitree_api::msg::Request>("/api/vui/request", 10);

        // Service
        srv_go2_modes = this->create_service<go2_srvs::srv::Go2Modes>(
            "modes", std::bind(&go2_highlevel_request::callback_go2_modes, this, std::placeholders::_1, std::placeholders::_2));
        srv_go2_light = this->create_service<go2_srvs::srv::Go2Light>(
            "light", std::bind(&go2_highlevel_request::callback_go2_light, this, std::placeholders::_1, std::placeholders::_2));
        srv_go2_volume = this->create_service<go2_srvs::srv::Go2Volume>(
            "volume", std::bind(&go2_highlevel_request::callback_go2_volume, this, std::placeholders::_1, std::placeholders::_2));

        // Provide Mode Info
        printAllModes();
    };

private:
    void callback_go2_cmd(geometry_msgs::msg::Twist::SharedPtr msg)
    {   
        float linear_x  = 0.0;
        float linear_y  = 0.0;
        float angular_z = 0.0;   
    
        if (msg->linear.x && msg->linear.y && msg->angular.z) {
            linear_x = msg->linear.x;
            linear_y = msg->linear.y;
            angular_z = msg->angular.z;
        }
        else if (msg->linear.x && msg->linear.y) {
            linear_x = msg->linear.x;
            linear_y = msg->linear.y;
        }
        else if (msg->linear.x && msg->angular.z) {
            linear_x = msg->linear.x;
            angular_z = msg->angular.z;
        }
        else if (msg->linear.x) {
            linear_x = msg->linear.x;        
        }
        else if (msg->linear.y) {
            linear_y = msg->linear.y;        
        }
        else if (msg->angular.z) {
            angular_z = msg->angular.z;
        }
        else {     
            return ;
        } 
        sport_client.Move(req, linear_x, linear_y, angular_z);
        pub_go2_api->publish(req);
    }
    
    // Callback for Subscribers and Services
    void callback_go2_volume(const std::shared_ptr<go2_srvs::srv::Go2Volume::Request> request,
                                   std::shared_ptr<go2_srvs::srv::Go2Volume::Response> response)
    {
        try
        {
            RCLCPP_INFO(get_logger(), BOLD(FORA("Service request received for volume level: ")) FBLU("%d"), request->volume_level);
            bool success = go2_volume_configuration(request);

            response->success = success;

            if (success)
            {
                response->reason = "Volume configuration set successfully";
            }
            else
            {
                response->reason = "Failed to set Volume configuration";
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception in callback_go2_volume: %s", e.what());
        }
    }

    void callback_go2_light(const std::shared_ptr<go2_srvs::srv::Go2Light::Request> request,
                                   std::shared_ptr<go2_srvs::srv::Go2Light::Response> response)
    {
        try
        {
            RCLCPP_INFO(get_logger(), BOLD(FORA("Service request received for light level: ")) FBLU("%d"), request->light_level);
            bool success = go2_light_configuration(request);

            response->success = success;

            if (success)
            {
                response->reason = "Light configuration set successfully";
            }
            else
            {
                response->reason = "Failed to set light configuration";
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception in callback_go2_light: %s", e.what());
        }
    }

    void callback_go2_modes(const std::shared_ptr<go2_srvs::srv::Go2Modes::Request> request,
                                  std::shared_ptr<go2_srvs::srv::Go2Modes::Response> response)
    {
        try
        {
            RCLCPP_INFO(get_logger(), BOLD(FORA("Service request received: ")) FBLU("%s"), request->request_data.c_str());

            bool success = go2_mode_selection(request);

            response->success = success;

            if (success)
            {
                response->reason = "GO2 mode set successfully";
            }
            else
            {
                response->reason = "Failed to set GO2 mode";
            }

        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Exception in callback_go2_modes: %s", e.what());
        }
    }
    
    bool go2_volume_configuration(const std::shared_ptr<go2_srvs::srv::Go2Volume::Request> request)
    {
        if (request->volume_level >= 0 && request->volume_level <= 10)
        {
            req.header.identity.id = 0;
            req.header.identity.api_id = 1003;
            req.header.lease.id = 0;
            req.header.policy.priority = 0;
            req.header.policy.noreply = false;
            req.parameter = "{\"volume\":" + std::to_string(request->volume_level) + "}";
            req.binary = {};
            pub_go2_vui->publish(req);
            return true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown light configuration received: %d", request->volume_level);
            return false;
        }
    }

    bool go2_light_configuration(const std::shared_ptr<go2_srvs::srv::Go2Light::Request> request)
    {
        if (request->light_level >= 0 && request->light_level <= 10)
        {
            req.header.identity.id = 0;
            req.header.identity.api_id = 1005;
            req.header.lease.id = 0;
            req.header.policy.priority = 0;
            req.header.policy.noreply = false;
            req.parameter = "{\"brightness\":" + std::to_string(request->light_level) + "}";
            req.binary = {};
            pub_go2_vui->publish(req);
            return true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown light configuration received: %d", request->light_level);
            return false;
        }
    }

    bool go2_mode_selection(const std::shared_ptr<go2_srvs::srv::Go2Modes::Request> request)
    {   
        if (request->request_data == "normal_stand") {
            sport_client.SwitchGait(req, 0); 
            sport_client.StandUp(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "gait_type_idle") {
            sport_client.SwitchGait(req, 0);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "gait_type_trot") {
            sport_client.SwitchGait(req, 1);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "gait_type_trot_running") {
            sport_client.SwitchGait(req, 2);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "gait_type_climb") {
            sport_client.SwitchGait(req, 3);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "gait_type_obstacle") {
            sport_client.SwitchGait(req, 4);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "balance_stand") {
            sport_client.Euler(req, 0.1, 0.2, 0.3);
            sport_client.BodyHeight(req, 0.0);
            sport_client.BalanceStand(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "stand_down") {
            sport_client.StandDown(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "stand_up") {
            sport_client.StandUp(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "damp") {
            sport_client.Damp(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "recovery_stand") {
            sport_client.RecoveryStand(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "sit") {
            sport_client.Sit(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "rise_sit") {
            sport_client.RiseSit(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "stretch") {
            sport_client.Stretch(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "wallow") {
            sport_client.Wallow(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "pose") {
            sport_client.Pose(req, true);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "front_jump") {
            sport_client.FrontJump(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "front_pounce") {
            sport_client.FrontPounce(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "front_flip") {
            sport_client.FrontFlip(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "scrape") {
            sport_client.Scrape(req);
            pub_go2_api->publish(req);
            return true;
        } else if (request->request_data == "stop_move") {
            sport_client.StopMove(req);
            pub_go2_api->publish(req);
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown mode received: %s", request->request_data.c_str());
            sport_client.StopMove(req);
            pub_go2_api->publish(req);
            return false;
        }
    }
    
    void printAllModes()
    {  
        RCLCPP_INFO(get_logger(), BOLD(FBLU("Available GO2 Modes via ROS2 Services:")) );

        std::vector<std::string> modes = {
            "normal_stand",
            "balance_stand",
            "recovery_stand",
            "stand_down",
            "stand_up",
            "gait_type_idle",
            "gait_type_trot",
            "gait_type_trot_running",
            "gait_type_climb",
            "gait_type_obstacle",
            "stretch",
            "wallow",
            "damp",
            "sit",
            "rise_sit",
            "front_jump",
            "front_pounce",
            "front_flip",
            "stop_move",
        };

        for (const auto& mode : modes)
        {
            RCLCPP_INFO(get_logger(), FCYN("\t- %s"), mode.c_str());
        }

        RCLCPP_INFO(get_logger(), BOLD(FBLU("Available GO2 Functionalities via ROS2 Services:")) );

        std::vector<std::string> functionalities = {
            "Light level:  0-10",
            "Volume level: 0-10",
        };

        for (const auto& functionalities : functionalities)
        {
            RCLCPP_INFO(get_logger(), FCYN("\t- %s"), functionalities.c_str());
        }
    }

    // Initialize Subscribers 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_go2_cmd_vel;

    // Initialize Publishers    
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_go2_api;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_go2_vui;

    // Initialize Service
    rclcpp::Service<go2_srvs::srv::Go2Modes>::SharedPtr srv_go2_modes;
    rclcpp::Service<go2_srvs::srv::Go2Light>::SharedPtr srv_go2_light;
    rclcpp::Service<go2_srvs::srv::Go2Volume>::SharedPtr srv_go2_volume;
    
    // Initialize Unitree Api 
    unitree_api::msg::Request req; 
    SportClient sport_client;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<go2_highlevel_request>();
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "\x1B[1m\x1B[31mTerminating GO2 ROS Hardware Communication\x1B[0m");
    rclcpp::shutdown();
    return 0;
}
