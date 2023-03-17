#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <stdio.h>
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_nav_interfaces/srv/set_body_rpy.hpp"

using namespace std::chrono_literals;

enum class HGRCode : uint8_t
{
     no_data = 99,
     open = 0
    ,close = 1
    ,pointer = 2
    ,ok = 3
    ,peace = 4
    ,thumbs_up = 5
    ,thumbs_down = 6
    ,quiet_coyote = 7
};

class HGRCom : public rclcpp::Node
{
  public:
    HGRCom()
    : Node("hgr_com")
    {
      //Publishers
      cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      // hgr_topic subscriber
      hgr_sub_ = create_subscription<std_msgs::msg::Int32>(
                  "/hgr_topic",
                  10, std::bind(&HGRCom::hgr_callback, this, std::placeholders::_1));

      // service clients
      standup_client = create_client<std_srvs::srv::Empty>("stand_up");

      laydown_client = create_client<std_srvs::srv::Empty>("lay_down");

      recoverstand_client = create_client<std_srvs::srv::Empty>("recover_stand");

      setbodyrpy_client = create_client<unitree_nav_interfaces::srv::SetBodyRPY>("set_body_rpy");

      // timer
      timer_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&HGRCom::timer_callback, this));
        // previously 5ms
      
      RCLCPP_INFO_STREAM(get_logger(), "Waiting...");
    }

  private:

    /// @brief Continuously running timer callback for sending commands to the Go1.
    void timer_callback()
    {
      geometry_msgs::msg::Twist cmdvel_msg;

      if (laydown_flag == 1)    // if the Go1 is laying down, need to stand up before doing anything else
      {
        if (hgr_code == HGRCode::pointer)
        {
          auto request = std::make_shared<std_srvs::srv::Empty::Request>();
          auto result = recoverstand_client->async_send_request(request);
          laydown_flag = 0;
        }
      }

      else {
        switch (hgr_code)
        {
          case HGRCode::no_data:  // Go1 should not move
          {
            srv_call_flag = 0;
            cmdvel_msg.linear.x = 0.0;
            cmdvel_msg.linear.y = 0.0;
            cmdvel_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmdvel_msg);
            break;
          }
          case HGRCode::open:   // tell the Go1 to stop moving
          {
            cmdvel_msg.linear.x = 0.0;
            cmdvel_msg.linear.y = 0.0;
            cmdvel_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmdvel_msg);
            break;
          }
          case HGRCode::close: // tell the Go1 to go back to normal orientation
          {
            auto request = std::make_shared<unitree_nav_interfaces::srv::SetBodyRPY::Request>();
            request->roll = 0.0;
            request->pitch = 0.0;
            request->yaw = 0.0;
            auto result = setbodyrpy_client->async_send_request(request);
            break;
          }
          case HGRCode::pointer: // tell the Go1 to stand up
          {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = recoverstand_client->async_send_request(request);
            break;
          }
          case HGRCode::ok:     // tell the Go1 to tilt back
          {
            auto request = std::make_shared<unitree_nav_interfaces::srv::SetBodyRPY::Request>();
            request->roll = 0.0;
            request->pitch = -0.6;
            request->yaw = 0.0;
            auto result = setbodyrpy_client->async_send_request(request);
            break;
          }
          case HGRCode::peace:  // tell the Go1 to tilt forward
          {
            auto request = std::make_shared<unitree_nav_interfaces::srv::SetBodyRPY::Request>();
            request->roll = 0.0;
            request->pitch = 0.3;
            request->yaw = 0.0;
            auto result = setbodyrpy_client->async_send_request(request);
            break;
          }
          case HGRCode::thumbs_up:    // tell the Go1 to walk forward
          {
            cmdvel_msg.linear.x = 0.3;
            cmdvel_msg.linear.y = 0.0;
            cmdvel_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmdvel_msg);
            break;
          }
          case HGRCode::thumbs_down:  // tell the Go1 to walk backward
          {
            cmdvel_msg.linear.x = -0.3;
            cmdvel_msg.linear.y = 0.0;
            cmdvel_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmdvel_msg);
            break;
          }
          case HGRCode::quiet_coyote:   // tell the Go1 to lay down
          {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = laydown_client->async_send_request(request);
            laydown_flag = 1;
            break;
          }
        }
      }

    }

    /// @brief Subscription callback to receive hand gesture recognition data.
    /// @param msg - HGR data
    void hgr_callback(const std_msgs::msg::Int32 & msg)
    {
        if (msg.data == -1)
        {
          hgr_code = HGRCode::no_data;
        }
        else 
        {
          hgr_code = static_cast<HGRCode>(msg.data);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hgr_sub_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr standup_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr laydown_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr recoverstand_client;
    rclcpp::Client<unitree_nav_interfaces::srv::SetBodyRPY>::SharedPtr setbodyrpy_client;

    int count = 0;
    long motiontime = 0;
    int srv_call_flag = 0;
    int laydown_flag = 0;
    HGRCode hgr_code = HGRCode::no_data;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HGRCom>());
  rclcpp::shutdown();
  return 0;
}
