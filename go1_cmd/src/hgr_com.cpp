#include "rclcpp/rclcpp.hpp"
// #include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
// #include "ros2_unitree_legged_msgs/msg/high_state.hpp"
// #include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
// #include "ros2_unitree_legged_msgs/msg/low_state.hpp"
// #include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include "convert.h"
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
    // ,peace = 4
    ,thumbs_up = 5
    ,thumbs_down = 6
    ,quiet_coyote = 7
};

// enum class Go1Mode : uint8_t
// {
//    idle = 0
//   ,force_stand = 1
//   ,target_velocity_walking = 2
//   ,target_position_walking = 3
//   ,path_walking = 4
//   ,position_stand_down = 5
//   ,position_stand_up = 6
//   ,damping = 7
//   ,recovery_stand = 8
//   ,backflip = 9
//   ,jump_yaw = 10
//   ,straight_hand = 11
//   ,dance1 = 12
//   ,dance2 = 13
// };


class HGRCom : public rclcpp::Node
{
  public:
    HGRCom()
    : Node("hgr_com")
    {
      // high_cmd publisher
      // cmd_pub_ = create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 10);

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
    void timer_callback()
    {
      geometry_msgs::msg::Twist cmdvel_msg;
      // RCLCPP_INFO_STREAM(get_logger(), "srv_call_flag: " << srv_call_flag);
      // RCLCPP_INFO_STREAM(get_logger(), "hgr_code: " << static_cast<int>(hgr_code));

      if (laydown_flag == 1)
      {
        if (hgr_code == HGRCode::pointer)
        {
          auto request = std::make_shared<std_srvs::srv::Empty::Request>();
          auto result = recoverstand_client->async_send_request(request);
          laydown_flag = 0;
        }
      }

      else{
        switch (hgr_code)
        {
          case HGRCode::no_data:
          {
            srv_call_flag = 0;
            // tell the Go1 to stop moving (for now)
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
            if (srv_call_flag == 0)
            {
              auto request = std::make_shared<unitree_nav_interfaces::srv::SetBodyRPY::Request>();
              request->roll = 0.0;
              request->pitch = 0.0;
              request->yaw = 0.0;
              auto result = setbodyrpy_client->async_send_request(request);
              // srv_call_flag = 1;
            }
            break;
          }
          case HGRCode::pointer: // tell the Go1 to stand up
          {
            if (srv_call_flag == 0)
            {
              auto request = std::make_shared<std_srvs::srv::Empty::Request>();
              auto result = recoverstand_client->async_send_request(request);
              // srv_call_flag = 1;
            }
            break;
          }
          case HGRCode::ok:
          {
            if (srv_call_flag == 0)
            {
              auto request = std::make_shared<unitree_nav_interfaces::srv::SetBodyRPY::Request>();
              request->roll = 0.0;
              request->pitch = -0.6;
              request->yaw = 0.0;
              auto result = setbodyrpy_client->async_send_request(request);
              // while (result.wait_for(1s) != std::future_status::ready) 
              // {
              //   RCLCPP_INFO_STREAM(get_logger(), "AHHHAHDSUAHDUAHSFDHJJSDHF");
              // } // wait for future to finish !!! this does not work !!!
              // RCLCPP_INFO_STREAM(get_logger(), "SetBodyRPY future complete");
              // auto result_future = setbodyrpy_client->async_send_request(
              //       request, std::bind(&HGRCom::response_callback, this,
              //                         std::placeholders::_1));
              // srv_call_flag = 1;
            }
            break;
          }
          case HGRCode::thumbs_up:
          {
            cmdvel_msg.linear.x = 0.3;
            cmdvel_msg.linear.y = 0.0;
            cmdvel_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmdvel_msg);
            break;
          }
          case HGRCode::thumbs_down:
          {
            cmdvel_msg.linear.x = -0.3;
            cmdvel_msg.linear.y = 0.0;
            cmdvel_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmdvel_msg);
            break;
          }
          case HGRCode::quiet_coyote:
          {
            if (srv_call_flag == 0)
            {
              // while (!laydown_client->wait_for_service(1s)) {
              //   if (rclcpp::ok()) {
              //     RCLCPP_ERROR(
              //         this->get_logger(),
              //         "Client interrupted while waiting for service. Terminating...");
              //     return;
              //   }
              //   RCLCPP_INFO(this->get_logger(),
              //               "Service Unavailable. Waiting for Service...");
              // }

              // auto request = std::make_shared<std_srvs::srv::Empty::Request>();

              // RCLCPP_INFO_STREAM(get_logger(), "BEFORE CALL");
              // auto result_future = laydown_client->async_send_request(
              //     request, std::bind(&HGRCom::response_callback, this,
              //                       std::placeholders::_1));
              // RCLCPP_INFO_STREAM(get_logger(), "AFTER CALL");

              auto request = std::make_shared<std_srvs::srv::Empty::Request>();
              auto result = laydown_client->async_send_request(request);
              laydown_flag = 1;
            }

            break;
          }
        }
      }

    }

    // void response_callback(
    //   rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
    //   auto status = future.wait_for(1s);
    //   if (status == std::future_status::ready) {
    //     RCLCPP_INFO(this->get_logger(), "Result: success");
    //   } else {
    //     RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    //   }
    // }

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
