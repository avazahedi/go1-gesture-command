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

using namespace std::chrono_literals;

enum class HGRCode : uint8_t
{
     no_data = 99,
     open = 0
    // ,close = 1
    // ,pointer = 2
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
      rclcpp::Client<std_srvs::srv::Empty>::SharedPtr standup_client =
      create_client<std_srvs::srv::Empty>("stand_up");

      rclcpp::Client<std_srvs::srv::Empty>::SharedPtr laydown_client =
      create_client<std_srvs::srv::Empty>("lay_down");

      rclcpp::Client<std_srvs::srv::Empty>::SharedPtr recoverstand_client =
      create_client<std_srvs::srv::Empty>("recover_stand");

      /////// TEST //////////
      srv_test = create_service<std_srvs::srv::Empty>(
        "test",
        std::bind(&HGRCom::test_callback, this,
                  std::placeholders::_1, std::placeholders::_2)
      );

      rclcpp::Client<std_srvs::srv::Empty>::SharedPtr test_client =
      create_client<std_srvs::srv::Empty>("test");
      //////////////////////////////

      // timer
      timer_ = create_wall_timer(
        std::chrono::milliseconds(5), std::bind(&HGRCom::timer_callback, this));
      
      RCLCPP_INFO_STREAM(get_logger(), "Waiting...");
    }

  private:
    void timer_callback()
    {
      geometry_msgs::msg::Twist cmdvel_msg;

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
        case HGRCode::ok:
        {
          // auto request = std::make_shared<std_srvs::srv::Empty::Request>();
          // auto result = standup_client->async_send_request(request);
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
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            RCLCPP_INFO_STREAM(get_logger(), "lay down REQUEST");
            RCLCPP_INFO_STREAM(get_logger(), "laydown client status " << laydown_client);

            auto result = laydown_client->async_send_request(request);
            RCLCPP_INFO_STREAM(get_logger(), "lay down service CALLED");

            // RCLCPP_INFO_STREAM(get_logger(), "test async request");
            // auto result = test_client->async_send_request(request);
            // RCLCPP_INFO_STREAM(get_logger(), "test async called");
            srv_call_flag = 1;

            // RCLCPP_INFO_STREAM(get_logger(), "client waiting for service " << laydown_client->wait_for_service(std::chrono::seconds(1)));
            // if (laydown_client->service_is_ready())
            // {
            //   RCLCPP_INFO_STREAM(get_logger(), "lay down service READY");
            //   auto result = laydown_client->async_send_request(request);
            //   RCLCPP_INFO_STREAM(get_logger(), "lay down service CALLED");
            //   srv_call_flag = 1;
            // }
          }
          break;
        }
      }

    }

    void test_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>
    ) {
      RCLCPP_INFO_STREAM(get_logger(), "INSIDE TEST CALLBACK");
    }

    void hgr_callback(const std_msgs::msg::Int32 & msg)
    {
        // if (msg.data == 5)
        // {
        //     hgr_code = HGRCode::WalkForward;
        // }
        // if (msg.data == 0)
        // {
        //     hgr_code = HGRCode::Stop;
        // }

        hgr_code = static_cast<HGRCode>(msg.data);
        // RCLCPP_INFO_STREAM(get_logger(), "hgr msg from callback: " << msg.data);
        // if (hgr_code != prev_code)
        // {
        //   switch (hgr_code)
        //   {
        //     case HGRCode::no_data:
        //     {
        //       break;
        //     }
        //     case HGRCode::open:   // tell the Go1 to stop moving
        //     {
        //       geometry_msgs::msg::Twist cmdvel_msg;
        //       cmd_vel_pub_->publish(cmdvel_msg);
        //       break;
        //     }
        //     case HGRCode::ok:
        //     {
        //       // auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        //       // auto result = standup_client->async_send_request(request);
        //       break;
        //     }
        //     case HGRCode::thumbs_up:
        //     {
        //       geometry_msgs::msg::Twist cmdvel_msg;
        //       // cmdvel_msg.linear.x = 0.0;
        //       // cmdvel_msg.linear.y = 0.0;
        //       // cmdvel_msg.angular.z = 0.0;
        //       cmd_vel_pub_->publish(cmdvel_msg);
        //       break;
        //     }
        //     case HGRCode::thumbs_down:
        //     {
        //       auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        //       auto result = laydown_client->async_send_request(request);
        //       break;
        //     }
        //   }
        //   prev_code = hgr_code;
        // }

    }

    rclcpp::TimerBase::SharedPtr timer_;
    // ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;
    // rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hgr_sub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_test;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr test_client;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr standup_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr laydown_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr recoverstand_client;

    int count = 0;
    long motiontime = 0;
    int srv_call_flag = 0;
    HGRCode hgr_code = HGRCode::no_data;
    HGRCode prev_code = HGRCode::no_data;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HGRCom>());
  rclcpp::shutdown();
  return 0;
}
