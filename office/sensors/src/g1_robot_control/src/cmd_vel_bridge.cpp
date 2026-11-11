#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "common/time_tools.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "nlohmann/json.hpp"
#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"

namespace {
constexpr int32_t kLocoSetVelocityApiId = 7105;
}

class G1CmdVelBridge : public rclcpp::Node {
 public:
  G1CmdVelBridge() : Node("g1_cmd_vel_bridge") {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    max_vx_ = declare_parameter<double>("max_vx", 0.5);
    max_vy_ = declare_parameter<double>("max_vy", 0.3);
    max_omega_ = declare_parameter<double>("max_omega", 0.4);
    deadzone_xy_ = declare_parameter<double>("deadzone_xy", 0.1);
    web_joystick_ =
        declare_parameter<bool>("web_joystick", false);
    command_duration_ = declare_parameter<double>("command_duration", 0.25);
    watchdog_timeout_ = declare_parameter<double>("watchdog_timeout", 1.0);
    stop_publish_period_ = declare_parameter<double>("stop_publish_period", 0.5);
    send_stop_on_timeout_ = declare_parameter<bool>("send_stop_on_timeout", true);
    log_commands_ = declare_parameter<bool>("log_commands", false);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          HandleCmdVel(*msg);
        });
    sport_request_pub_ = create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", rclcpp::QoS(1));

    worker_ = std::thread([this] { CommandLoop(); });

    RCLCPP_INFO(get_logger(),
                "G1 cmd_vel bridge listening on %s, normalized input [-1, 1], "
                "scale vx=%.3f vy=%.3f omega=%.3f, xy deadzone=%.3f, "
                "web joystick=%s, duration=%.3f, watchdog=%.3f",
                cmd_vel_topic_.c_str(), max_vx_, max_vy_, max_omega_,
                deadzone_xy_, web_joystick_ ? "true" : "false",
                command_duration_, watchdog_timeout_);
  }

  ~G1CmdVelBridge() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  struct VelocityCommand {
    float vx = 0.0F;
    float vy = 0.0F;
    float omega = 0.0F;
    rclcpp::Time stamp;
    uint64_t sequence = 0;
  };

  void HandleCmdVel(const geometry_msgs::msg::Twist &msg) {
    VelocityCommand command;
    const auto normalized_x = ApplyDeadzone(ClampUnit(msg.linear.x), deadzone_xy_);
    const auto normalized_y = ApplyDeadzone(ClampUnit(msg.linear.y), deadzone_xy_);
    const auto normalized_z = ClampUnit(msg.angular.z);

    if (web_joystick_) {
      command.vx = static_cast<float>(normalized_y * max_vx_);
      command.vy = static_cast<float>(-normalized_x * max_vy_);
    } else {
      command.vx = static_cast<float>(normalized_x * max_vx_);
      command.vy = static_cast<float>(-normalized_y * max_vy_);
    }
    command.omega = static_cast<float>(-normalized_z * max_omega_);
    command.stamp = now();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      command.sequence = ++sequence_;
      latest_command_ = command;
      have_command_ = true;
      timeout_active_ = false;
    }
    condition_.notify_one();
  }

  void CommandLoop() {
    uint64_t sent_sequence = 0;
    rclcpp::Time last_timeout_stop_time(0, 0, get_clock()->get_clock_type());

    while (rclcpp::ok()) {
      VelocityCommand command;
      bool send_command = false;
      bool send_timeout_stop = false;

      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(20), [this] {
          return !running_ ||
                 (have_command_ && latest_command_.sequence != sent_sequence_);
        });

        if (!running_) {
          break;
        }

        const auto now_time = now();
        if (have_command_ && latest_command_.sequence != sent_sequence) {
          command = latest_command_;
          sent_sequence = command.sequence;
          sent_sequence_ = sent_sequence;
          send_command = true;
        } else if (send_stop_on_timeout_ && have_command_ &&
                   (now_time - latest_command_.stamp).seconds() >
                       watchdog_timeout_) {
          const auto should_repeat_stop =
              !timeout_active_ ||
              (now_time - last_timeout_stop_time).seconds() >=
                  stop_publish_period_;
          if (should_repeat_stop) {
            timeout_active_ = true;
            last_timeout_stop_time = now_time;
            send_timeout_stop = true;
          }
        }
      }

      if (send_command) {
        SendVelocity(command.vx, command.vy, command.omega,
                     static_cast<float>(command_duration_));
      } else if (send_timeout_stop) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "cmd_vel timeout, publishing zero velocity");
        SendVelocity(0.0F, 0.0F, 0.0F,
                     static_cast<float>(command_duration_));
      }
    }
  }

  void SendVelocity(float vx, float vy, float omega, float duration) {
    if (log_commands_) {
      RCLCPP_INFO(get_logger(), "SetVelocity vx=%.3f vy=%.3f omega=%.3f",
                  vx, vy, omega);
    }

    unitree_api::msg::Request req;
    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    req.header.identity.api_id = kLocoSetVelocityApiId;

    nlohmann::json js;
    js["velocity"] = std::vector<float>{vx, vy, omega};
    js["duration"] = duration;
    req.parameter = js.dump();

    sport_request_pub_->publish(req);
  }

  static double ClampUnit(double value) {
    if (!std::isfinite(value)) {
      return 0.0;
    }
    return std::clamp(value, -1.0, 1.0);
  }

  static double ApplyDeadzone(double value, double deadzone) {
    const auto threshold = std::clamp(std::abs(deadzone), 0.0, 1.0);
    if (std::abs(value) < threshold) {
      return 0.0;
    }
    return value;
  }

  std::string cmd_vel_topic_;
  double max_vx_ = 0.5;
  double max_vy_ = 0.3;
  double max_omega_ = 0.4;
  double deadzone_xy_ = 0.1;
  bool web_joystick_ = false;
  double command_duration_ = 0.25;
  double watchdog_timeout_ = 1.0;
  double stop_publish_period_ = 0.5;
  bool send_stop_on_timeout_ = true;
  bool log_commands_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_request_pub_;

  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  VelocityCommand latest_command_;
  bool running_ = true;
  bool have_command_ = false;
  bool timeout_active_ = false;
  uint64_t sequence_ = 0;
  uint64_t sent_sequence_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1CmdVelBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
