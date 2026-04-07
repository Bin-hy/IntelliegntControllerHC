/**
 * @file glove_hand_bridge.cpp
 * @brief Bridge node: subscribes to UDE glove finger data and drives the
 *        DH116 dexterous hand in real-time via lhandpro_service.
 *
 * Mapping (left hand glove → left DH116 hand):
 *   Motor 1 (thumb spread)  ← Thumb1.Y  (abduction / adduction)
 *   Motor 2 (thumb bend)    ← avg(Thumb1.X, Thumb2.X, Thumb3.X)  (flexion)
 *   Motor 3 (index bend)    ← avg(Index1.X, Index2.X, Index3.X)
 *   Motor 4 (middle bend)   ← avg(Middle1.X, Middle2.X, Middle3.X)
 *   Motor 5 (ring bend)     ← avg(Ring1.X, Ring2.X, Ring3.X)
 *   Motor 6 (pinky bend)    ← avg(Pinky1.X, Pinky2.X, Pinky3.X)
 *
 * The glove provides Euler angles in degrees. We map them to the 0-10000
 * position range used by the DH116 hand.
 */

#include <rclcpp/rclcpp.hpp>
#include <common_msgs/msg/glove_joints.hpp>
#include <common_msgs/msg/device_status.hpp>
#include <lhandpro_interfaces/srv/set_all_position.hpp>
#include <lhandpro_interfaces/srv/move_motors.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class GloveHandBridge : public rclcpp::Node
{
public:
  GloveHandBridge() : Node("glove_hand_bridge")
  {
    // ---- Parameters ----
    this->declare_parameter("hand_namespace", std::string("lhandpro_service"));
    this->declare_parameter("hand_side", std::string("left"));   // "left" or "right"
    this->declare_parameter("control_rate_hz", 20.0);            // control loop rate
    this->declare_parameter("smoothing_alpha", 0.3);             // EMA filter: 0=no update, 1=no filter
    this->declare_parameter("deadzone_deg", 2.0);                // ignore changes smaller than this
    this->declare_parameter("enabled_at_start", false);          // require explicit enable

    // Glove angle range parameters (degrees) — calibrate per user
    this->declare_parameter("glove_flex_min", 0.0);    // fully open (degrees)
    this->declare_parameter("glove_flex_max", 90.0);   // fully closed (degrees)
    this->declare_parameter("glove_spread_min", -30.0);
    this->declare_parameter("glove_spread_max", 30.0);

    hand_ns_     = this->get_parameter("hand_namespace").as_string();
    hand_side_   = this->get_parameter("hand_side").as_string();
    alpha_       = this->get_parameter("smoothing_alpha").as_double();
    deadzone_    = this->get_parameter("deadzone_deg").as_double();
    enabled_     = this->get_parameter("enabled_at_start").as_bool();
    flex_min_    = this->get_parameter("glove_flex_min").as_double();
    flex_max_    = this->get_parameter("glove_flex_max").as_double();
    spread_min_  = this->get_parameter("glove_spread_min").as_double();
    spread_max_  = this->get_parameter("glove_spread_max").as_double();

    double control_rate = this->get_parameter("control_rate_hz").as_double();
    if (control_rate <= 0) control_rate = 20.0;
    control_period_ms_ = static_cast<int>(1000.0 / control_rate);

    // Determine glove joint offset: left hand joints = indices 0-14, right = 15-29
    // In the glove/joints message, each joint emits 3 entries (x, y, z)
    // Left hand: entries 0..44  (15 joints * 3), Right hand: entries 45..89
    glove_offset_ = (hand_side_ == "right") ? 15 : 0;

    // ---- Service clients ----
    set_all_pos_client_ = this->create_client<lhandpro_interfaces::srv::SetAllPosition>(
        "/" + hand_ns_ + "/set_all_position");
    move_motors_client_ = this->create_client<lhandpro_interfaces::srv::MoveMotors>(
        "/" + hand_ns_ + "/move_motors");

    // ---- Enable/disable service ----
    enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          enabled_ = req->data;
          res->success = true;
          res->message = enabled_ ? "Glove-hand bridge enabled" : "Glove-hand bridge disabled";
          RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
        });

    // ---- Subscriber: glove data ----
    glove_sub_ = this->create_subscription<common_msgs::msg::GloveJoints>(
        "glove/joints", 10,
        std::bind(&GloveHandBridge::glove_callback, this, std::placeholders::_1));

    // ---- Status publisher ----
    pub_device_status_ = this->create_publisher<common_msgs::msg::DeviceStatus>(
        "/system/device_status", 10);

    // ---- Control timer ----
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(control_period_ms_),
        std::bind(&GloveHandBridge::control_loop, this));

    // ---- Status timer (1Hz) ----
    status_timer_ = this->create_wall_timer(
        1000ms, std::bind(&GloveHandBridge::publish_status, this));

    // Initialize state
    filtered_positions_.fill(0);
    last_sent_positions_.fill(-1);  // force first send

    RCLCPP_INFO(this->get_logger(),
        "GloveHandBridge initialized: hand=%s, rate=%.0fHz, alpha=%.2f, deadzone=%.1f deg, enabled=%s",
        hand_ns_.c_str(), control_rate, alpha_, deadzone_, enabled_ ? "true" : "false");
  }

private:
  // ---- Constants ----
  static constexpr int NUM_MOTORS = 6;
  static constexpr int POS_MIN = 0;
  static constexpr int POS_MAX = 10000;

  // Glove joint indices within a single hand (0-14):
  //   Thumb:  0(Thumb1), 1(Thumb2), 2(Thumb3)
  //   Index:  3(Index1), 4(Index2), 5(Index3)
  //   Middle: 6(Middle1), 7(Middle2), 8(Middle3)
  //   Ring:   9(Ring1), 10(Ring2), 11(Ring3)
  //   Pinky: 12(Pinky1), 13(Pinky2), 14(Pinky3)

  /**
   * @brief Extract a joint value from the flat GloveJoints message.
   *
   * The message stores: for each joint index i, three consecutive entries:
   *   [i*3+0] = x (flexion), [i*3+1] = y (abduction), [i*3+2] = z
   *
   * @param msg The glove joints message
   * @param joint_index Joint index within hand (0-14) + glove_offset_
   * @param axis 0=x, 1=y, 2=z
   */
  double get_joint_value(const common_msgs::msg::GloveJoints& msg,
                         int joint_index, int axis) const
  {
    int global_index = (glove_offset_ + joint_index) * 3 + axis;
    if (global_index < 0 || global_index >= static_cast<int>(msg.position.size())) {
      return 0.0;
    }
    return msg.position[global_index];
  }

  /**
   * @brief Map a degree value to 0-10000 position range.
   */
  int deg_to_position(double deg, double min_deg, double max_deg) const
  {
    if (max_deg <= min_deg) return POS_MIN;
    double ratio = (deg - min_deg) / (max_deg - min_deg);
    ratio = std::clamp(ratio, 0.0, 1.0);
    return static_cast<int>(ratio * POS_MAX);
  }

  /**
   * @brief Called on each glove/joints message. Extracts raw finger data
   *        and computes target motor positions.
   */
  void glove_callback(const common_msgs::msg::GloveJoints::SharedPtr msg)
  {
    if (msg->position.empty()) return;

    has_glove_data_ = true;
    last_glove_time_ = this->now();

    // ---- Motor 1: Thumb spread (Thumb1.Y = abduction) ----
    double thumb_spread = get_joint_value(*msg, 0, 1);  // Thumb1 Y-axis
    raw_positions_[0] = deg_to_position(thumb_spread, spread_min_, spread_max_);

    // ---- Motor 2: Thumb bend (average flexion of 3 thumb joints) ----
    double thumb_flex = (std::abs(get_joint_value(*msg, 0, 0)) +
                         std::abs(get_joint_value(*msg, 1, 0)) +
                         std::abs(get_joint_value(*msg, 2, 0))) / 3.0;
    raw_positions_[1] = deg_to_position(thumb_flex, flex_min_, flex_max_);

    // ---- Motor 3: Index bend ----
    double index_flex = (std::abs(get_joint_value(*msg, 3, 0)) +
                         std::abs(get_joint_value(*msg, 4, 0)) +
                         std::abs(get_joint_value(*msg, 5, 0))) / 3.0;
    raw_positions_[2] = deg_to_position(index_flex, flex_min_, flex_max_);

    // ---- Motor 4: Middle bend ----
    double middle_flex = (std::abs(get_joint_value(*msg, 6, 0)) +
                          std::abs(get_joint_value(*msg, 7, 0)) +
                          std::abs(get_joint_value(*msg, 8, 0))) / 3.0;
    raw_positions_[3] = deg_to_position(middle_flex, flex_min_, flex_max_);

    // ---- Motor 5: Ring bend ----
    double ring_flex = (std::abs(get_joint_value(*msg, 9, 0)) +
                        std::abs(get_joint_value(*msg, 10, 0)) +
                        std::abs(get_joint_value(*msg, 11, 0))) / 3.0;
    raw_positions_[4] = deg_to_position(ring_flex, flex_min_, flex_max_);

    // ---- Motor 6: Pinky bend ----
    double pinky_flex = (std::abs(get_joint_value(*msg, 12, 0)) +
                         std::abs(get_joint_value(*msg, 13, 0)) +
                         std::abs(get_joint_value(*msg, 14, 0))) / 3.0;
    raw_positions_[5] = deg_to_position(pinky_flex, flex_min_, flex_max_);
  }

  /**
   * @brief Periodic control loop: apply smoothing, deadzone, then send to hand.
   */
  void control_loop()
  {
    if (!enabled_ || !has_glove_data_) return;

    // Check for stale data (>500ms since last glove msg → stop sending)
    auto elapsed = (this->now() - last_glove_time_).seconds();
    if (elapsed > 0.5) {
      if (has_glove_data_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Glove data stale (%.1fs), pausing hand control", elapsed);
      }
      return;
    }

    // ---- EMA smoothing ----
    bool changed = false;
    for (int i = 0; i < NUM_MOTORS; ++i) {
      double smoothed = alpha_ * raw_positions_[i] + (1.0 - alpha_) * filtered_positions_[i];
      int pos = static_cast<int>(std::round(smoothed));
      pos = std::clamp(pos, POS_MIN, POS_MAX);
      filtered_positions_[i] = pos;

      // Deadzone: only mark as changed if difference is significant
      // Convert deadzone from degrees to position units
      int deadzone_pos = static_cast<int>((deadzone_ / (flex_max_ - flex_min_)) * POS_MAX);
      if (deadzone_pos < 50) deadzone_pos = 50;  // minimum deadzone in position units
      if (std::abs(filtered_positions_[i] - last_sent_positions_[i]) >= deadzone_pos) {
        changed = true;
      }
    }

    if (!changed) return;

    // ---- Send to lhandpro_service ----
    send_to_hand();
  }

  /**
   * @brief Send current filtered positions to the hand via SetAllPosition + MoveMotors.
   */
  void send_to_hand()
  {
    if (!set_all_pos_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "set_all_position service not available, waiting...");
      return;
    }

    // Don't pile up requests — skip if a previous call is still in flight
    if (request_in_flight_) return;
    request_in_flight_ = true;

    auto request = std::make_shared<lhandpro_interfaces::srv::SetAllPosition::Request>();
    for (int i = 0; i < NUM_MOTORS; ++i) {
      request->positions[i] = filtered_positions_[i];
      last_sent_positions_[i] = filtered_positions_[i];
    }

    RCLCPP_DEBUG(this->get_logger(),
        "Sending positions: [%d, %d, %d, %d, %d, %d]",
        filtered_positions_[0], filtered_positions_[1],
        filtered_positions_[2], filtered_positions_[3],
        filtered_positions_[4], filtered_positions_[5]);

    set_all_pos_client_->async_send_request(request,
        [this](rclcpp::Client<lhandpro_interfaces::srv::SetAllPosition>::SharedFuture future) {
          auto result = future.get();
          if (result->result != 0) {
            RCLCPP_WARN(this->get_logger(), "set_all_position failed: %d", result->result);
            request_in_flight_ = false;
            return;
          }
          // Now trigger move_motors
          trigger_move();
        });
  }

  /**
   * @brief Trigger move_motors(0) to execute all pending position changes.
   */
  void trigger_move()
  {
    if (!move_motors_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "move_motors service not available");
      request_in_flight_ = false;
      return;
    }

    auto request = std::make_shared<lhandpro_interfaces::srv::MoveMotors::Request>();
    request->joint_id = 0;  // 0 = move all

    move_motors_client_->async_send_request(request,
        [this](rclcpp::Client<lhandpro_interfaces::srv::MoveMotors>::SharedFuture future) {
          auto result = future.get();
          if (result->result != 0) {
            RCLCPP_WARN(this->get_logger(), "move_motors failed: %d", result->result);
          }
          request_in_flight_ = false;
        });
  }

  /**
   * @brief Publish device status at 1Hz.
   */
  void publish_status()
  {
    common_msgs::msg::DeviceStatus status;
    status.device_type = "glove_bridge";
    status.device_usage = "teleoperation";
    status.device_model = "GloveHandBridge";
    status.device_name = "Glove-Hand Bridge (" + hand_side_ + ")";
    status.device_sn = "bridge_" + hand_side_;

    if (!has_glove_data_) {
      status.status = "waiting";  // waiting for glove data
    } else if (!enabled_) {
      status.status = "disabled";
    } else if (!set_all_pos_client_->service_is_ready()) {
      status.status = "no_hand";  // hand service not available
    } else {
      status.status = "active";
    }

    pub_device_status_->publish(status);
  }

  // ---- Parameters ----
  std::string hand_ns_;
  std::string hand_side_;
  double alpha_;         // EMA smoothing factor
  double deadzone_;      // deadzone in degrees
  double flex_min_;
  double flex_max_;
  double spread_min_;
  double spread_max_;
  int control_period_ms_;
  int glove_offset_;     // 0 for left hand, 15 for right hand

  // ---- State ----
  std::atomic<bool> enabled_{false};
  bool has_glove_data_{false};
  bool request_in_flight_{false};
  rclcpp::Time last_glove_time_;
  std::array<int, NUM_MOTORS> raw_positions_{};
  std::array<int, NUM_MOTORS> filtered_positions_{};
  std::array<int, NUM_MOTORS> last_sent_positions_{};

  // ---- ROS interfaces ----
  rclcpp::Subscription<common_msgs::msg::GloveJoints>::SharedPtr glove_sub_;
  rclcpp::Client<lhandpro_interfaces::srv::SetAllPosition>::SharedPtr set_all_pos_client_;
  rclcpp::Client<lhandpro_interfaces::srv::MoveMotors>::SharedPtr move_motors_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Publisher<common_msgs::msg::DeviceStatus>::SharedPtr pub_device_status_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GloveHandBridge>());
  rclcpp::shutdown();
  return 0;
}
