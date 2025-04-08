/**
 * Joystick Servo Control Node
 *
 * This code implements a joystick-based control node that switches between two control modes:
 * 1. Twist mode - uses joystick axes for Cartesian control.
 * 2. Joint Jog mode - uses discrete button presses (detected on rising edge) to command individual
 * joints.
 *
 * Features:
 * - Two separate buttons switch the command mode (Twist or Joint Jog).
 * - In Twist mode:
 *     • Two separate buttons change the command frame (Planning frame or End-Effector frame).
 *     • Joystick axes are mapped as follows:
 *         - Left Stick X (axis 0)  --> linear y
 *         - Left Stick Y (axis 1)  --> linear z
 *         - Left Bumper (button 4)  --> linear x positive (on rising edge)
 *         - Right Bumper (button 5) --> linear x negative (on rising edge)
 *         - Right Stick X (axis 2)  --> angular z
 *         - Right Stick Y (axis 5)  --> angular y
 *         - Left Trigger (axis 4) and Right Trigger (axis 10) --> angular x control.
 *
 * - In Joint Jog mode:
 *     • A dedicated button reverses the joint velocity direction.
 *     • Six buttons control individual joints:
 *         - D-pad Up (axis 7 > 0.5)    --> Joint 1
 *         - D-pad Down (axis 7 < -0.5) --> Joint 2
 *         - D-pad Left (axis 6 < -0.5) --> Joint 3
 *         - D-pad Right (axis 6 > 0.5) --> Joint 4
 *         - Button X (index 2)         --> Joint 5
 *         - Button Y (index 3)         --> Joint 6
 *     • A separate Home Button (button HOME, index 8) commands all joints to a fixed ("home")
 *       position.
 *
 * - Mode switching:
 *     • Button A (index 0) sets mode to Twist.
 *     • Button B (index 1) sets mode to Joint Jog.
 *
 * - Twist mode frame switching (only applicable in Twist mode):
 *     • Left Stick Click (button index 9) sets the command frame to the Planning frame.
 *     • Right Stick Click (button index 10) sets the command frame to the End-Effector frame.
 *
 * - Button press events are handled on a rising edge basis (transition from 0 to 1).
 *
 * Adjust the button and axis mappings in the configuration section below as required.
 */

#include <signal.h>
#include <stdlib.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

//------------------ Configuration Section ------------------//

// Topics
const std::string JOY_TOPIC   = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string POSE_TOPIC  = "/servo_node/pose_target_cmds";

// Frame IDs
const std::string PLANNING_FRAME_ID = "base_link";
const std::string EE_FRAME_ID       = "link6";

// ROS Queue Size
const size_t ROS_QUEUE_SIZE = 10;

// Default trigger offset mapping: triggers have a default value of 1.0
// and press from 1.0 to -1.0. We compute an effective range [0, 1].
std::map<int, double> AXIS_DEFAULTS = {{4, 1.0}, {10, 1.0}};

// Joystick axis indices (PS4 Controller)
enum Axis
{
  LEFT_STICK_X  = 0,  // used in twist mode for linear y
  LEFT_STICK_Y  = 1,  // used in twist mode for linear z
  RIGHT_STICK_X = 3,  // used in twist mode for angular z
  RIGHT_STICK_Y = 4,  // used in twist mode for angular y
  // Triggers
  LEFT_TRIGGER  = 2,  // used in twist mode for angular x control
  RIGHT_TRIGGER = 5,  // used in twist mode for angular x control
  // D-Pad: these axes act as buttons
  D_PAD_X = 6,
  D_PAD_Y = 7
};

// Joystick button indices (PS4 Controller)
// Note: Adjust indices as necessary.
enum Button
{
  BTN_A            = 0,  // Switch to Twist mode
  BTN_B            = 1,  // Switch to Joint Jog mode
  BTN_X            = 2,  // Joint Jog: Control for Joint 5
  BTN_Y            = 3,  // Joint Jog: Control for Joint 6
  BTN_LEFT_BUMPER  = 4,  // Twist mode: Linear x positive (rising edge)
  BTN_RIGHT_BUMPER = 5,  // Twist mode: Linear x negative (rising edge)
  BTN_CHANGE_VIEW  = 8,  // Joint Jog: Toggle reverse direction
  // Unused button index 7 can be assigned as needed
  BTN_HOME        = 10,  // Joint Jog: Home position for all joints
  BTN_LEFT_STICK  = 11,  // Twist mode: Set command frame to Planning frame
  BTN_RIGHT_STICK = 12   // Twist mode: Set command frame to End-Effector frame
};

// Threshold for considering an axis (e.g., D-pad) pressed as a button.
const double AXIS_THRESHOLD = 0.5;

// Scaling factors for twist mode.
const double LINEAR_SCALE  = 0.02;  // scale for linear velocities
const double ANGULAR_SCALE = 0.05;  // scale for angular velocities

// Joint velocity command value (base value, will be multiplied by reverse factor).
const double JOINT_VEL_CMD = 1.5;

//------------------ End Configuration ------------------//

// Control modes.
enum class ControlMode
{
  TWIST,
  JOINT_JOG,
  POSE
};

class JoystickServo : public rclcpp::Node
{
public:
  JoystickServo()
    : Node("joystick_servo_input")
    , control_mode_(ControlMode::TWIST)
    , reverse_multiplier_(1.0)
    , command_frame_id_(PLANNING_FRAME_ID)
  {
    // Initialize publishers for twist, joint jog and pose commands
    twist_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(POSE_TOPIC, ROS_QUEUE_SIZE);

    // Create a client for switching the servo command type.
    switch_input_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>("servo_node/"
                                                                                   "switch_command_"
                                                                                   "type");

    // Subscribe to joystick messages.
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      JOY_TOPIC,
      ROS_QUEUE_SIZE,
      std::bind(&JoystickServo::joyCallback, this, std::placeholders::_1));

    // Initialize previous button states vector (based on expected size).
    prev_buttons_.resize(12, 0);
    // For d-pad axes, initialize previous values.
    prev_dpad_x_ = 0.0;
    prev_dpad_y_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "JoystickServo node started in TWIST mode.");
  }

private:
  // Callback for incoming Joy messages.
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Process mode switching based on rising edge detection on specific buttons.
    // Mode switching buttons: BTN_A for TWIST, BTN_B for JOINT_JOG.
    if (isRisingEdge(msg->buttons, BTN_A)) switchControlMode(ControlMode::TWIST);
    if (isRisingEdge(msg->buttons, BTN_B)) switchControlMode(ControlMode::JOINT_JOG);

    // Process based on current control mode.
    if (control_mode_ == ControlMode::TWIST) {
      processTwistMode(msg);
    } else if (control_mode_ == ControlMode::JOINT_JOG) {
      processJointJogMode(msg);
    }

    // Send home pose
    if (isRisingEdge(msg->buttons, BTN_HOME)) {
      sendHomePose();
    }

    // Update previous button states and d-pad axes values.
    prev_buttons_ = msg->buttons;
    prev_dpad_x_  = msg->axes[D_PAD_X];
    prev_dpad_y_  = msg->axes[D_PAD_Y];
  }

  void sendHomePose()
  {
    RCLCPP_INFO(this->get_logger(),
                "Home button pressed: Switching to POSE mode and sending home pose.");

    // Switch input type to POSE.
    switchControlMode(ControlMode::POSE);

    // Add a small delay to ensure the mode switch is processed before sending the pose.
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));  // 1.5 seconds delay

    // Publish the home pose.
    geometry_msgs::msg::PoseStamped home_pose_;

    // Initialize fixed home pose.
    home_pose_.header.stamp       = this->now();
    home_pose_.header.frame_id    = PLANNING_FRAME_ID;  // Adjust if needed.
    home_pose_.pose.position.x    = 0.3;                // Set desired x.
    home_pose_.pose.position.y    = 0.0;                // Set desired y.
    home_pose_.pose.position.z    = 0.4;                // Set desired z.
    home_pose_.pose.orientation.w = 1.0;
    home_pose_.pose.orientation.x = 0.0;
    home_pose_.pose.orientation.y = 0.0;
    home_pose_.pose.orientation.z = 0.0;

    pose_pub_->publish(home_pose_);
  }

  // Process Twist Mode commands.
  void processTwistMode(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Create a twist command message.
    auto twist_msg             = std::make_unique<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.stamp    = this->now();
    twist_msg->header.frame_id = command_frame_id_;

    // Map continuous axes:
    // Left Stick: axis 0 -> linear y, axis 1 -> linear z.
    twist_msg->twist.linear.y = msg->axes[LEFT_STICK_X] * LINEAR_SCALE;
    twist_msg->twist.linear.z = msg->axes[LEFT_STICK_Y] * LINEAR_SCALE;

    // Left and Right bumpers control linear x left button gives a fixed negative linear x and
    // button right gives fixed positive linear x.
    twist_msg->twist.linear.x =
      (msg->buttons[BTN_RIGHT_BUMPER] - msg->buttons[BTN_LEFT_BUMPER]) * LINEAR_SCALE;

    // Right Stick: axis 2 -> angular z, axis 5 -> angular y.
    twist_msg->twist.angular.z = msg->axes[RIGHT_STICK_X] * ANGULAR_SCALE;
    twist_msg->twist.angular.y = msg->axes[RIGHT_STICK_Y] * ANGULAR_SCALE;

    // Triggers control angular x.
    // Compute effective value from trigger axes.
    double left_trigger_val  = msg->axes[LEFT_TRIGGER];
    double right_trigger_val = msg->axes[RIGHT_TRIGGER];
    double left_effective = (AXIS_DEFAULTS[LEFT_TRIGGER] - left_trigger_val) / 2.0;  // Range 0 to 1
    double right_effective =
      (AXIS_DEFAULTS[RIGHT_TRIGGER] - right_trigger_val) / 2.0;  // Range 0 to 1
    twist_msg->twist.angular.x = (left_effective - right_effective) * ANGULAR_SCALE;

    // Process command frame switching in Twist mode (rising edge buttons).
    // Left Stick Click for Planning frame, Right Stick Click for EE frame.
    if (isRisingEdge(msg->buttons, BTN_LEFT_STICK)) {
      command_frame_id_ = PLANNING_FRAME_ID;
      RCLCPP_INFO(this->get_logger(), "Twist Mode: Command frame set to Planning frame.");
    }
    if (isRisingEdge(msg->buttons, BTN_RIGHT_STICK)) {
      command_frame_id_ = EE_FRAME_ID;
      RCLCPP_INFO(this->get_logger(), "Twist Mode: Command frame set to End-Effector frame.");
    }

    // Publish the twist command.
    twist_pub_->publish(std::move(twist_msg));
  }

  // Process Joint Jog Mode commands.
  void processJointJogMode(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // In Joint Jog mode, we only act on rising edge button events.
    // Create a JointJog message.
    auto joint_msg          = std::make_unique<control_msgs::msg::JointJog>();
    joint_msg->header.stamp = this->now();
    // For Joint Jog commands, the command frame is always the Planning frame.
    joint_msg->header.frame_id = PLANNING_FRAME_ID;
    joint_msg->joint_names     = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    // Initialize velocities to 0.
    joint_msg->velocities.resize(6, 0.0);

    bool commandIssued = false;

    // Button to reverse joint jogging direction.
    if (isRisingEdge(msg->buttons, BTN_CHANGE_VIEW)) {
      reverse_multiplier_ *= -1;
      RCLCPP_INFO(this->get_logger(),
                  "Joint Jog Mode: Reverse multiplier toggled to %.1f",
                  reverse_multiplier_);
    }

    // Six buttons to control individual joints:
    // D-pad Up for Joint 1.
    if (isAxisButtonRising(msg->axes[D_PAD_Y], prev_dpad_y_, 1.0)) {
      joint_msg->velocities[0] = JOINT_VEL_CMD * reverse_multiplier_;
      commandIssued            = true;
      RCLCPP_INFO(this->get_logger(), "Joint Jog Mode: Triggering Joint 1");
    }
    // D-pad Down for Joint 2.
    if (isAxisButtonRising(msg->axes[D_PAD_Y], prev_dpad_y_, -1.0)) {
      joint_msg->velocities[1] = JOINT_VEL_CMD * reverse_multiplier_;
      commandIssued            = true;
      RCLCPP_INFO(this->get_logger(), "Joint Jog Mode: Triggering Joint 2");
    }
    // D-pad Left for Joint 3.
    if (isAxisButtonRising(msg->axes[D_PAD_X], prev_dpad_x_, -1.0)) {
      joint_msg->velocities[2] = JOINT_VEL_CMD * reverse_multiplier_;
      commandIssued            = true;
      RCLCPP_INFO(this->get_logger(), "Joint Jog Mode: Triggering Joint 3");
    }
    // D-pad Right for Joint 4.
    if (isAxisButtonRising(msg->axes[D_PAD_X], prev_dpad_x_, 1.0)) {
      joint_msg->velocities[3] = JOINT_VEL_CMD * reverse_multiplier_;
      commandIssued            = true;
      RCLCPP_INFO(this->get_logger(), "Joint Jog Mode: Triggering Joint 4");
    }
    // Button X for Joint 5.
    if (isRisingEdge(msg->buttons, BTN_X)) {
      joint_msg->velocities[4] = JOINT_VEL_CMD * reverse_multiplier_;
      commandIssued            = true;
      RCLCPP_INFO(this->get_logger(), "Joint Jog Mode: Triggering Joint 5");
    }
    // Button Y for Joint 6.
    if (isRisingEdge(msg->buttons, BTN_Y)) {
      joint_msg->velocities[5] = JOINT_VEL_CMD * reverse_multiplier_;
      commandIssued            = true;
      RCLCPP_INFO(this->get_logger(), "Joint Jog Mode: Triggering Joint 6");
    }

    // Stop all joints
    // When BTN is pressed, send a command with zero velocities
    if (isRisingEdge(msg->buttons, BTN_B)) {
      RCLCPP_INFO(this->get_logger(), "Joint Jog Mode: Stops");
      joint_msg->displacements.assign(6, 1.0);
      // joint_msg->velocities.assign(6, 0.0);
      joint_pub_->publish(std::move(joint_msg));
      return;  // Skip further processing for this callback.
    }

    // Publish the joint jog command only if a command was issued.
    if (commandIssued) joint_pub_->publish(std::move(joint_msg));
  }

  // Helper function to detect rising edge on a button.
  // Returns true if button at index 'idx' transitioned from 0 to 1.
  bool isRisingEdge(const std::vector<int> & buttons, int idx)
  {
    return (buttons.at(idx) == 1 && prev_buttons_.at(idx) == 0);
  }

  // Helper function for axis-based button detection.
  // Checks if an axis transitioned from a value not equal to target to equal (or close enough) to
  // target.
  bool isAxisButtonRising(double current, double previous, double target)
  {
    if (target > 0)
      return (previous < AXIS_THRESHOLD && current >= AXIS_THRESHOLD);
    else
      return (previous > -AXIS_THRESHOLD && current <= -AXIS_THRESHOLD);
  }

  // Handle control mode switching.
  void switchControlMode(ControlMode new_mode)
  {
    if (control_mode_ == new_mode) return;  // Already in the selected mode.

    control_mode_ = new_mode;
    // Optionally, switch input type via service.
    auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    // Map control mode to servo command type.
    if (control_mode_ == ControlMode::TWIST) {
      request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
      RCLCPP_INFO(this->get_logger(), "Switched to TWIST mode.");
    } else if (control_mode_ == ControlMode::JOINT_JOG) {
      request->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
      RCLCPP_INFO(this->get_logger(), "Switched to JOINT_JOG mode.");
    } else {
      request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
      RCLCPP_INFO(this->get_logger(), "Switched to POSE mode.");
    }
    if (switch_input_client_->wait_for_service(std::chrono::seconds(1))) {
      auto result_future = switch_input_client_->async_send_request(request);
      // BUG: Blocks the code flow.

      // if (result_future.get()->success) {
      //   RCLCPP_INFO(this->get_logger(),
      //               "Successfully switched input type to %s.",
      //               control_mode_ == ControlMode::TWIST ? "TWIST" : "JOINT_JOG");
      // } else {
      //   RCLCPP_WARN(this->get_logger(),
      //               "Could not switch input type to %s.",
      //               control_mode_ == ControlMode::TWIST ? "TWIST" : "JOINT_JOG");
      // }

    } else {
      RCLCPP_WARN(this->get_logger(), "Switch input service not available.");
    }
  }

  // Publishers and client.
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // Previous state for buttons and d-pad axes.
  std::vector<int> prev_buttons_;
  double prev_dpad_x_, prev_dpad_y_;

  // Control mode and parameters.
  ControlMode control_mode_;
  double reverse_multiplier_;
  std::string command_frame_id_;
};

// Signal handler to allow clean shutdown.
void quit(int sig)
{
  (void)sig;
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickServo>();
  signal(SIGINT, quit);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}