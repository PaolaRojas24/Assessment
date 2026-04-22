#include "qcar_gazebo/joystick_controller.hpp"

JoystickController::JoystickController(const double timer_period) :
  Node{"joystick_controller"},
  max_steering_angle_{0.0},
  max_velocity_{0.0},
  steering_angle_{0.0},
  velocity_{0.0}
{
  // Declare the used parameters
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);

  // Get parameters on startup
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);

  // Subscription to the 'joy' topic
  subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&JoystickController::listener_callback, this, std::placeholders::_1));

  // Publishers
  user_command_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/qcar_sim/user_command", 1);

  // Timer to periodically publish the desired angle and velocity
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&JoystickController::timer_callback, this));
}

void JoystickController::listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[4] == 1) {
    // Set the desired angle and desired velocity based on the joystick's axis
    steering_angle_ = std::clamp(msg->axes[0] * max_steering_angle_, -0.3, 0.3);
    if(msg->buttons[5] == 1){
      velocity_ = 0.15 * max_velocity_;
    }else{
      velocity_ = 0.5 * abs(msg->axes[5] - 1.0) * max_velocity_;
    }
    if (msg->buttons[0] == 1) {
      velocity_ = -velocity_;
    }
  }
  
}

void JoystickController::timer_callback()
{
  // Publish the desired angle and velocity as a Vector3Stamped message
  auto user_command_msg{std::make_shared<geometry_msgs::msg::Vector3Stamped>()};
  user_command_msg->header.stamp = get_clock()->now();
  user_command_msg->vector.y = steering_angle_;
  user_command_msg->vector.x = velocity_;
  user_command_publisher_->publish(*user_command_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickController>());
  rclcpp::shutdown();
  return 0;
}