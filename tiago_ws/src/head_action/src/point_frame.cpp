#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/point_head.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class PointHeadActionServer : public rclcpp::Node
{
public:
  using PointHead = control_msgs::action::PointHead;
  using GoalHandlePointHead = rclcpp_action::ServerGoalHandle<PointHead>;

  PointHeadActionServer()
  : Node("point_head_action_server")
  {
    this->declare_parameter<double>("success_angle_threshold", 0.01);
    this->declare_parameter<std::string>("pan_link", "head_2_link");
    this->declare_parameter<std::string>("default_pointing_frame", "stereo_optical_frame");

    action_server_ = rclcpp_action::create_server<PointHead>(
      this,
      "head_controller/point_head",
      std::bind(&PointHeadActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PointHeadActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PointHeadActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<PointHead>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PointHead::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePointHead> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePointHead> goal_handle)
  {
    std::thread{std::bind(&PointHeadActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }


  void execute(const std::shared_ptr<GoalHandlePointHead> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
  
    auto goal = goal_handle->get_goal();
    auto target = goal->target.point;
  
    // Basic logic to convert target point into pan/tilt angles (VERY simplified)
    double pan = atan2(target.y, target.x);
    double tilt = -atan2(target.z, sqrt(target.x * target.x + target.y * target.y));
  
    RCLCPP_INFO(this->get_logger(), "Target XYZ: %.2f, %.2f, %.2f -> Pan: %.2f, Tilt: %.2f",
                target.x, target.y, target.z, pan, tilt);
  
    // Create trajectory message
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    traj_msg.joint_names = {"head_1_joint", "head_2_joint"};
  
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {pan, tilt};
    point.time_from_start = rclcpp::Duration::from_seconds(1.5);
    traj_msg.points.push_back(point);
  
    // Publisher (create inside the class and reuse ideally)
    auto pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/head_controller/joint_trajectory", 10);
    pub->publish(traj_msg);
  
    rclcpp::sleep_for(std::chrono::seconds(2));  // wait to let the motion happen
  
    goal_handle->succeed(std::make_shared<PointHead::Result>());
    RCLCPP_INFO(this->get_logger(), "Head movement completed.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointHeadActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
