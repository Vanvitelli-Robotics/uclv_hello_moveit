#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node =
      std::make_shared<rclcpp::Node>("hello_moveit",
                                     // this second argument in Node constructor
                                     //  is needed for MoveIt because of how MoveIt uses ROS Parameters.
                                     //  Google it if you are curious
                                     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger (just for the output... not really needed....)
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  // This object will be used to interact with move_group, 
  // which allows us to plan and execute trajectories.
  // The first parameter is the node, the second parameter is the group to use
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // Set a target Pose
  // Here we use 'lambda functions' for 
  // constructing the message type target_pose.
  // This is a pattern you’ll find in modern C++ codebases 
  // that enables writing in a more declarative style. 
  // For more information about this pattern:
  // https://en.cppreference.com/w/cpp/language/lambda
  // https://www.cppstories.com/2016/11/iife-for-complex-initialization/
  // https://www.cppstories.com/2016/12/please-declare-your-variables-as-const/
  // At the end we just build a geometry_msgs::msg::Pose object!
  // You could do it without lambda functions!
  // The only real difference is that, this way, you can declare it const
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  // Here we set our target pose.
  // Note that only the target pose is set (via setPoseTarget). 
  // The starting pose is implicitly the position published by the joint state publisher,
  // which could be changed using the MoveGroupInterface::setStartState* 
  // family of functions (but is not in this tutorial).
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  // Here we call the plan function move_group_interface.plan(msg)
  // We also use the lambda function pattern just because, this way the
  // variavle success and plan can be const.
  // It is not really needed, you could not use the lambda expression!
  auto const [success, plan] = [&move_group_interface] {
    // The plan method takes a MoveGroupInterface::Plan msg as reference input,
    // The msg will contain the planned path (it is really an output!)
    // The output of move_group_interface.plan(...) is a moveit::core::MoveItErrorCode
    // The error code is converted (static_cast) in a bool (false=error).
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    // This std::make_pair is just to output two values from the lambda expression.
    return std::make_pair(ok, msg);
  }();

  //Here the plan has been done!!!!
  // (note: we need to check the success variable to check if the plan succeded)

  // Execute the plan
  if (success)
  {
    // I the plan succedded we can execute it
    move_group_interface.execute(plan);
  }
  else
  {
    // else error
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
