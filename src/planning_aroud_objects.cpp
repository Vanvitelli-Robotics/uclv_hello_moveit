#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  // This way we don't have to spin
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Construct and initialize MoveItVisualTools
  // This is just to visualize thinks in rviz
  // If you want to know more:
  // https://moveit.picknik.ai/main/doc/tutorials/visualizing_in_rviz/visualizing_in_rviz.html
  // NOTE: to comple this you need to add the dependency moveit_visual_tools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // ******************************************************
  // Here we are just defining functions for visualization
  // It is really not needed for the planning point of view
  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("panda_arm")](
          auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };
  // ***********************************************************

  // *********************************************
  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.y = 0.8;
    msg.orientation.w = 0.6;
    msg.position.x = 0.1;
    msg.position.y = 0.4;
    msg.position.z = 0.4;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  // *********************************************

  // **********************************************
  // Create collision object for the robot to avoid
  // Here we are creatng a moveit_msgs::msg::CollisionObject
  // it is a ROS message used to define geometry and pose of an object in the scene
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;  // <- reference frame for the object pose
    collision_object.id = "box1";                 // <- name of the object (id)

    // This will be a subfield of the message defining a geometry primitive
    shape_msgs::msg::SolidPrimitive primitive;
    // Define the size of the box in meters
    primitive.type = primitive.BOX;  // <- primitive type, a box in this case
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;  // size of the box...
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    // Add the primitive and the pose to the collision_object message
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    // The operation is ADD (we want to add the object to the scene)
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Now we have just defined the object message
  // In order to create the object we need to apply it to the scene
  // To interact with the planning scene we can use the utility object
  // moveit::planning_interface::PlanningSceneInterface

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);  // <- here the object is creeted

  // Now let's create a plan to that target pose

  //*************************************************************
  // This block is just to visualize stuff in RVIZ
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  //*************************************************************

  //*************************************************************
  // Here we plan a trakectory (as in the previous tutorial)
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  //*************************************************************

  // Execute the plan
  if (success)
  {
    //*************************************************************
    // This block is just to visualize stuff in RVIZ
    draw_trajectory_tool_path(plan.trajectory_); // WARNING! you need the underscore_ 'trajectory_', it is wrong in the tutorial
    moveit_visual_tools.trigger();
    prompt("Press 'next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    //*************************************************************

    // Finally execute the plan!
    move_group_interface.execute(plan);
  }
  else
  {
    //*************************************************************
    // This block is just to visualize stuff in RVIZ
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    //*************************************************************
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  // We need to join the spinner thread!
  spinner.join();
  return 0;
}
