#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  void declareParameters();

  double param(const std::string& name) const;

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  declareParameters();
}

void MTCTaskNode::declareParameters()
{
  const auto declare_if_missing = [this](const std::string& name, double default_value) {
    if (!node_->has_parameter(name))
    {
      node_->declare_parameter(name, default_value);
    }
  };

  declare_if_missing("object_x", 0.7);
  declare_if_missing("object_y", 0.4);
  declare_if_missing("object_z", 0.05);
  declare_if_missing("object_height", 0.10);
  declare_if_missing("object_radius", 0.02);

  declare_if_missing("place_x", 0.3);
  declare_if_missing("place_y", -0.3);
  declare_if_missing("place_z", 0.05);
}

double MTCTaskNode::param(const std::string& name) const
{
  return node_->get_parameter(name).as_double();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { param("object_height"), param("object_radius") };

  geometry_msgs::msg::Pose pose;
  pose.position.x = param("object_x");
  pose.position.y = param("object_y");
  pose.position.z = param("object_z");
  pose.orientation.w = 1.0;
  object.pose = pose;

  /*
  moveit_msgs::msg::CollisionObject wall;
  wall.id = "box1";
  wall.header.frame_id = "world";
  wall.primitives.resize(1);
  wall.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  wall.primitives[0].dimensions = { 0.01, 1, 0.2 };

  geometry_msgs::msg::Pose wall_pose;
  wall_pose.orientation.w = 1.0;
  wall_pose.position.x = 0.5;
  wall_pose.position.y = 0.0;
  wall_pose.position.z = 0.0;
  wall.primitive_poses.push_back(wall_pose);
  wall.operation = wall.ADD;

  moveit_msgs::msg::CollisionObject wall1;
  wall1.id = "box2";
  wall1.header.frame_id = "world";
  wall1.primitives.resize(1);
  wall1.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  wall1.primitives[0].dimensions = { 0.01, 1, 0.2 };

  geometry_msgs::msg::Pose wall_pose1;
  wall_pose1.orientation.w = 1.0;
  wall_pose1.position.x = -0.5;
  wall_pose1.position.y = 0.0;
  wall_pose1.position.z = 0.0;
  wall1.primitive_poses.push_back(wall_pose1);
  wall1.operation = wall1.ADD;

  moveit_msgs::msg::CollisionObject wall2;
  wall2.id = "box3";
  wall2.header.frame_id = "world";
  wall2.primitives.resize(1);
  wall2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  wall2.primitives[0].dimensions = { 1, 0.01, 0.2 };

  geometry_msgs::msg::Pose wall_pose2;
  wall_pose2.orientation.w = 1.0;
  wall_pose2.position.x = 0.0;
  wall_pose2.position.y = 0.5;
  wall_pose2.position.z = 0.0;
  wall2.primitive_poses.push_back(wall_pose2);
  wall2.operation = wall2.ADD;

  moveit_msgs::msg::CollisionObject wall3;
  wall3.id = "box4";
  wall3.header.frame_id = "world";
  wall3.primitives.resize(1);
  wall3.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  wall3.primitives[0].dimensions = { 1, 0.01, 0.2 };

  geometry_msgs::msg::Pose wall_pose3;
  wall_pose3.orientation.w = 1.0;
  wall_pose3.position.x = 0.0;
  wall_pose3.position.y = -0.5;
  wall_pose3.position.z = 0.0;
  wall3.primitive_poses.push_back(wall_pose3);
  wall3.operation = wall3.ADD;

  */

  moveit_msgs::msg::CollisionObject floor;
  floor.id = "floor_z_guard";
  floor.header.frame_id = "world";
  floor.primitives.resize(1);
  floor.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  floor.primitives[0].dimensions = { 4.0, 4.0, 0.02 };

  geometry_msgs::msg::Pose floor_pose;
  floor_pose.orientation.w = 1.0;
  floor_pose.position.x = 0.0;
  floor_pose.position.y = 0.0;
  floor_pose.position.z = -0.03;
  floor.primitive_poses.push_back(floor_pose);
  floor.operation = floor.ADD;

  moveit_msgs::msg::ObjectColor floor_color;
  floor_color.id = floor.id;
  floor_color.color.r = 0.0;
  floor_color.color.g = 0.2;
  floor_color.color.b = 1.0;
  floor_color.color.a = 0.25;

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.world.collision_objects = { object, floor };
  planning_scene.object_colors.push_back(floor_color);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyPlanningScene(planning_scene);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  const auto& solutions = task_.solutions();
  if (solutions.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning produced no solutions");
    return;
  }

  const auto best_solution_it = std::min_element(
      solutions.begin(), solutions.end(),
      [](const auto& lhs, const auto& rhs) { return lhs->cost() < rhs->cost(); });
  const auto& best_solution = **best_solution_it;

  RCLCPP_INFO_STREAM(LOGGER, "Executing lowest-cost solution out of "
                                << solutions.size() << " solution(s), cost: " << best_solution.cost());
  task_.introspection().publishSolution(best_solution);

  auto result = task_.execute(best_solution);
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with MoveIt error code: " << result.val);
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur_manipulator";
  const auto& hand_group_name = "gripper";
  const auto& eef_name = "robotiq_2f140";
  const auto& hand_frame = "robotiq_grasping_frame";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(7.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator

  
  {
  auto grasp =
      std::make_unique<mtc::SerialContainer>("pick object");

  task.properties().exposeTo(
      grasp->properties(),
      { "eef", "group", "ik_frame" }
  );

  grasp->properties().configureInitFrom(
      mtc::Stage::PARENT,
      { "eef", "group", "ik_frame" }
  );

  {
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
  stage->properties().set("marker_ns", "approach_object");
  stage->properties().set("link", hand_frame);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.15);

  // robotiq_grasping_frame is the "Grasping frame X" from the Robotiq URDF.
  // Its X axis is the approach axis between the fingers, so approach along X.
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame;
  vec.vector.x = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
}

{
  // Sample grasp pose
  auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  stage->setPreGraspPose("open");
  stage->setObject("object");
  stage->setAngleDelta(M_PI / 12);
  stage->setMonitoredStage(current_state_ptr);  // Hook into current state

  Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  grasp_frame_transform.linear() = q.matrix();
  grasp_frame_transform.translation().x() = 0.05;

  // Compute IK
  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  grasp->insert(std::move(wrapper));
}

{
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
  stage->allowCollisions("object",
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        true);
  grasp->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("close");
  grasp->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage->attachObject("object", hand_frame);
  attach_object_stage = stage.get();
  grasp->insert(std::move(stage));
}

{
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "lift_object");

  // Set upward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
}

  task.add(std::move(grasp));
}

{
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                { hand_group_name, sampling_planner } });
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));
}

{
  auto place = std::make_unique<mtc::SerialContainer>("place object");
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

  {
  // Sample place pose
  auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "place_pose");
  stage->setObject("object");

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "world";
  target_pose_msg.pose.position.x = param("place_x");
  target_pose_msg.pose.position.y = param("place_y");
  target_pose_msg.pose.position.z = param("place_z");
  target_pose_msg.pose.orientation.w = 1.0;
  stage->setPose(target_pose_msg);
  stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

  // Compute IK
  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(2);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame("object");
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  place->insert(std::move(wrapper));
}

{
  auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("open");
  place->insert(std::move(stage));
}

{
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
  stage->allowCollisions("object",
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        false);
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage->detachObject("object", hand_frame);
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "retreat");

  // Set retreat direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame;
  vec.vector.x = -0.5;
  stage->setDirection(vec);
  place->insert(std::move(stage));
}

  task.add(std::move(place));
}

{
  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setGoal("ready");
  task.add(std::move(stage));
}

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
