#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/cost_terms.h>
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
#include <unistd.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  bool doTask();

  bool setupPlanningScene();

  void clearTaskVisualization();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  void declareParameters();
  moveit_msgs::msg::CollisionObject makeObject(double x, double y, double z) const;
  bool applyTouchCollisionAcm();

  std::string objectId() const;
  std::string plannerId() const;
  double param(const std::string& name) const;
  static std::vector<std::string> robotiqTouchLinks();

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node_" + std::to_string(getpid()), options) }
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

  if (!node_->has_parameter("object_id"))
  {
    node_->declare_parameter<std::string>("object_id", "red_block");
  }
  if (!node_->has_parameter("planner_id"))
  {
    node_->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");
  }

  declare_if_missing("object_x", -0.3);
  declare_if_missing("object_y", 0.3);
  declare_if_missing("object_z", 0.05);
  declare_if_missing("object_size_x", 0.10);
  declare_if_missing("object_size_y", 0.0515);
  declare_if_missing("object_size_z", 0.10);

  declare_if_missing("place_x", 0.3);
  declare_if_missing("place_y", -0.3);
  declare_if_missing("place_z", 0.05);

  declare_if_missing("max_solutions", 3.0);
  declare_if_missing("move_to_pick_timeout", 5.0);
  declare_if_missing("move_to_pick_max_path_length", 8.0);
  declare_if_missing("move_to_place_timeout", 6.0);
  declare_if_missing("return_home_timeout", 1.0);
  declare_if_missing("gripper_close_min", 0.02);
  declare_if_missing("gripper_close_max", 0.45);
  declare_if_missing("gripper_close_step", 0.08);
}

double MTCTaskNode::param(const std::string& name) const
{
  return node_->get_parameter(name).as_double();
}

std::string MTCTaskNode::objectId() const
{
  return node_->get_parameter("object_id").as_string();
}

std::string MTCTaskNode::plannerId() const
{
  return node_->get_parameter("planner_id").as_string();
}

std::vector<std::string> MTCTaskNode::robotiqTouchLinks()
{
  return {
    "robotiq_base_link",
    "robotiq_left_inner_finger",
    "robotiq_left_inner_finger_pad",
    "robotiq_left_inner_knuckle",
    "robotiq_left_outer_finger",
    "robotiq_left_outer_knuckle",
    "robotiq_right_inner_finger",
    "robotiq_right_inner_finger_pad",
    "robotiq_right_inner_knuckle",
    "robotiq_right_outer_finger",
    "robotiq_right_outer_knuckle",
  };
}

moveit_msgs::msg::CollisionObject MTCTaskNode::makeObject(double x, double y, double z) const
{
  moveit_msgs::msg::CollisionObject object;
  object.id = objectId();
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = {
    param("object_size_x"), param("object_size_y"), param("object_size_z")
  };

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  return object;
}

void addAcmName(moveit_msgs::msg::AllowedCollisionMatrix& matrix, const std::string& name)
{
  if (std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name) != matrix.entry_names.end())
  {
    return;
  }

  matrix.entry_names.push_back(name);
  matrix.entry_values.emplace_back();
}

void resizeAcm(moveit_msgs::msg::AllowedCollisionMatrix& matrix)
{
  while (matrix.entry_values.size() < matrix.entry_names.size())
  {
    matrix.entry_values.emplace_back();
  }

  for (auto& entry : matrix.entry_values)
  {
    entry.enabled.resize(matrix.entry_names.size(), false);
  }
}

void setAcmPair(
    moveit_msgs::msg::AllowedCollisionMatrix& matrix,
    const std::string& first,
    const std::string& second,
    bool allowed)
{
  const auto first_it = std::find(matrix.entry_names.begin(), matrix.entry_names.end(), first);
  const auto second_it = std::find(matrix.entry_names.begin(), matrix.entry_names.end(), second);
  if (first_it == matrix.entry_names.end() || second_it == matrix.entry_names.end())
  {
    return;
  }

  const auto first_index = static_cast<size_t>(std::distance(matrix.entry_names.begin(), first_it));
  const auto second_index = static_cast<size_t>(std::distance(matrix.entry_names.begin(), second_it));
  matrix.entry_values[first_index].enabled[second_index] = allowed;
  matrix.entry_values[second_index].enabled[first_index] = allowed;
}

void setAcmDefault(moveit_msgs::msg::AllowedCollisionMatrix& matrix, const std::string& name, bool allowed)
{
  const auto name_it = std::find(
      matrix.default_entry_names.begin(), matrix.default_entry_names.end(), name);
  if (name_it == matrix.default_entry_names.end())
  {
    matrix.default_entry_names.push_back(name);
    matrix.default_entry_values.push_back(allowed);
    return;
  }

  const auto index = static_cast<size_t>(std::distance(matrix.default_entry_names.begin(), name_it));
  matrix.default_entry_values[index] = allowed;
}

bool MTCTaskNode::applyTouchCollisionAcm()
{
  constexpr auto planning_scene_service_timeout = std::chrono::seconds(10);
  auto service_node = std::make_shared<rclcpp::Node>(
      "mtc_touch_acm_node_" + std::to_string(getpid()));
  auto get_scene_client =
      service_node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  auto apply_scene_client =
      service_node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

  if (!get_scene_client->wait_for_service(planning_scene_service_timeout) ||
      !apply_scene_client->wait_for_service(planning_scene_service_timeout))
  {
    RCLCPP_ERROR(LOGGER, "MoveIt planning scene services are not available for touch ACM update");
    return false;
  }

  auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  get_request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto get_future = get_scene_client->async_send_request(get_request);
  if (rclcpp::spin_until_future_complete(service_node, get_future, planning_scene_service_timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Failed to fetch MoveIt ACM before MTC execution");
    return false;
  }

  auto matrix = get_future.get()->scene.allowed_collision_matrix;
  if (matrix.entry_names.empty())
  {
    RCLCPP_ERROR(LOGGER, "MoveIt returned an empty ACM before MTC execution");
    return false;
  }

  const auto object_id = objectId();
  addAcmName(matrix, object_id);
  setAcmDefault(matrix, object_id, true);
  for (const auto& link : robotiqTouchLinks())
  {
    addAcmName(matrix, link);
  }
  resizeAcm(matrix);

  for (const auto& link : robotiqTouchLinks())
  {
    setAcmPair(matrix, object_id, link, true);
  }

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.allowed_collision_matrix = matrix;

  auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  apply_request->scene = planning_scene;
  auto apply_future = apply_scene_client->async_send_request(apply_request);
  if (rclcpp::spin_until_future_complete(service_node, apply_future, planning_scene_service_timeout) !=
      rclcpp::FutureReturnCode::SUCCESS ||
      !apply_future.get()->success)
  {
    RCLCPP_ERROR(LOGGER, "Failed to apply touch ACM before MTC execution");
    return false;
  }

  RCLCPP_INFO(
      LOGGER,
      "Allowed %s to touch robot links in MoveIt ACM",
      object_id.c_str());
  return true;
}

bool MTCTaskNode::setupPlanningScene()
{
  constexpr auto planning_scene_service_timeout = std::chrono::seconds(10);
  const auto object_id = objectId();
  RCLCPP_INFO_STREAM(LOGGER, "Preparing PlanningScene for " << object_id);
  auto service_node = std::make_shared<rclcpp::Node>(
      "mtc_scene_setup_node_" + std::to_string(getpid()));
  auto get_scene_client =
      service_node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  auto apply_scene_client =
      service_node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

  if (!get_scene_client->wait_for_service(planning_scene_service_timeout) ||
      !apply_scene_client->wait_for_service(planning_scene_service_timeout))
  {
    RCLCPP_ERROR(LOGGER, "MoveIt planning scene services are unavailable during scene setup");
    return false;
  }

  auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  get_request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
  auto get_future = get_scene_client->async_send_request(get_request);
  if (rclcpp::spin_until_future_complete(service_node, get_future, planning_scene_service_timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Timed out fetching the PlanningScene during scene setup");
    return false;
  }

  const auto& current_scene = get_future.get()->scene;
  const auto is_attached = [&current_scene](const std::string& id) {
    return std::any_of(
        current_scene.robot_state.attached_collision_objects.begin(),
        current_scene.robot_state.attached_collision_objects.end(),
        [&id](const auto& attached) { return attached.object.id == id; });
  };
  const auto is_world_object = [&current_scene](const std::string& id) {
    return std::any_of(
        current_scene.world.collision_objects.begin(),
        current_scene.world.collision_objects.end(),
        [&id](const auto& object) { return object.id == id; });
  };

  auto object = makeObject(param("object_x"), param("object_y"), param("object_z"));

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
  planning_scene.robot_state.is_diff = true;

  for (const auto& attached_id : { std::string("object"), object_id })
  {
    if (!is_attached(attached_id))
    {
      continue;
    }
    moveit_msgs::msg::AttachedCollisionObject detach;
    detach.object.id = attached_id;
    detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene.robot_state.attached_collision_objects.push_back(detach);
  }

  if (is_world_object("object"))
  {
    moveit_msgs::msg::CollisionObject remove_legacy_object;
    remove_legacy_object.id = "object";
    remove_legacy_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene.world.collision_objects.push_back(remove_legacy_object);
  }

  planning_scene.world.collision_objects.push_back(object);
  planning_scene.world.collision_objects.push_back(floor);
  planning_scene.object_colors.push_back(floor_color);

  auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  apply_request->scene = planning_scene;
  auto apply_future = apply_scene_client->async_send_request(apply_request);
  if (rclcpp::spin_until_future_complete(service_node, apply_future, planning_scene_service_timeout) !=
          rclcpp::FutureReturnCode::SUCCESS ||
      !apply_future.get()->success)
  {
    RCLCPP_ERROR(LOGGER, "Failed to apply the PlanningScene during scene setup");
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!applyTouchCollisionAcm())
  {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  RCLCPP_INFO_STREAM(LOGGER, "PlanningScene setup complete for " << object_id);
  return true;
}

bool MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  const auto max_solutions = static_cast<unsigned int>(param("max_solutions"));
  RCLCPP_INFO_STREAM(LOGGER, "Planning until " << max_solutions << " task solution(s) are available");
  if (!task_.plan(max_solutions))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return false;
  }

  const auto& solutions = task_.solutions();
  if (solutions.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning produced no solutions");
    return false;
  }
  if (solutions.size() < max_solutions)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning produced only " << solutions.size() << " of "
                                                                << max_solutions
                                                                << " required solutions; execution skipped");
    return false;
  }

  const auto best_solution_it = std::min_element(
      solutions.begin(), solutions.end(),
      [](const auto& lhs, const auto& rhs) { return lhs->cost() < rhs->cost(); });
  const auto& best_solution = **best_solution_it;

  RCLCPP_INFO_STREAM(LOGGER, "Executing lowest-cost solution out of "
                                << solutions.size() << " planned solution(s), cost: "
                                << best_solution.cost());

  auto result = task_.execute(best_solution);
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with MoveIt error code: " << result.val);
    return false;
  }

  return true;
}

void MTCTaskNode::clearTaskVisualization()
{
  task_.reset();
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
  const auto object_id = objectId();
  const auto gripper_collision_links = robotiqTouchLinks();
  const std::vector<std::string> forearm_collision_links = { "forearm_link" };

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef_name);
  task.setProperty("ik_frame", hand_frame);

  auto sequence = std::make_unique<mtc::SerialContainer>("pick-place sequence");
  task.properties().exposeTo(sequence->properties(), { "group", "eef", "ik_frame" });
  sequence->properties().configureInitFrom(
      mtc::Stage::PARENT, { "group", "eef", "ik_frame" });

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  sequence->insert(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  sampling_planner->setPlannerId(plannerId());
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  RCLCPP_INFO_STREAM(LOGGER, "Using OMPL planner: " << plannerId());
  auto gripper_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper forearm self-collision");
    stage->allowCollisions(forearm_collision_links, gripper_collision_links, true);
    sequence->insert(std::move(stage));
  }

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", gripper_interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  sequence->insert(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move above pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(param("move_to_pick_timeout"));
  const double move_to_pick_max_path_length = param("move_to_pick_max_path_length");
  stage_move_to_pick->setCostTerm(
      [move_to_pick_max_path_length](const mtc::SubTrajectory& solution, std::string& comment) {
        mtc::cost::PathLength path_length_cost;
        const double path_length = path_length_cost(solution, comment);
        if (move_to_pick_max_path_length >= 0.0 && path_length > move_to_pick_max_path_length)
        {
          std::ostringstream stream;
          stream << "move above pick path too long: " << path_length << " > "
                 << move_to_pick_max_path_length;
          comment = stream.str();
          return std::numeric_limits<double>::infinity();
        }
        return path_length;
      });
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  sequence->insert(std::move(stage_move_to_pick));

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

  // Descend in the world frame so the approach remains vertical regardless of
  // which axis-aligned yaw candidate is selected for the gripper.
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = -1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
}

{
  // Sample grasp pose
  auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  stage->setPreGraspPose("open");
  stage->setObject(object_id);
  // Rectangular objects only permit grasps normal to a box face. Sampling at
  // quarter turns excludes diagonal 30/60-degree wrist orientations.
  stage->setAngleDelta(M_PI / 2.0);
  stage->setMonitoredStage(current_state_ptr);  // Hook into current state

  Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
  // The Robotiq grasp frame uses X as its tool/approach axis. This virtual-frame
  // rotation makes hand-frame X point along world -Z at every sampled yaw.
  Eigen::Quaterniond q(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()));
  grasp_frame_transform.linear() = q.matrix();
  // Target the top face instead of driving the grasp frame into the table.
  grasp_frame_transform.translation().x() = 0.5 * param("object_size_z");

  // Compute IK
  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(4);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  grasp->insert(std::move(wrapper));
}

{
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow pick object robot contact");
  stage->allowCollisions(object_id, true);
  grasp->insert(std::move(stage));
}

{
  auto close_hand = std::make_unique<mtc::Fallbacks>("close hand");
  const double close_min = std::max(0.0, param("gripper_close_min"));
  const double close_max = std::max(close_min, param("gripper_close_max"));
  const double close_step = std::max(0.001, param("gripper_close_step"));

  for (double position = close_max; position >= close_min - 1e-9; position -= close_step)
  {
    std::ostringstream stage_name;
    stage_name << "close hand " << position;

    auto stage = std::make_unique<mtc::stages::MoveTo>(stage_name.str(), gripper_interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal(std::map<std::string, double>{ { "robotiq_finger_joint", position } });
    close_hand->insert(std::move(stage));
  }

  grasp->insert(std::move(close_hand));
}

{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage->attachObject(object_id, hand_frame);
  attach_object_stage = stage.get();
  grasp->insert(std::move(stage));
}

{
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.18);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "lift_object");

  // Set upward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
}

  sequence->insert(std::move(grasp));
}

{
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move above place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_place->setTimeout(param("move_to_place_timeout"));
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  sequence->insert(std::move(stage_move_to_place));
}

{
  auto place = std::make_unique<mtc::SerialContainer>("place object");
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });

  {
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
  stage->properties().set("marker_ns", "lower_object");
  stage->properties().set("link", hand_frame);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.15);

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = -1.0;
  stage->setDirection(vec);
  place->insert(std::move(stage));
}

  {
  // Sample place pose
  auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "place_pose");
  stage->setObject(object_id);

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
  wrapper->setMaxIKSolutions(1);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(object_id);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  place->insert(std::move(wrapper));
}

{
  auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", gripper_interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("open");
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage->detachObject(object_id, hand_frame);
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.18);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "retreat");

  // Rise vertically before planning the return-home motion.
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  place->insert(std::move(stage));
}

  sequence->insert(std::move(place));
}

{
  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
  stage->setGroup(arm_group_name);
  stage->properties().set("timeout", param("return_home_timeout"));
  stage->setCostTerm(std::make_unique<mtc::cost::PathLength>());
  stage->setGoal("ready");
  sequence->insert(std::move(stage));
}

  task.add(std::move(sequence));

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

  const bool scene_ready = mtc_task_node->setupPlanningScene();
  const bool success = scene_ready && mtc_task_node->doTask();
  mtc_task_node->clearTaskVisualization();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
