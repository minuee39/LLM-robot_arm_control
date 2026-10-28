#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
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
#include <mtc_tutorial/pallet_obstacle_audit.hpp>
#include <mtc_tutorial/pallet_release.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

void logTrajectorySegments(
    const mtc::SolutionBase& solution, size_t& segment_index, double& total_duration)
{
  if (const auto* sequence = dynamic_cast<const mtc::SolutionSequence*>(&solution))
  {
    for (const auto* child : sequence->solutions())
    {
      logTrajectorySegments(*child, segment_index, total_duration);
    }
    return;
  }

  if (const auto* wrapped = dynamic_cast<const mtc::WrappedSolution*>(&solution))
  {
    logTrajectorySegments(*wrapped->wrapped(), segment_index, total_duration);
    return;
  }

  const auto* sub_trajectory = dynamic_cast<const mtc::SubTrajectory*>(&solution);
  if (!sub_trajectory || !sub_trajectory->trajectory())
  {
    return;
  }

  const auto& trajectory = *sub_trajectory->trajectory();
  const double duration = trajectory.getDuration();
  if (duration <= 0.0 || trajectory.getWayPointCount() == 0)
  {
    return;
  }

  ++segment_index;
  total_duration += duration;
  const std::string stage_name = solution.creator() ? solution.creator()->name() : "unknown";
  RCLCPP_INFO(
      LOGGER, "Trajectory segment %zu: '%s', %.3f s, %zu waypoint(s)", segment_index,
      stage_name.c_str(), duration, trajectory.getWayPointCount());
}

bool auditPalletObstacleSolution(
    const mtc::SolutionBase& solution, const std::vector<std::string>& obstacle_ids,
    const std::string& target_id, size_t& checked_scenes)
{
  if (const auto* sequence = dynamic_cast<const mtc::SolutionSequence*>(&solution))
  {
    for (const auto* child : sequence->solutions())
    {
      if (!auditPalletObstacleSolution(*child, obstacle_ids, target_id, checked_scenes))
        return false;
    }
    return true;
  }
  if (const auto* wrapped = dynamic_cast<const mtc::WrappedSolution*>(&solution))
    return auditPalletObstacleSolution(*wrapped->wrapped(), obstacle_ids, target_id, checked_scenes);
  const auto* trajectory = dynamic_cast<const mtc::SubTrajectory*>(&solution);
  if (!trajectory || !trajectory->trajectory() || !solution.creator())
    return true;
  const auto stage = solution.creator()->name();
  if (stage != "lift object" && stage != "move above place" && stage != "lower object")
    return true;
  for (const auto* state : { solution.start(), solution.end() })
  {
    if (!state || !state->scene())
      return false;
    const auto& scene = state->scene();
    const auto& matrix = scene->getAllowedCollisionMatrix();
    auto names = scene->getRobotModel()->getLinkModelNames();
    names.push_back(target_id);
    for (const auto& id : obstacle_ids)
    {
      if (!scene->getWorld()->hasObject(id))
      {
        RCLCPP_ERROR(LOGGER, "Pallet obstacle %s missing from selected stage %s", id.c_str(), stage.c_str());
        return false;
      }
      for (const auto& name : names)
      {
        if (!palletObstaclePairForbidden(matrix, id, name))
        {
          RCLCPP_ERROR(LOGGER, "Pallet obstacle contact allowed in stage %s: %s/%s",
                       stage.c_str(), id.c_str(), name.c_str());
          return false;
        }
      }
    }
    ++checked_scenes;
    RCLCPP_INFO(LOGGER, "PALLET_OBSTACLE_SOLUTION_AUDIT stage=%s scene=%zu effective_pairs_forbidden=true",
                stage.c_str(), checked_scenes);
  }
  return true;
}

template <typename ServiceT>
typename ServiceT::Response::SharedPtr callPlanningSceneService(
    const rclcpp::Node::SharedPtr& node,
    const typename rclcpp::Client<ServiceT>::SharedPtr& client,
    const typename ServiceT::Request::SharedPtr& request,
    const std::string& operation,
    size_t max_attempts = 3)
{
  constexpr auto response_timeout = std::chrono::seconds(4);
  for (size_t attempt = 1; attempt <= max_attempts; ++attempt)
  {
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, response_timeout) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (attempt > 1)
      {
        RCLCPP_INFO(
            LOGGER, "%s succeeded on attempt %zu/%zu", operation.c_str(), attempt, max_attempts);
      }
      return future.get();
    }

    RCLCPP_WARN(
        LOGGER,
        "%s timed out on attempt %zu/%zu; retrying",
        operation.c_str(),
        attempt,
        max_attempts);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return nullptr;
}

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  bool doTask();

  bool setupPlanningScene();

  void clearTaskVisualization();

  bool recoveryOnly() const;

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task createRecoveryTask();
  void declareParameters();
  moveit_msgs::msg::CollisionObject makeObject(double x, double y, double z) const;
  bool applyTouchCollisionAcm();
  bool logRecoverySceneDiagnostics();
  moveit::core::MoveItErrorCode executeBlended(const mtc::SolutionBase& solution);
  void publishCollisionState(const mtc::SolutionBase& solution);

  std::string objectId() const;
  std::string plannerId() const;
  std::string stringParam(const std::string& name) const;
  std::vector<std::string> stringArrayParam(const std::string& name) const;
  double param(const std::string& name) const;

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr collision_state_publisher_;
  bool collision_state_published_{ false };
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node_" + std::to_string(getpid()), options) }
{
  declareParameters();
  collision_state_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>(
      "/mtc_collision_state", rclcpp::QoS(1).transient_local().reliable());
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
  if (!node_->has_parameter("recovery_only"))
  {
    node_->declare_parameter("recovery_only", false);
  }
  if (!node_->has_parameter("recovery_candidate_id"))
  {
    node_->declare_parameter<std::string>("recovery_candidate_id", "vertical");
  }
  if (!node_->has_parameter("planning_only"))
  {
    node_->declare_parameter("planning_only", false);
  }
  if (!node_->has_parameter("cartesian_transfer"))
  {
    node_->declare_parameter("cartesian_transfer", false);
  }
  if (!node_->has_parameter("upright_transfer"))
  {
    node_->declare_parameter("upright_transfer", false);
  }
  const auto declare_string_if_missing = [this](
      const std::string& name, const std::string& default_value) {
    if (!node_->has_parameter(name))
    {
      node_->declare_parameter<std::string>(name, default_value);
    }
  };
  const auto declare_string_array_if_missing = [this](
      const std::string& name, const std::vector<std::string>& default_value) {
    if (!node_->has_parameter(name))
    {
      node_->declare_parameter<std::vector<std::string>>(name, default_value);
    }
  };

  declare_string_if_missing("execution_mode", "standard");
  declare_string_if_missing("planning_frame", "world");
  declare_string_if_missing("arm_group_name", "ur_manipulator");
  declare_string_if_missing("hand_group_name", "gripper");
  declare_string_if_missing("eef_name", "robotiq_2f140");
  declare_string_if_missing("hand_frame", "robotiq_grasping_frame");
  declare_string_if_missing("gripper_joint_name", "robotiq_finger_joint");
  declare_string_if_missing("open_pose_name", "open");
  declare_if_missing("release_open_position", -1.0);
  declare_string_if_missing("home_pose_name", "ready");
  declare_string_if_missing("arm_controller_name", "ur_manipulator_controller");
  declare_string_if_missing("gripper_controller_name", "gripper_controller");
  declare_string_array_if_missing(
      "touch_links",
      {
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
      });
  declare_string_array_if_missing("forearm_collision_links", { "forearm_link" });
  declare_string_array_if_missing("pallet_obstacle_ids", {});
  if (!node_->has_parameter("pallet_obstacle_spheres"))
  {
    node_->declare_parameter<std::vector<double>>("pallet_obstacle_spheres", std::vector<double>{});
  }

  declare_if_missing("object_x", -0.3);
  declare_if_missing("object_y", 0.3);
  declare_if_missing("object_z", 0.05);
  declare_if_missing("object_size_x", 0.10);
  declare_if_missing("object_size_y", 0.0515);
  declare_if_missing("object_size_z", 0.10);
  declare_if_missing("table_size_x", 0.0);
  declare_if_missing("table_size_y", 0.0);
  declare_if_missing("table_height", 0.0);
  declare_if_missing("table_center_x", 0.0);
  declare_if_missing("table_center_y", 0.0);
  declare_if_missing("table_top_z", 0.0);

  declare_if_missing("place_x", 0.3);
  declare_if_missing("place_y", -0.3);
  declare_if_missing("place_z", 0.05);
  declare_if_missing("place_yaw", 0.0);

  declare_if_missing("max_solutions", 1.0);
  declare_if_missing("max_solution_cost", 60.0);
  declare_if_missing("task_timeout", -1.0);
  declare_if_missing("place_max_ik_solutions", 1.0);
  declare_if_missing("place_ik_min_solution_distance", 1.0);
  declare_if_missing("move_to_pick_timeout", 5.0);
  // Segment PathLength is accumulated joint-space motion, not Cartesian
  // distance or total task cost. Keep it observable but unbounded by default.
  declare_if_missing("move_to_pick_max_path_length", -1.0);
  declare_if_missing("move_to_place_timeout", 4.0);
  declare_if_missing("move_to_place_max_path_length", -1.0);
  declare_if_missing("transfer_min_clearance", -1.0);
  declare_if_missing("transfer_clearance_check_step", 0.02);
  declare_if_missing("transfer_orientation_tolerance", M_PI / 36.0);
  declare_if_missing("return_home_timeout", 1.0);
  declare_if_missing("return_home_max_path_length", -1.0);
  declare_if_missing("arm_velocity_scaling", 0.5);
  declare_if_missing("arm_acceleration_scaling", 0.5);
  declare_if_missing("cartesian_velocity_scaling", 0.25);
  declare_if_missing("cartesian_acceleration_scaling", 0.25);
  declare_if_missing("gripper_velocity_scaling", 0.5);
  declare_if_missing("gripper_acceleration_scaling", 0.5);
  declare_if_missing("gripper_close_min", 0.02);
  declare_if_missing("gripper_close_max", 0.50);
  declare_if_missing("gripper_close_step", 0.08);
  declare_if_missing("recovery_retreat_distance", 0.15);
  declare_if_missing("recovery_probe_distance", 0.005);
  declare_if_missing("recovery_retreat_timeout", 3.0);
  declare_if_missing("recovery_home_timeout", 5.0);
  declare_if_missing("recovery_direction_x", 0.0);
  declare_if_missing("recovery_direction_y", 0.0);
  declare_if_missing("recovery_direction_z", 1.0);
  declare_if_missing("grasp_transform_roll", 0.0);
  declare_if_missing("grasp_transform_pitch", -M_PI / 2.0);
  declare_if_missing("grasp_transform_yaw", 0.0);
  declare_if_missing("grasp_angle_delta", M_PI / 2.0);
  declare_if_missing("grasp_offset_x", 0.0);
  declare_if_missing("grasp_offset_y", 0.0);
  declare_if_missing("grasp_offset_z", 0.0);
  declare_if_missing("grasp_object_height_scale_x", 0.5);
  declare_if_missing("grasp_object_height_scale_y", 0.0);
  declare_if_missing("grasp_object_height_scale_z", 0.0);
  declare_if_missing("approach_min_distance", 0.10);
  declare_if_missing("approach_max_distance", 0.15);
  declare_if_missing("lift_min_distance", 0.10);
  declare_if_missing("lift_max_distance", 0.18);
  declare_if_missing("lower_min_distance", 0.10);
  declare_if_missing("lower_max_distance", 0.15);
  declare_if_missing("retreat_min_distance", 0.10);
  declare_if_missing("retreat_max_distance", 0.18);
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

std::string MTCTaskNode::stringParam(const std::string& name) const
{
  return node_->get_parameter(name).as_string();
}

std::vector<std::string> MTCTaskNode::stringArrayParam(const std::string& name) const
{
  auto values = node_->get_parameter(name).as_string_array();
  values.erase(
      std::remove_if(values.begin(), values.end(), [](const auto& value) { return value.empty(); }),
      values.end());
  return values;
}

bool MTCTaskNode::recoveryOnly() const
{
  return node_->get_parameter("recovery_only").as_bool();
}

void MTCTaskNode::publishCollisionState(const mtc::SolutionBase& solution)
{
  if (collision_state_published_)
  {
    return;
  }

  const mtc::InterfaceState* interface_states[] = { solution.start(), solution.end() };
  for (const auto* interface_state : interface_states)
  {
    if (!interface_state || !interface_state->scene())
    {
      continue;
    }
    const auto& scene = interface_state->scene();
    const auto& robot_state = scene->getCurrentState();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 100;
    request.max_contacts_per_pair = 10;
    collision_detection::CollisionResult result;
    scene->checkSelfCollision(
        request, result, robot_state, scene->getAllowedCollisionMatrix());
    if (!result.collision)
    {
      continue;
    }

    moveit_msgs::msg::DisplayRobotState message;
    moveit::core::robotStateToRobotStateMsg(robot_state, message.state, true);
    std::vector<std::string> colliding_links;
    for (const auto& [pair, contacts] : result.contacts)
    {
      (void)contacts;
      colliding_links.push_back(pair.first);
      colliding_links.push_back(pair.second);
    }
    std::sort(colliding_links.begin(), colliding_links.end());
    colliding_links.erase(
        std::unique(colliding_links.begin(), colliding_links.end()), colliding_links.end());
    for (const auto& link : colliding_links)
    {
      moveit_msgs::msg::ObjectColor highlight;
      highlight.id = link;
      highlight.color.r = 1.0f;
      highlight.color.g = 0.0f;
      highlight.color.b = 0.0f;
      highlight.color.a = 1.0f;
      message.highlight_links.push_back(std::move(highlight));
    }
    collision_state_publisher_->publish(message);
    collision_state_published_ = true;
    RCLCPP_ERROR(
        LOGGER,
        "Published colliding IK posture to /mtc_collision_state for RViz (%zu highlighted links)",
        colliding_links.size());
    return;
  }
}

moveit_msgs::msg::CollisionObject MTCTaskNode::makeObject(double x, double y, double z) const
{
  moveit_msgs::msg::CollisionObject object;
  object.id = objectId();
  object.header.frame_id = stringParam("planning_frame");
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
  auto get_response = callPlanningSceneService<moveit_msgs::srv::GetPlanningScene>(
      service_node, get_scene_client, get_request, "Fetch MoveIt touch ACM");
  if (!get_response)
  {
    RCLCPP_ERROR(LOGGER, "Failed to fetch MoveIt ACM after retries");
    return false;
  }

  auto matrix = get_response->scene.allowed_collision_matrix;
  if (matrix.entry_names.empty())
  {
    RCLCPP_ERROR(LOGGER, "MoveIt returned an empty ACM before MTC execution");
    return false;
  }

  const auto object_id = objectId();
  addAcmName(matrix, object_id);
  setAcmDefault(matrix, object_id, true);
  const auto touch_links = stringArrayParam("touch_links");
  const auto obstacle_ids = stringArrayParam("pallet_obstacle_ids");
  for (const auto& id : obstacle_ids)
  {
    addAcmName(matrix, id);
  }
  for (const auto& link : touch_links)
  {
    addAcmName(matrix, link);
  }
  resizeAcm(matrix);

  for (const auto& link : touch_links)
  {
    setAcmPair(matrix, object_id, link, true);
  }
  // Previously picked objects can retain robot-contact permissions. A current
  // obstacle must not inherit those permissions from a preceding MTC task.
  for (const auto& id : obstacle_ids)
  {
    setAcmDefault(matrix, id, false);
    for (const auto& other : matrix.entry_names)
    {
      setAcmPair(matrix, id, other, false);
    }
  }

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.allowed_collision_matrix = matrix;

  auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  apply_request->scene = planning_scene;
  auto apply_response = callPlanningSceneService<moveit_msgs::srv::ApplyPlanningScene>(
      service_node, apply_scene_client, apply_request, "Apply MoveIt touch ACM");
  if (!apply_response || !apply_response->success)
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

bool MTCTaskNode::logRecoverySceneDiagnostics()
{
  constexpr auto service_timeout = std::chrono::seconds(10);
  auto service_node = std::make_shared<rclcpp::Node>(
      "mtc_recovery_scene_diagnostic_" + std::to_string(getpid()));
  auto client = service_node->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
  if (!client->wait_for_service(service_timeout))
  {
    RCLCPP_ERROR(LOGGER, "RECOVERY_SCENE_DIAGNOSTIC service_unavailable=true");
    return false;
  }

  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto response = callPlanningSceneService<moveit_msgs::srv::GetPlanningScene>(
      service_node, client, request, "Fetch recovery PlanningScene");
  if (!response)
  {
    RCLCPP_ERROR(LOGGER, "RECOVERY_SCENE_DIAGNOSTIC fetch_failed=true");
    return false;
  }

  planning_scene::PlanningScene scene(task_.getRobotModel());
  if (!scene.setPlanningSceneMsg(response->scene))
  {
    RCLCPP_ERROR(LOGGER, "RECOVERY_SCENE_DIAGNOSTIC scene_decode_failed=true");
    return false;
  }

  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = stringParam("arm_group_name");
  collision_request.contacts = true;
  collision_request.max_contacts = 100;
  collision_request.max_contacts_per_pair = 4;
  collision_detection::CollisionResult collision_result;
  scene.checkCollision(collision_request, collision_result);

  std::ostringstream attached;
  const auto& attached_objects = response->scene.robot_state.attached_collision_objects;
  for (size_t index = 0; index < attached_objects.size(); ++index)
  {
    if (index > 0)
    {
      attached << ",";
    }
    attached << attached_objects[index].object.id << "@" << attached_objects[index].link_name;
  }

  std::ostringstream contacts;
  size_t contact_index = 0;
  for (const auto& [pair, _] : collision_result.contacts)
  {
    if (contact_index++ > 0)
    {
      contacts << ",";
    }
    contacts << pair.first << ":" << pair.second;
  }

  RCLCPP_INFO_STREAM(
      LOGGER,
      "RECOVERY_SCENE_DIAGNOSTIC attached_count=" << attached_objects.size()
          << " attached=" << (attached.str().empty() ? "none" : attached.str())
          << " world_count=" << response->scene.world.collision_objects.size()
          << " acm_entries=" << response->scene.allowed_collision_matrix.entry_names.size()
          << " collision=" << (collision_result.collision ? "true" : "false")
          << " contact_count=" << collision_result.contact_count
          << " contact_pairs=" << (contacts.str().empty() ? "none" : contacts.str()));
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
  auto get_response = callPlanningSceneService<moveit_msgs::srv::GetPlanningScene>(
      service_node, get_scene_client, get_request, "Fetch PlanningScene");
  if (!get_response)
  {
    RCLCPP_ERROR(LOGGER, "Failed to fetch the PlanningScene after retries");
    return false;
  }

  const auto& current_scene = get_response->scene;
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

  moveit_msgs::msg::CollisionObject table;
  const bool table_enabled =
      param("table_size_x") > 0.0 && param("table_size_y") > 0.0 &&
      param("table_height") > 0.0;
  if (table_enabled)
  {
    table.id = "work_table";
    table.header.frame_id = stringParam("planning_frame");
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {
      param("table_size_x"), param("table_size_y"), param("table_height")
    };

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = param("table_center_x");
    table_pose.position.y = param("table_center_y");
    table_pose.position.z = param("table_top_z") - 0.5 * param("table_height");
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
  }

  moveit_msgs::msg::CollisionObject floor;
  floor.id = "floor_z_guard";
  floor.header.frame_id = stringParam("planning_frame");
  floor.primitives.resize(1);
  floor.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  floor.primitives[0].dimensions = { 4.0, 4.0, 0.02 };

  geometry_msgs::msg::Pose floor_pose;
  floor_pose.orientation.w = 1.0;
  floor_pose.position.x = 0.0;
  floor_pose.position.y = 0.0;
  floor_pose.position.z =
      param("table_top_z") - (table_enabled ? param("table_height") : 0.0) - 0.03;
  floor.primitive_poses.push_back(floor_pose);
  floor.operation = floor.ADD;

  moveit_msgs::msg::ObjectColor floor_color;
  floor_color.id = floor.id;
  floor_color.color.r = 0.0;
  floor_color.color.g = 0.2;
  floor_color.color.b = 1.0;
  floor_color.color.a = 0.25;

  moveit_msgs::msg::ObjectColor table_color;
  table_color.id = table.id;
  table_color.color.r = 0.45;
  table_color.color.g = 0.30;
  table_color.color.b = 0.15;
  table_color.color.a = 0.80;

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
  const auto obstacle_ids = stringArrayParam("pallet_obstacle_ids");
  const auto spheres = node_->get_parameter("pallet_obstacle_spheres").as_double_array();
  if (obstacle_ids.size() > 2 || spheres.size() != obstacle_ids.size() * 4 ||
      (!obstacle_ids.empty() && stringParam("planning_frame") != "base_footprint"))
  {
    RCLCPP_ERROR(LOGGER, "Invalid Pallet obstacle parameter shape or frame");
    return false;
  }
  std::vector<std::string> seen_obstacles;
  for (size_t i = 0; i < obstacle_ids.size(); ++i)
  {
    const auto& id = obstacle_ids[i];
    const double x = spheres[4 * i], y = spheres[4 * i + 1];
    const double z = spheres[4 * i + 2], radius = spheres[4 * i + 3];
    if ((id != "red_block" && id != "green_block" && id != "blue_block") || id == object_id ||
        std::find(seen_obstacles.begin(), seen_obstacles.end(), id) != seen_obstacles.end() ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(radius) ||
        std::abs(x) > 0.85 || std::abs(y) > 0.35 || z < 0.0 || z > 0.40 ||
        radius <= 0.0 || radius > 0.15 || is_attached(id))
    {
      RCLCPP_ERROR(LOGGER, "Invalid or still-attached Pallet obstacle: %s", id.c_str());
      return false;
    }
    seen_obstacles.push_back(id);
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.header.frame_id = "base_footprint";
    obstacle.id = id;
    obstacle.operation = obstacle.ADD;
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions = { radius };
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1.0;
    obstacle.primitives.push_back(sphere);
    obstacle.primitive_poses.push_back(pose);
    planning_scene.world.collision_objects.push_back(obstacle);
    RCLCPP_INFO(LOGGER, "PALLET_OBSTACLE_AUDIT id=%s radius_m=%.6f position_m=(%.6f,%.6f,%.6f)",
                id.c_str(), radius, x, y, z);
  }
  if (table_enabled)
  {
    planning_scene.world.collision_objects.push_back(table);
    planning_scene.object_colors.push_back(table_color);
  }
  planning_scene.world.collision_objects.push_back(floor);
  planning_scene.object_colors.push_back(floor_color);

  auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  apply_request->scene = planning_scene;
  auto apply_response = callPlanningSceneService<moveit_msgs::srv::ApplyPlanningScene>(
      service_node, apply_scene_client, apply_request, "Apply PlanningScene");
  if (!apply_response || !apply_response->success)
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

moveit::core::MoveItErrorCode MTCTaskNode::executeBlended(const mtc::SolutionBase& solution)
{
  using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
  ExecuteTaskSolution::Goal goal;
  solution.toMsg(goal.solution);

  const auto is_arm_trajectory = [](const auto& sub) {
    return sub.trajectory.joint_trajectory.joint_names.size() > 1 &&
           !sub.trajectory.joint_trajectory.points.empty();
  };

  std::vector<moveit_task_constructor_msgs::msg::SubTrajectory> merged;
  std::vector<std::vector<size_t>> merged_boundary_indices;
  size_t merged_boundaries = 0;
  for (auto sub : goal.solution.sub_trajectory)
  {
    if (!merged.empty() && is_arm_trajectory(merged.back()) && is_arm_trajectory(sub) &&
        merged.back().trajectory.joint_trajectory.joint_names ==
            sub.trajectory.joint_trajectory.joint_names)
    {
      auto& destination = merged.back().trajectory.joint_trajectory.points;
      auto& source = sub.trajectory.joint_trajectory.points;
      merged_boundary_indices.back().push_back(destination.size() - 1);
      const size_t begin = destination.back().positions == source.front().positions ? 1 : 0;
      destination.insert(destination.end(), source.begin() + begin, source.end());
      merged.back().scene_diff = std::move(sub.scene_diff);
      ++merged_boundaries;
      continue;
    }
    merged.push_back(std::move(sub));
    merged_boundary_indices.emplace_back();
  }

  moveit::core::RobotState reference_state(task_.getRobotModel());
  reference_state.setToDefaultValues();
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
  size_t arm_segments = 0;
  for (size_t sub_index = 0; sub_index < merged.size(); ++sub_index)
  {
    auto& sub = merged[sub_index];
    if (!sub.trajectory.joint_trajectory.points.empty())
    {
      sub.execution_info.controller_names = {
        is_arm_trajectory(sub) ? stringParam("arm_controller_name") :
                                 stringParam("gripper_controller_name")
      };
    }
    if (!is_arm_trajectory(sub))
    {
      continue;
    }

    auto& joint_trajectory = sub.trajectory.joint_trajectory;
    for (size_t index = 0; index < joint_trajectory.joint_names.size(); ++index)
    {
      reference_state.setVariablePosition(
          joint_trajectory.joint_names[index], joint_trajectory.points.front().positions[index]);
    }
    reference_state.update();

    robot_trajectory::RobotTrajectory trajectory(
        task_.getRobotModel(), stringParam("arm_group_name"));
    trajectory.setRobotTrajectoryMsg(reference_state, sub.trajectory);
    if (!time_parameterization.computeTimeStamps(
            trajectory, param("arm_velocity_scaling"), param("arm_acceleration_scaling")))
    {
      RCLCPP_ERROR(LOGGER, "Failed to retime blended arm trajectory");
      return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
    }
    trajectory.getRobotTrajectoryMsg(sub.trajectory);
    for (const size_t boundary_index : merged_boundary_indices[sub_index])
    {
      const auto& velocities = sub.trajectory.joint_trajectory.points[boundary_index].velocities;
      const double peak_velocity = velocities.empty() ? 0.0 :
          *std::max_element(velocities.begin(), velocities.end(),
                            [](double lhs, double rhs) { return std::abs(lhs) < std::abs(rhs); });
      RCLCPP_INFO(
          LOGGER, "Blended boundary at waypoint %zu keeps peak joint velocity %.6f rad/s",
          boundary_index, std::abs(peak_velocity));
    }
    ++arm_segments;
  }
  goal.solution.sub_trajectory = std::move(merged);

  RCLCPP_INFO(
      LOGGER, "Blended execution merged %zu arm boundary/boundaries into %zu arm segment(s)",
      merged_boundaries, arm_segments);
  if (merged_boundaries == 0)
  {
    RCLCPP_ERROR(LOGGER, "Blended execution found no adjacent arm trajectories to merge");
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }

  auto client = rclcpp_action::create_client<ExecuteTaskSolution>(node_, "execute_task_solution");
  if (!client->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(LOGGER, "ExecuteTaskSolution action server is unavailable");
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }

  auto goal_future = client->async_send_goal(goal);
  if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
      !goal_future.get())
  {
    RCLCPP_ERROR(LOGGER, "Blended ExecuteTaskSolution goal was rejected or timed out");
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }

  auto result_future = client->async_get_result(goal_future.get());
  result_future.wait();
  const auto wrapped_result = result_future.get();
  if (!wrapped_result.result)
  {
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }
  return moveit::core::MoveItErrorCode(wrapped_result.result->error_code.val);
}

bool MTCTaskNode::doTask()
{
  if (!palletReleasePositionValid(param("release_open_position"), stringParam("gripper_joint_name"),
                                 stringParam("planning_frame"), param("object_size_x"), param("object_size_y")))
  {
    RCLCPP_ERROR(LOGGER, "Invalid Pallet release opening: require positive block clearance and J8 bounds");
    return false;
  }
  const double place_ik_count = param("place_max_ik_solutions");
  const double place_ik_distance = param("place_ik_min_solution_distance");
  if (!std::isfinite(place_ik_count) || place_ik_count < 1.0 ||
      place_ik_count > 64.0 || std::floor(place_ik_count) != place_ik_count ||
      !std::isfinite(place_ik_distance) || place_ik_distance < 0.0)
  {
    RCLCPP_ERROR(LOGGER, "Invalid place IK search parameters");
    return false;
  }
  task_ = recoveryOnly() ? createRecoveryTask() : createTask();
  if (recoveryOnly() && !applyTouchCollisionAcm())
  {
    RCLCPP_ERROR(LOGGER, "Recovery touch ACM could not be restored; refusing motion");
    return false;
  }
  if (recoveryOnly() && !logRecoverySceneDiagnostics())
  {
    RCLCPP_WARN(LOGGER, "Recovery scene diagnostics were unavailable; planning remains fail-closed");
  }

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
  const double task_timeout = param("task_timeout");
  if (task_timeout > 0.0)
  {
    task_.setTimeout(task_timeout);
    RCLCPP_INFO(LOGGER, "Task planning timeout: %.1f s", task_timeout);
  }
  RCLCPP_INFO_STREAM(LOGGER, "Planning until " << max_solutions << " task solution(s) are available");
  const auto planning_started = std::chrono::steady_clock::now();
  if (!task_.plan(max_solutions))
  {
    const auto planning_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - planning_started).count();
    RCLCPP_ERROR(LOGGER, "Task planning failed after %.3f s", planning_seconds);
    std::ostringstream failure_details;
    task_.printState(failure_details);
    task_.explainFailure(failure_details);
    RCLCPP_ERROR_STREAM(LOGGER, "Task failure details:\n" << failure_details.str());
    return false;
  }
  const auto planning_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - planning_started).count();
  RCLCPP_INFO(LOGGER, "Task planning completed in %.3f s", planning_seconds);

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
  const auto obstacle_ids = stringArrayParam("pallet_obstacle_ids");
  if (!obstacle_ids.empty())
  {
    size_t checked_scenes = 0;
    if (!auditPalletObstacleSolution(best_solution, obstacle_ids, objectId(), checked_scenes) ||
        checked_scenes < 6)
    {
      RCLCPP_ERROR(LOGGER, "Selected Pallet obstacle scene/ACM audit failed; execution skipped");
      return false;
    }
  }

  const double max_solution_cost = param("max_solution_cost");
  if (max_solution_cost >= 0.0 && best_solution.cost() > max_solution_cost)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Best solution cost " << best_solution.cost()
                                                        << " exceeds execution limit "
                                                        << max_solution_cost
                                                        << "; execution skipped");
    return false;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Executing lowest-cost solution out of "
                                << solutions.size() << " planned solution(s), cost: "
                                << best_solution.cost());

  size_t segment_index = 0;
  double planned_trajectory_duration = 0.0;
  logTrajectorySegments(best_solution, segment_index, planned_trajectory_duration);
  RCLCPP_INFO(
      LOGGER, "Planned trajectory total: %.3f s across %zu controller segment(s)",
      planned_trajectory_duration, segment_index);

  if (node_->get_parameter("planning_only").as_bool())
  {
    RCLCPP_INFO(LOGGER, "Planning-only mode: execution skipped");
    return true;
  }

  const auto execution_started = std::chrono::steady_clock::now();
  const auto execution_mode = stringParam("execution_mode");
  if (execution_mode != "standard" && execution_mode != "blended")
  {
    RCLCPP_ERROR(LOGGER, "Unknown execution_mode '%s'", execution_mode.c_str());
    return false;
  }
  RCLCPP_INFO(LOGGER, "Execution mode: %s", execution_mode.c_str());
  auto result = execution_mode == "blended" ? executeBlended(best_solution) :
                                               task_.execute(best_solution);
  const auto execution_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - execution_started).count();
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(
        LOGGER, "Task execution failed after %.3f s with MoveIt error code: %d",
        execution_seconds, result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "Task execution completed in %.3f s", execution_seconds);

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

  const auto arm_group_name = stringParam("arm_group_name");
  const auto hand_group_name = stringParam("hand_group_name");
  const auto eef_name = stringParam("eef_name");
  const auto hand_frame = stringParam("hand_frame");
  const auto planning_frame = stringParam("planning_frame");
  const auto gripper_joint_name = stringParam("gripper_joint_name");
  const auto open_pose_name = stringParam("open_pose_name");
  const auto home_pose_name = stringParam("home_pose_name");
  const auto object_id = objectId();
  const auto gripper_collision_links = stringArrayParam("touch_links");
  const auto forearm_collision_links = stringArrayParam("forearm_collision_links");

  Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond grasp_rotation =
      Eigen::AngleAxisd(param("grasp_transform_roll"), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(param("grasp_transform_pitch"), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(param("grasp_transform_yaw"), Eigen::Vector3d::UnitZ());
  grasp_frame_transform.linear() = grasp_rotation.matrix();
  const double object_height = param("object_size_z");
  grasp_frame_transform.translation().x() =
      param("grasp_offset_x") + param("grasp_object_height_scale_x") * object_height;
  grasp_frame_transform.translation().y() =
      param("grasp_offset_y") + param("grasp_object_height_scale_y") * object_height;
  grasp_frame_transform.translation().z() =
      param("grasp_offset_z") + param("grasp_object_height_scale_z") * object_height;

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef_name);
  task.setProperty("ik_frame", hand_frame);

  auto sequence = std::make_unique<mtc::SerialContainer>("pick-place sequence");
  task.properties().exposeTo(sequence->properties(), { "group", "eef", "ik_frame" });
  sequence->properties().configureInitFrom(
      mtc::Stage::PARENT, { "group", "eef", "ik_frame" });

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  sequence->insert(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  sampling_planner->setPlannerId("ompl", plannerId());
  sampling_planner->setMaxVelocityScalingFactor(param("arm_velocity_scaling"));
  sampling_planner->setMaxAccelerationScalingFactor(param("arm_acceleration_scaling"));
  // The Cartesian stages provide the final grasp/place precision. Requiring
  // 1e-5 rad from the global sampling planner only increases rejection and
  // planning time without improving the final tool pose in practice.
  sampling_planner->setProperty("goal_joint_tolerance", 1e-3);
  RCLCPP_INFO_STREAM(LOGGER, "Using OMPL planner: " << plannerId());
  auto gripper_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  gripper_interpolation_planner->setMaxVelocityScalingFactor(param("gripper_velocity_scaling"));
  gripper_interpolation_planner->setMaxAccelerationScalingFactor(
      param("gripper_acceleration_scaling"));

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(param("cartesian_velocity_scaling"));
  cartesian_planner->setMaxAccelerationScalingFactor(param("cartesian_acceleration_scaling"));
  cartesian_planner->setStepSize(.01);
  RCLCPP_INFO(
      LOGGER,
      "Motion scaling: arm velocity/acceleration %.2f/%.2f, Cartesian %.2f/%.2f, gripper %.2f/%.2f",
      param("arm_velocity_scaling"), param("arm_acceleration_scaling"),
      param("cartesian_velocity_scaling"), param("cartesian_acceleration_scaling"),
      param("gripper_velocity_scaling"), param("gripper_acceleration_scaling"));

  if (!forearm_collision_links.empty() && !gripper_collision_links.empty())
  {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper forearm self-collision");
    stage->allowCollisions(forearm_collision_links, gripper_collision_links, true);
    sequence->insert(std::move(stage));
  }

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", gripper_interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal(open_pose_name);
  mtc::Stage* open_hand_stage_ptr = stage_open_hand.get();
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
  stage->addSolutionCallback([](const mtc::SolutionBase& solution) {
    if (solution.isFailure())
      RCLCPP_WARN(LOGGER, "MTC approach failure: %s", solution.comment().c_str());
  });
  stage->properties().set("marker_ns", "approach_object");
  stage->properties().set("link", hand_frame);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(
      param("approach_min_distance"), param("approach_max_distance"));

  // Descend in the world frame so the approach remains vertical regardless of
  // which axis-aligned yaw candidate is selected for the gripper.
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = planning_frame;
  vec.vector.z = -1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
}

{
  // Sample grasp pose
  auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  stage->setPreGraspPose(open_pose_name);
  stage->setObject(object_id);
  // Evaluate both orthogonal block faces. The half-turn variants are retained
  // because they can lead to a substantially shorter arm IK branch.
  stage->setAngleDelta(param("grasp_angle_delta"));
  stage->setMonitoredStage(open_hand_stage_ptr);

  // Compute IK
  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(4);
  wrapper->setMinSolutionDistance(0.5);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  // Keep ComputeIK's native cost: joint-space distance from the monitored
  // current state.  Overriding this with the fixed "ready" pose can prefer a
  // distant wrist/shoulder branch even when a nearby IK solution is available.
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  grasp->insert(std::move(wrapper));
}

{
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("allow pick object robot contact");
  stage->allowCollisions(object_id, true);
  const auto obstacle_ids = stringArrayParam("pallet_obstacle_ids");
  if (!obstacle_ids.empty())
  {
    stage->allowCollisions(object_id, obstacle_ids, false);
  }
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
    stage->setGoal(std::map<std::string, double>{ { gripper_joint_name, position } });
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
  stage->setMinMaxDistance(param("lift_min_distance"), param("lift_max_distance"));
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "lift_object");

  // Set upward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = planning_frame;
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
}

  sequence->insert(std::move(grasp));
}

{
  const bool cartesian_transfer = node_->get_parameter("cartesian_transfer").as_bool();
  mtc::solvers::PlannerInterfacePtr transfer_planner = sampling_planner;
  if (cartesian_transfer)
  {
    auto straight_line_planner = std::make_shared<mtc::solvers::CartesianPath>();
    straight_line_planner->setIKFrame(hand_frame);
    straight_line_planner->setMaxVelocityScalingFactor(param("cartesian_velocity_scaling"));
    straight_line_planner->setMaxAccelerationScalingFactor(param("cartesian_acceleration_scaling"));
    straight_line_planner->setStepSize(.01);
    straight_line_planner->setMinFraction(1.0);
    transfer_planner = straight_line_planner;
    RCLCPP_INFO(
        LOGGER,
        "Using straight Cartesian transfer for the loaded move to place; IK frame: %s",
        hand_frame.c_str());
  }
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move above place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, transfer_planner } });
  stage_move_to_place->setTimeout(param("move_to_place_timeout"));
  if (node_->get_parameter("upright_transfer").as_bool())
  {
    const Eigen::Matrix3d desired_hand_rotation =
        Eigen::AngleAxisd(param("place_yaw"), Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        grasp_frame_transform.linear().transpose();
    const Eigen::Quaterniond desired_hand_orientation(desired_hand_rotation);
    moveit_msgs::msg::OrientationConstraint constraint;
    constraint.header.frame_id = planning_frame;
    constraint.link_name = hand_frame;
    constraint.orientation.x = desired_hand_orientation.x();
    constraint.orientation.y = desired_hand_orientation.y();
    constraint.orientation.z = desired_hand_orientation.z();
    constraint.orientation.w = desired_hand_orientation.w();
    constraint.absolute_x_axis_tolerance = param("transfer_orientation_tolerance");
    constraint.absolute_y_axis_tolerance = param("transfer_orientation_tolerance");
    constraint.absolute_z_axis_tolerance = M_PI;
    constraint.weight = 1.0;
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "upright_loaded_transfer";
    constraints.orientation_constraints.push_back(std::move(constraint));
    stage_move_to_place->setPathConstraints(std::move(constraints));
    RCLCPP_INFO(
        LOGGER,
        "Constraining loaded transfer roll/pitch to %.1f degrees; yaw remains free",
        param("transfer_orientation_tolerance") * 180.0 / M_PI);
  }
  const double transfer_min_clearance = param("transfer_min_clearance");
  const double transfer_clearance_check_step = param("transfer_clearance_check_step");
  std::vector<Eigen::Vector3d> bottom_corners_in_hand;
  if (transfer_min_clearance >= 0.0)
  {
    if (transfer_clearance_check_step <= 0.0)
    {
      throw std::runtime_error("transfer_clearance_check_step must be positive");
    }
    const double half_x = 0.5 * param("object_size_x");
    const double half_y = 0.5 * param("object_size_y");
    const double half_z = 0.5 * param("object_size_z");
    for (const double x_sign : { -1.0, 1.0 })
    {
      for (const double y_sign : { -1.0, 1.0 })
      {
        const Eigen::Vector3d corner_in_object(x_sign * half_x, y_sign * half_y, -half_z);
        bottom_corners_in_hand.push_back(grasp_frame_transform * corner_in_object);
      }
    }
    RCLCPP_INFO(
        LOGGER,
        "Rejecting transfer paths with an attached-object bottom corner below %.3f m above table Z %.3f m",
        transfer_min_clearance,
        param("table_top_z"));
  }
  const double move_to_place_max_path_length = param("move_to_place_max_path_length");
  const double table_top_z = param("table_top_z");
  stage_move_to_place->setCostTerm(
      [move_to_place_max_path_length, transfer_min_clearance, transfer_clearance_check_step,
       table_top_z, hand_frame, bottom_corners_in_hand](
          const mtc::SubTrajectory& solution, std::string& comment) {
        const auto trajectory = solution.trajectory();
        if (transfer_min_clearance >= 0.0 && trajectory && trajectory->getWayPointCount() > 0)
        {
          double minimum_clearance = std::numeric_limits<double>::infinity();
          const auto* group = trajectory->getGroup();
          auto inspect_state = [&](const moveit::core::RobotState& state) {
            const Eigen::Isometry3d& hand_pose = state.getGlobalLinkTransform(hand_frame);
            for (const auto& corner : bottom_corners_in_hand)
            {
              minimum_clearance = std::min(
                  minimum_clearance, (hand_pose * corner).z() - table_top_z);
            }
          };

          inspect_state(trajectory->getWayPoint(0));
          for (size_t index = 1; index < trajectory->getWayPointCount(); ++index)
          {
            const auto& from = trajectory->getWayPoint(index - 1);
            const auto& to = trajectory->getWayPoint(index);
            const double distance = group ? from.distance(to, group) : from.distance(to);
            const size_t subdivisions = std::max<size_t>(
                1, static_cast<size_t>(std::ceil(distance / transfer_clearance_check_step)));
            for (size_t sample = 1; sample <= subdivisions; ++sample)
            {
              moveit::core::RobotState interpolated(from);
              const double fraction = static_cast<double>(sample) / subdivisions;
              if (group)
              {
                from.interpolate(to, fraction, interpolated, group);
              }
              else
              {
                from.interpolate(to, fraction, interpolated);
              }
              interpolated.updateLinkTransforms();
              inspect_state(interpolated);
            }
          }

          if (minimum_clearance + 1e-6 < transfer_min_clearance)
          {
            std::ostringstream stream;
            stream << "transfer bottom clearance too small: " << minimum_clearance << " < "
                   << transfer_min_clearance;
            comment = stream.str();
            return std::numeric_limits<double>::infinity();
          }
        }
        mtc::cost::PathLength path_length_cost;
        const double path_length = path_length_cost(solution, comment);
        if (move_to_place_max_path_length >= 0.0 && path_length > move_to_place_max_path_length)
        {
          std::ostringstream stream;
          stream << "move above place path too long: " << path_length << " > "
                 << move_to_place_max_path_length;
          comment = stream.str();
          return std::numeric_limits<double>::infinity();
        }
        return path_length;
      });
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  sequence->insert(std::move(stage_move_to_place));
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
  stage->setObject(object_id);

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = planning_frame;
  target_pose_msg.pose.position.x = param("place_x");
  target_pose_msg.pose.position.y = param("place_y");
  target_pose_msg.pose.position.z =
      param("place_z") + std::max(0.0, param("lower_max_distance"));
  const double place_half_yaw = 0.5 * param("place_yaw");
  target_pose_msg.pose.orientation.z = std::sin(place_half_yaw);
  target_pose_msg.pose.orientation.w = std::cos(place_half_yaw);
  stage->setPose(target_pose_msg);
  stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

  // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
    wrapper->addSolutionCallback([this](const mtc::SolutionBase& solution) {
      if (solution.isFailure())
      {
        RCLCPP_WARN(LOGGER, "MTC place IK failure: %s", solution.comment().c_str());
        publishCollisionState(solution);
      }
    });
  wrapper->setMaxIKSolutions(static_cast<unsigned int>(param("place_max_ik_solutions")));
  wrapper->setMinSolutionDistance(param("place_ik_min_solution_distance"));
  wrapper->setIKFrame(object_id);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  place->insert(std::move(wrapper));
}

if (param("lower_max_distance") > 0.0)
{
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
  stage->addSolutionCallback([](const mtc::SolutionBase& solution) {
    if (solution.isFailure())
      RCLCPP_WARN(LOGGER, "MTC lower failure: %s", solution.comment().c_str());
  });
  stage->properties().set("marker_ns", "lower_object");
  stage->properties().set("link", hand_frame);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(param("lower_min_distance"), param("lower_max_distance"));
  stage->setIKFrame(hand_frame);

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = planning_frame;
  vec.vector.z = -1.0;
  stage->setDirection(vec);
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", gripper_interpolation_planner);
  stage->setGroup(hand_group_name);
  const double release_open_position = param("release_open_position");
  if (release_open_position == -1.0)
    stage->setGoal(open_pose_name);
  else
  {
    stage->setGoal(std::map<std::string, double>{{stringParam("gripper_joint_name"), release_open_position}});
    RCLCPP_INFO(LOGGER, "PALLET_RELEASE_OPEN_POSITION_M %.6f (pre-grasp named opening unchanged)",
                release_open_position);
  }
  if (!stringArrayParam("pallet_obstacle_ids").empty())
  {
    stage->addSolutionCallback([hand_group_name, object_id](const mtc::SolutionBase& solution) {
      if (!solution.isFailure())
        return;
      const auto* state = solution.end();
      if (!state || !state->scene())
      {
        RCLCPP_WARN(LOGGER, "PALLET_OPEN_HAND_FAILURE target=%s goal_scene_unavailable comment=%s",
                    object_id.c_str(), solution.comment().c_str());
        return;
      }
      collision_detection::CollisionRequest request;
      request.group_name = hand_group_name;
      request.contacts = true;
      request.max_contacts = 100;
      request.max_contacts_per_pair = 10;
      collision_detection::CollisionResult result;
      const auto& scene = state->scene();
      scene->checkCollision(request, result, scene->getCurrentState(), scene->getAllowedCollisionMatrix());
      RCLCPP_WARN(LOGGER, "PALLET_OPEN_HAND_FAILURE target=%s collision=%s pairs=%zu comment=%s",
                  object_id.c_str(), result.collision ? "true" : "false",
                  result.contacts.size(), solution.comment().c_str());
      for (const auto& [pair, contacts] : result.contacts)
      {
        double max_depth = 0.0;
        for (const auto& contact : contacts)
          max_depth = std::max(max_depth, contact.depth);
        RCLCPP_WARN(LOGGER, "PALLET_OPEN_HAND_CONTACT target=%s pair=%s/%s contacts=%zu max_depth_m=%.6f",
                    object_id.c_str(), pair.first.c_str(), pair.second.c_str(), contacts.size(), max_depth);
      }
      // Diagnostic state copies only: never change the planned goal or command hardware.
      for (const double opening : {0.016, 0.018, 0.020, 0.025, 0.0335})
      {
        moveit::core::RobotState candidate(scene->getCurrentState());
        candidate.setVariablePosition("J8", opening);
        candidate.setVariablePosition("J9", opening);
        candidate.update();
        collision_detection::CollisionResult candidate_result;
        scene->checkCollision(request, candidate_result, candidate, scene->getAllowedCollisionMatrix());
        RCLCPP_WARN(LOGGER, "PALLET_RELEASE_SWEEP target=%s one_jaw_m=%.4f collision=%s pairs=%zu",
                    object_id.c_str(), opening, candidate_result.collision ? "true" : "false",
                    candidate_result.contacts.size());
      }
    });
  }
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage->detachObject(object_id, hand_frame);
  place->insert(std::move(stage));
}

if (param("retreat_max_distance") > 0.0)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(param("retreat_min_distance"), param("retreat_max_distance"));
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "retreat");

  // Rise vertically before planning the return-home motion.
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = planning_frame;
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
  const double return_home_max_path_length = param("return_home_max_path_length");
  stage->setCostTerm(
      [return_home_max_path_length](const mtc::SubTrajectory& solution, std::string& comment) {
        mtc::cost::PathLength path_length_cost;
        const double path_length = path_length_cost(solution, comment);
        if (return_home_max_path_length >= 0.0 && path_length > return_home_max_path_length)
        {
          std::ostringstream stream;
          stream << "return home path too long: " << path_length << " > "
                 << return_home_max_path_length;
          comment = stream.str();
          return std::numeric_limits<double>::infinity();
        }
        return path_length;
      });
  stage->setGoal(home_pose_name);
  sequence->insert(std::move(stage));
}

  task.add(std::move(sequence));

  return task;
}

mtc::Task MTCTaskNode::createRecoveryTask()
{
  mtc::Task task;
  task.stages()->setName("grasp failure recovery");
  task.loadRobotModel(node_);

  const auto arm_group_name = stringParam("arm_group_name");
  const auto hand_group_name = stringParam("hand_group_name");
  const auto eef_name = stringParam("eef_name");
  const auto hand_frame = stringParam("hand_frame");
  const auto planning_frame = stringParam("planning_frame");
  const auto open_pose_name = stringParam("open_pose_name");
  const auto home_pose_name = stringParam("home_pose_name");
  const auto recovery_candidate_id = stringParam("recovery_candidate_id");
  const double direction_x = param("recovery_direction_x");
  const double direction_y = param("recovery_direction_y");
  const double direction_z = param("recovery_direction_z");
  const double direction_norm = std::sqrt(
      direction_x * direction_x + direction_y * direction_y + direction_z * direction_z);
  if (!std::isfinite(direction_norm) || direction_norm < 1e-9)
  {
    throw std::runtime_error("Recovery direction must be finite and non-zero");
  }

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef_name);
  task.setProperty("ik_frame", hand_frame);

  auto sequence = std::make_unique<mtc::SerialContainer>("hold-retreat-open-home");
  task.properties().exposeTo(sequence->properties(), { "group", "eef", "ik_frame" });
  sequence->properties().configureInitFrom(
      mtc::Stage::PARENT, { "group", "eef", "ik_frame" });
  sequence->insert(std::make_unique<mtc::stages::CurrentState>("current"));

  // A safety stop holds the measured pose first.  Move upward while retaining
  // the grasp; opening before retreat could drop a slipping object in place.
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  const double retreat_distance = param("recovery_retreat_distance");
  const double probe_distance = param("recovery_probe_distance");
  if (!std::isfinite(retreat_distance) || !std::isfinite(probe_distance) ||
      retreat_distance <= 0.0 || probe_distance <= 0.0 || probe_distance > retreat_distance)
  {
    throw std::runtime_error(
        "Recovery distances must be finite and satisfy 0 < probe <= retreat");
  }
  cartesian_planner->setStepSize(std::min(0.0025, probe_distance / 2.0));

  std::vector<double> retreat_targets;
  for (double target = probe_distance; target < retreat_distance; target *= 2.0)
  {
    retreat_targets.push_back(target);
  }
  if (retreat_targets.empty() || retreat_targets.back() < retreat_distance - 1e-9)
  {
    retreat_targets.push_back(retreat_distance);
  }

  double previous_target = 0.0;
  for (const double target : retreat_targets)
  {
    const double increment = target - previous_target;
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>(
            "recovery retreat " + recovery_candidate_id + " to " +
                std::to_string(target) + " m",
            cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->properties().set("link", hand_frame);
    stage->properties().set("timeout", param("recovery_retreat_timeout"));
    stage->setMinMaxDistance(increment, increment);
    stage->setIKFrame(hand_frame);

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = planning_frame;
    direction.vector.x = direction_x / direction_norm;
    direction.vector.y = direction_y / direction_norm;
    direction.vector.z = direction_z / direction_norm;
    stage->setDirection(direction);
    sequence->insert(std::move(stage));
    previous_target = target;
  }

  auto gripper_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("recovery open hand", gripper_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal(open_pose_name);
    sequence->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("recovery detach object");
    stage->detachObject(objectId(), hand_frame);
    sequence->insert(std::move(stage));
  }

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  sampling_planner->setPlannerId("ompl", plannerId());
  sampling_planner->setProperty("goal_joint_tolerance", 1e-3);
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("recovery return home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->properties().set("timeout", param("recovery_home_timeout"));
    stage->setGoal(home_pose_name);
    sequence->insert(std::move(stage));
  }

  task.add(std::move(sequence));
  RCLCPP_INFO_STREAM(
      LOGGER,
      "Configured recovery candidate=" << recovery_candidate_id << " direction="
          << planning_frame << "[" << direction_x / direction_norm << ","
          << direction_y / direction_norm << "," << direction_z / direction_norm << "] distance="
          << retreat_distance << " m probe=" << probe_distance << " m stages="
          << retreat_targets.size() << " -> " << open_pose_name << " -> "
          << home_pose_name);
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

  // Recovery must preserve the interrupted PlanningScene, especially its
  // attached-object state.  Recreating the nominal scene would teleport the
  // collision object back to the original pick pose.
  const bool scene_ready = mtc_task_node->recoveryOnly() || mtc_task_node->setupPlanningScene();
  const bool success = scene_ready && mtc_task_node->doTask();
  mtc_task_node->clearTaskVisualization();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
