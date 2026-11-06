#include <moveit/motion_planning_rviz_plugin/motion_planning_display.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <QPushButton>
#include <QPointer>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pallet_physical_moveit_config {
// Native gripper preview and explicit Execute-only direct angle control.
class AdaptiveMotionPlanningDisplay : public moveit_rviz_plugin::MotionPlanningDisplay {
public:
  AdaptiveMotionPlanningDisplay() {
    QFile file(QString::fromStdString(ament_index_cpp::get_package_share_directory(
      "pallet_physical_moveit_config")+"/config/adaptive_preview_states.json"));
    if(!file.open(QIODevice::ReadOnly))throw std::runtime_error("Cannot load adaptive linkage preview table");
    const auto root=QJsonDocument::fromJson(file.readAll()).object();
    lower_=root["lower_rad"].toDouble();step_=root["step_rad"].toDouble();
    for(const auto & name:root["joint_names"].toArray())names_.push_back(name.toString().toStdString());
    for(const auto & item:root["positions"].toArray()) {
      std::vector<double> row;
      for(const auto & q:item.toArray())row.push_back(q.toDouble());
      if(row.size()!=names_.size())throw std::runtime_error("Invalid adaptive preview row");
      positions_.push_back(row);
    }
    if(step_<=0 || names_.size()!=15 || positions_.size()<2)
      throw std::runtime_error("Invalid adaptive preview calibration");
  }
  void update(float wall_dt,float ros_dt) override {
    moveit_rviz_plugin::MotionPlanningDisplay::update(wall_dt,ros_dt);
    if(!getRobotModel() || !getRobotInteraction() || !getRobotModel()->hasJointModel("gripper_motor"))return;
    couple(true);couple(false);
    updateExecution();
  }
private:
  using Action=control_msgs::action::FollowJointTrajectory;
  using Handle=rclcpp_action::ClientGoalHandle<Action>;
  void report(const std::string & text, bool error=false) {
    setStatus(error ? rviz_common::properties::StatusProperty::Error :
      rviz_common::properties::StatusProperty::Ok,"Gripper Execute",QString::fromStdString(text));
  }
  void executeGripper() {
    if(busy_)return;
    double start; rclcpp::Time stamp(0,0,node_->get_clock()->get_clock_type());
    {std::lock_guard<std::mutex> lock(feedback_mutex_); start=measured_;stamp=stamp_;}
    const double age=(node_->now()-stamp).seconds();
    const double target=getQueryGoalState()->getVariablePosition("gripper_motor");
    if(!std::isfinite(start) || std::abs(start)>12. || age<0 || age>.25) {report("No fresh gripper feedback",true);return;}
    if(!std::isfinite(target) || target < -12. || target > 12.) {report("Target outside [-12,12] rad",true);return;}
    if(!client_->action_server_is_ready()){report("Gripper bridge unavailable",true);return;}
    double distance=std::abs(target-start);
    // Arrival budget for the drive ramp; bridge sends the final angle directly.
    double duration=std::max({1.,1.5*distance/2.,std::sqrt(6*distance/5.)});
    if(duration>60.){report("Choose an intermediate angle (move exceeds 60 s)",true);return;}
    Action::Goal goal;goal.trajectory.joint_names={"gripper_motor"};
    trajectory_msgs::msg::JointTrajectoryPoint first,last;
    first.positions={start};first.velocities={0.};last.positions={target};last.velocities={0.};
    last.time_from_start=rclcpp::Duration::from_seconds(duration);
    goal.trajectory.points={first,last};
    try {goal_future_=client_->async_send_goal(goal);busy_=true;
      report("Executing gripper angle; no MoveIt collision planning");}
    catch(const std::exception & e){report(e.what(),true);}
  }
  void updateExecution() {
    if(!execute_ && frame_) {
      execute_=frame_->findChild<QPushButton*>("execute_button");
      stop_=frame_->findChild<QPushButton*>("stop_button");
      plan_=frame_->findChild<QPushButton*>("plan_button");
      plan_execute_=frame_->findChild<QPushButton*>("plan_and_execute_button");
      if(!execute_ || !stop_ || !plan_ || !plan_execute_) {execute_=nullptr;return;}
      client_=rclcpp_action::create_client<Action>(node_,"/pallet_gripper_controller/follow_joint_trajectory");
      feedback_=node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg){
          auto it=std::find(msg->name.begin(),msg->name.end(),"gripper_motor");
          if(it==msg->name.end())return;
          size_t i=it-msg->name.begin();if(i>=msg->position.size())return;
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          measured_=msg->position[i];stamp_=rclcpp::Time(msg->header.stamp,node_->get_clock()->get_clock_type());
        });
      QObject::disconnect(execute_,SIGNAL(clicked()),frame_,SLOT(executeButtonClicked()));
      QObject::connect(execute_,&QPushButton::clicked,this,[this](){
        if(getCurrentPlanningGroup()=="gripper")executeGripper();
        else QMetaObject::invokeMethod(frame_,"executeButtonClicked");
      });
      QObject::connect(stop_,&QPushButton::clicked,this,[this](){
        if(handle_)client_->async_cancel_goal(handle_);
        else if(busy_)cancel_pending_=true;
      });
    }
    if(!execute_)return;
    if(busy_) {
      if(goal_future_.valid() && goal_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        handle_=goal_future_.get();goal_future_={};
        if(!handle_) {busy_=false;cancel_pending_=false;report("Bridge rejected target; inspect bridge status",true);}
        else {result_future_=client_->async_get_result(handle_);
          if(cancel_pending_){client_->async_cancel_goal(handle_);cancel_pending_=false;}}
      }
      if(result_future_.valid() && result_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        auto result=result_future_.get();result_future_={};handle_.reset();busy_=false;
        bool ok=result.result && result.code==rclcpp_action::ResultCode::SUCCEEDED && result.result->error_code==0;
        if(getCurrentPlanningGroup()=="gripper")stop_->setEnabled(false);
        report(ok ? "Target reached; holding position" : (result.result ? "Execution stopped: "+result.result->error_string : "No controller result"),!ok);
      }
    }
    const bool gripper=getCurrentPlanningGroup()=="gripper";
    // The linkage is coupled in the preview, not in OMPL's state space.
    // Do not offer collision-planned execution for this unsupported group.
    if(gripper) {
      if(!was_gripper_) {
        plan_enabled_=plan_->isEnabled();plan_execute_enabled_=plan_execute_->isEnabled();
        plan_tooltip_=plan_->toolTip();plan_execute_tooltip_=plan_execute_->toolTip();
      }
      plan_->setEnabled(false);plan_execute_->setEnabled(false);
      const QString hint="Gripper uses Execute for direct motor control; MoveIt collision planning is unavailable.";
      plan_->setToolTip(hint);plan_execute_->setToolTip(hint);
      execute_->setToolTip(hint);
    } else if(was_gripper_) {
      plan_->setEnabled(plan_enabled_);plan_execute_->setEnabled(plan_execute_enabled_);
      plan_->setToolTip(plan_tooltip_);plan_execute_->setToolTip(plan_execute_tooltip_);
      execute_->setToolTip("");
    }
    if(gripper) {execute_->setEnabled(!busy_);if(busy_)stop_->setEnabled(true);}
    else if(was_gripper_)execute_->setEnabled(false); // Require a new arm plan after switching groups.
    was_gripper_=gripper;
  }
  QPointer<QPushButton> execute_,stop_,plan_,plan_execute_;
  bool plan_enabled_=true,plan_execute_enabled_=true;
  QString plan_tooltip_,plan_execute_tooltip_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_;
  std::mutex feedback_mutex_;
  double measured_=NAN;
  rclcpp::Time stamp_{0,0,RCL_ROS_TIME};
  bool busy_=false,cancel_pending_=false,was_gripper_=false;
  Handle::SharedPtr handle_;
  std::shared_future<Handle::SharedPtr> goal_future_;
  std::shared_future<Handle::WrappedResult> result_future_;
  void couple(bool goal) {
    const auto state=goal ? getQueryGoalState():getQueryStartState();
    if(!state)return;
    const double q=state->getVariablePosition("gripper_motor");
    if(!std::isfinite(q) || q<lower_-1e-8 || q>lower_+step_*(positions_.size()-1)+1e-8)return;
    const double index=std::clamp((q-lower_)/step_,0.,double(positions_.size()-1));
    const size_t a=std::min(size_t(index),positions_.size()-2);
    const double fraction=index-a;
    auto result=*state;bool changed=false;
    for(size_t j=0;j<names_.size();++j) {
      const double expected=positions_[a][j]*(1.-fraction)+positions_[a+1][j]*fraction;
      if(std::abs(state->getVariablePosition(names_[j])-expected)>1e-10) {
        result.setVariablePosition(names_[j],expected);changed=true;
      }
    }
    if(changed) {
      result.update();
      if(goal)setQueryGoalState(result);else setQueryStartState(result);
    }
  }
  double lower_,step_;
  std::vector<std::string> names_;
  std::vector<std::vector<double>> positions_;
};
}
PLUGINLIB_EXPORT_CLASS(pallet_physical_moveit_config::AdaptiveMotionPlanningDisplay,rviz_common::Display)
