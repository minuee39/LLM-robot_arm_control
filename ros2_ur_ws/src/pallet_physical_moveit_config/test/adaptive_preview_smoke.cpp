#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.hpp>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame_joints_widget.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QApplication>
#include <QAbstractItemView>
#include <QTabWidget>
#include <QTimer>
#include <QScreen>
#include <QPushButton>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <iostream>
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);QApplication app(argc,argv);
  auto node=std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("adaptive_preview_smoke");
  auto frame=new rviz_common::VisualizationFrame(node);frame->setApp(&app);frame->setSplashPath("");
  frame->initialize(node,QString::fromStdString(ament_index_cpp::get_package_share_directory(
    "pallet_physical_moveit_config")+"/config/adaptive_moveit.rviz"));frame->show();
  moveit_rviz_plugin::MotionPlanningDisplay * display=nullptr;
  auto group=frame->getManager()->getRootDisplayGroup();
  for(int i=0;i<group->numDisplays();++i)
    if(group->getDisplayAt(i)->getClassId()=="pallet_physical_moveit_config/AdaptiveMotionPlanning")
      display=dynamic_cast<moveit_rviz_plugin::MotionPlanningDisplay*>(group->getDisplayAt(i));
  if(!display){std::cerr<<"FAIL: adaptive display not loaded"<<std::endl;return 2;}
  double feedback_angle=NAN;
  auto subscription=node->get_raw_node()->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
    [&](sensor_msgs::msg::JointState::ConstSharedPtr msg){
      for(size_t i=0;i<msg->name.size();++i)if(msg->name[i]=="gripper_motor")feedback_angle=msg->position[i];
    });
  QTimer timer;int stage=0;double old=0.;
  QObject::connect(&timer,&QTimer::timeout,[&](){
    if(!display->getRobotModel() || !display->getRobotInteraction())return;
    if(stage==0) {display->subProp("Planning Request")->subProp("Planning Group")->setValue("gripper");stage=1;return;}
    if(stage==1) {
      for(const auto name:{"plan_button","plan_and_execute_button"}) {
        auto button=frame->findChild<QPushButton*>(name);
        if(!button || button->isEnabled()) {
          std::cerr<<"FAIL: unsupported gripper planning button enabled: "<<name<<std::endl;app.exit(9);return;
        }
      }
      for(auto tab:frame->findChildren<QTabWidget*>())
        for(int i=0;i<tab->count();++i)if(tab->tabText(i)=="Joints")tab->setCurrentIndex(i);
      for(auto view:frame->findChildren<QAbstractItemView*>()) {
        auto model=dynamic_cast<moveit_rviz_plugin::JMGItemModel*>(view->model());
        if(!model || !model->getJointModelGroup() || model->getJointModelGroup()->getName()!="gripper")continue;
        if(!view->isVisible())continue;
        old=display->getQueryGoalState()->getVariablePosition("g2");
        for(int row=0;row<model->rowCount();++row) {
          if(model->data(model->index(row,0),Qt::DisplayRole).toString()!="gripper_motor")continue;
          if(model->setData(model->index(row,1),180.,Qt::EditRole)){stage=2;return;}
        }
      }
    }else if(stage==2) {
      auto state=display->getQueryGoalState();double q=state->getVariablePosition("gripper_motor");
      if(std::abs(q)<.01)return;
      if(std::abs(state->getVariablePosition("g2")-old)<1e-7)return;
      // Reproduce repeated measured-state updates while a different goal is edited.
      const double expected_nut=state->getVariablePosition("g2");
      auto measured=*state;measured.setVariablePosition("gripper_motor",0.);
      measured.setVariablePosition("g2",0.);
      for(int i=0;i<100;++i) {
        display->updateQueryStates(measured);
        if(std::abs(display->getQueryGoalState()->getVariablePosition("g2")-expected_nut)>1e-10) {
          std::cerr<<"FAIL: measured state overwrote coupled goal"<<std::endl;app.exit(5);return;
        }
      }
      std::cout<<"PASS: 100 measured updates preserve coupled gripper goal"<<std::endl;
      auto l=state->getGlobalLinkTransform("l_pad_link").translation();
      auto r=state->getGlobalLinkTransform("r_pad_link").translation();
      std::cout<<"PASS: native MoveIt Joints table -> motor "<<q<<" rad -> coupled nut "
        <<state->getVariablePosition("g2")<<" m; pad distance "<<(l-r).norm()<<std::endl;
      if(!display->subProp("Planning Request")->subProp("Query Goal State")->getValue().toBool()) {
        std::cerr<<"FAIL: goal state rendering disabled"<<std::endl;app.exit(4);return;
      }
      auto view=frame->getManager()->getViewManager()->getCurrent();
      const auto focus=state->getGlobalLinkTransform("gripper").translation();
      auto focal=view->subProp("Focal Point");
      focal->subProp("X")->setValue(focus.x());focal->subProp("Y")->setValue(focus.y());focal->subProp("Z")->setValue(focus.z());
      view->subProp("Distance")->setValue(.35);
      stage=3;
      QTimer::singleShot(1000,[&](){
        if(!std::isfinite(feedback_angle) || std::abs(feedback_angle)>.001){std::cerr<<"FAIL: slider moved motor without Execute"<<std::endl;app.exit(6);return;}
        auto button=frame->findChild<QPushButton*>("execute_button");
        if(!button || !button->isEnabled()){std::cerr<<"FAIL: Execute unavailable without Plan"<<std::endl;app.exit(7);return;}
        auto probe=std::make_shared<rclcpp::Node>("gripper_smoke_dry_run_check");
        auto params=std::make_shared<rclcpp::SyncParametersClient>(probe,"/pallet_serial_trajectory_bridge");
        if(!params->wait_for_service(std::chrono::seconds(1)) ||
           !params->get_parameter<bool>("dry_run",false)){
          std::cerr<<"FAIL: refusing Execute without a dry-run bridge"<<std::endl;app.exit(8);return;
        }
        button->click();stage=4;
      });
    }else if(stage==4 && std::abs(feedback_angle-M_PI)<1e-6 && frame->findChild<QPushButton*>("execute_button")->isEnabled()) {
      std::cout<<"PASS: slider alone stationary; Execute without Plan reached selected angle"<<std::endl;
      display->subProp("Planning Request")->subProp("Planning Group")->setValue("pallet_arm");stage=5;
    }else if(stage==5) {
      for(const auto name:{"plan_button","plan_and_execute_button"}) {
        auto button=frame->findChild<QPushButton*>(name);
        if(!button || !button->isEnabled()) {
          std::cerr<<"FAIL: arm planning button not restored: "<<name<<std::endl;app.exit(10);return;
        }
      }
      std::cout<<"PASS: gripper planning disabled; arm Plan and Plan & Execute restored"<<std::endl;
      app.exit(0);stage=6;
    }
  });timer.start(100);
  QTimer::singleShot(60000,[&](){std::cerr<<"FAIL: native Joints preview timeout stage="<<stage<<std::endl;app.exit(3);});
  int result=app.exec();
  delete frame;rclcpp::shutdown();return result;
}
