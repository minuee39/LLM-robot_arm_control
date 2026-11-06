#include <moveit/robot_model/robot_model.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QFile>
#include <fstream>
#include <sstream>
#include <iostream>
std::string read(const std::string&p){std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){rclcpp::init(argc,argv);std::string base=ament_index_cpp::get_package_share_directory("pallet_physical_moveit_config")+"/config/";auto u=urdf::parseURDF(read(base+"pallet_adaptive_control.urdf"));auto s=std::make_shared<srdf::Model>();s->initString(*u,read(base+"pallet_adaptive.srdf"));auto m=std::make_shared<moveit::core::RobotModel>(u,s);planning_scene::PlanningScene scene(m);auto&state=scene.getCurrentStateNonConst();state.setToDefaultValues();QFile f(QString::fromStdString(base+"adaptive_preview_states.json"));f.open(QIODevice::ReadOnly);auto j=QJsonDocument::fromJson(f.readAll()).object();auto names=j["joint_names"].toArray();std::map<std::pair<std::string,std::string>,int> counts;int poses=0;for(auto row:j["positions"].toArray()){auto vals=row.toArray();for(int i=0;i<names.size();++i)state.setVariablePosition(names[i].toString().toStdString(),vals[i].toDouble());state.update();collision_detection::CollisionRequest req;req.contacts=true;req.max_contacts=1000;req.max_contacts_per_pair=1;collision_detection::CollisionResult res;scene.checkSelfCollision(req,res,state);for(auto&p:res.contacts)counts[p.first]++;poses++;}for(auto&p:counts)std::cout<<"PAIR "<<p.first.first<<" "<<p.first.second<<" "<<p.second<<"/"<<poses<<std::endl;std::cout<<(counts.empty()?"PASS":"FAIL")<<": self collision sweep, "<<poses<<" coupled poses [-12,12] rad"<<std::endl;rclcpp::shutdown();return counts.empty() && poses==481 ? 0 : 1;}
