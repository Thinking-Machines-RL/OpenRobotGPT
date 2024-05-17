#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "robotgpt_interfaces/msg/object_states.hpp"

using std::placeholders::_1;

// Implemented as a subscribe for testin, mus be modify to client/service
class MoveitSubscriber : public rclcpp::Node
{
  public:
    MoveitSubscriber()
    : Node("moveit_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/current_position", 10, std::bind(&MoveitSubscriber::topic_callback, this, _1));

      // moveit::planning_interface::MoveGroupInterface move_group_interface(std::make_shared<rclcpp::Node>(this->get_name()), planning_group_);
      move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {}), "panda_arm");

      subscription_obj_ = this->create_subscription<robotgpt_interfaces::msg::ObjectStates>(
      "/object_position", 10, std::bind(&MoveitSubscriber::obj_callback, this, _1));
      
    }

  private:
    
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
      // requires a current state as a Joint state
      // requires a EE position as the goal state
      auto const logger = rclcpp::get_logger("hello_moveit");

      moveit_msgs::msg::RobotState c_msg;

      c_msg.joint_state = *msg;
      auto const current_pose = c_msg;
      move_group_interface->setStartState(current_pose);
      // Set a target Pose with updated values !!!
      auto const target_pose = [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.y = 0.8;
        msg.orientation.w = 0.6;
        msg.position.x = 0.1;
        msg.position.y = 0.4;
        msg.position.z = 0.4;
        return msg;
      }();
      move_group_interface->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto const success = static_cast<bool>(move_group_interface->plan(plan));
      // of plan we need: plan->trajectory_->joint_trajectory
      // http://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectory.html
      if(success) {
         std::cout << "[INFO] Plan is good" << std::endl;

      } else {
        std::cout << "[info] failed" << std::endl;
      } 

    }

    void obj_callback(const robotgpt_interfaces::msg::ObjectStates::SharedPtr msg) const
    {

      //Write in order to add the object of the envirnoment
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = move_group_interface->getPlanningFrame();
      collision_object.id = "cube1";
      shape_msgs::msg::SolidPrimitive primitive;

      // Define the size of the box in meters
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.5;
      primitive.dimensions[primitive.BOX_Y] = 0.1;
      primitive.dimensions[primitive.BOX_Z] = 0.5;

      // Define the pose of the box (relative to the frame_id)
      geometry_msgs::msg::Pose box_pose;
      box_pose.orientation.w = 1.0;
      box_pose.position.x = 0.2;
      box_pose.position.y = 0.2;
      box_pose.position.z = 0.25;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      planning_scene_interface.applyCollisionObject(collision_object);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<robotgpt_interfaces::msg::ObjectStates>::SharedPtr subscription_obj_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveitSubscriber>());
  rclcpp::shutdown();
  return 0;
}
