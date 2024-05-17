#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "sensor_msgs/msg/joint_state.hpp"
#include "robotgpt_interfaces/srv/trajectory.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "robotgpt_interfaces/msg/object_states.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

// Implemented as a subscribe for testin, mus be modify to client/service
class MoveitSubscriber : public rclcpp::Node
{
  public:
    MoveitSubscriber()
    : Node("moveit_subscriber")
    {
      // subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      // "/current_position", 10, std::bind(&MoveitSubscriber::topic_callback, this, _1));

      service_ = this->create_service<robotgpt_interfaces::srv::Trajectory>(
            "traj", 
            std::bind(&MoveitSubscriber::handle_service, this, _1, _2));

      // moveit::planning_interface::MoveGroupInterface move_group_interface(std::make_shared<rclcpp::Node>(this->get_name()), planning_group_);
      move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {}), "panda_arm");

      subscription_obj_ = this->create_subscription<robotgpt_interfaces::msg::ObjectStates>(
      "/object_position", 10, std::bind(&MoveitSubscriber::obj_callback, this, _1));
      
    }

  private:
    
    void handle_service(
        const std::shared_ptr<robotgpt_interfaces::srv::Trajectory::Request> request,
        std::shared_ptr<robotgpt_interfaces::srv::Trajectory::Response> response)
    {
        auto const logger = rclcpp::get_logger("moveit_service");

        // setting current position for moveit planning
        moveit_msgs::msg::RobotState c_msg;
        c_msg.joint_state = request->current_position;
        move_group_interface->setStartState(c_msg);

        // Set the target pose using the ending_position from the request
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = request->ending_position[0];
        target_pose.position.y = request->ending_position[1];
        target_pose.position.z = request->ending_position[2];
        target_pose.orientation.x = request->ending_position[3];
        target_pose.orientation.y = request->ending_position[4];
        target_pose.orientation.z = request->ending_position[5];
        target_pose.orientation.w = request->ending_position[6];

        move_group_interface->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface->plan(plan));

        if (success) {
            RCLCPP_INFO(logger, "Planning successful");

            // Populate the response with the plan details
            response->plan = plan.trajectory.joint_trajectory;
            response->completion_flag = true;
        } else {
            RCLCPP_WARN(logger, "Planning failed");
            response->completion_flag = false;
        }
    }

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

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      for (size_t i = 0; i < msg->objects.size(); ++i) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface->getPlanningFrame();
        collision_object.id = msg->objects[i];
        
        shape_msgs::msg::SolidPrimitive primitive;
        // add dimension of the blocks
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.5;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.5;

        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = msg->states[i].pose[0];
        box_pose.position.y = msg->states[i].pose[1];
        box_pose.position.z = msg->states[i].pose[2];
        box_pose.orientation.x = msg->states[i].pose[3];
        box_pose.orientation.y = msg->states[i].pose[4];
        box_pose.orientation.z = msg->states[i].pose[5];
        box_pose.orientation.w = msg->states[i].pose[6];

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface.applyCollisionObject(collision_object);
        std::cout << "object "<< msg->objects[i] << std::endl;
      }
    }

    //rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<robotgpt_interfaces::msg::ObjectStates>::SharedPtr subscription_obj_;
    rclcpp::Service<robotgpt_interfaces::srv::Trajectory>::SharedPtr service_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveitSubscriber>());
  rclcpp::shutdown();
  return 0;
}
