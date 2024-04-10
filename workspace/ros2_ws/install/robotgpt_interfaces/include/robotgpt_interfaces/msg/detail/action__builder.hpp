// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotgpt_interfaces:msg/Action.idl
// generated code does not contain a copyright notice

#ifndef ROBOTGPT_INTERFACES__MSG__DETAIL__ACTION__BUILDER_HPP_
#define ROBOTGPT_INTERFACES__MSG__DETAIL__ACTION__BUILDER_HPP_

#include "robotgpt_interfaces/msg/detail/action__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robotgpt_interfaces
{

namespace msg
{

namespace builder
{

class Init_Action_action
{
public:
  Init_Action_action()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotgpt_interfaces::msg::Action action(::robotgpt_interfaces::msg::Action::_action_type arg)
  {
    msg_.action = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotgpt_interfaces::msg::Action msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotgpt_interfaces::msg::Action>()
{
  return robotgpt_interfaces::msg::builder::Init_Action_action();
}

}  // namespace robotgpt_interfaces

#endif  // ROBOTGPT_INTERFACES__MSG__DETAIL__ACTION__BUILDER_HPP_
