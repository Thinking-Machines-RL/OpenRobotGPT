// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotgpt_interfaces:msg/StateReward.idl
// generated code does not contain a copyright notice

#ifndef ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__BUILDER_HPP_
#define ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__BUILDER_HPP_

#include "robotgpt_interfaces/msg/detail/state_reward__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robotgpt_interfaces
{

namespace msg
{

namespace builder
{

class Init_StateReward_terminal
{
public:
  explicit Init_StateReward_terminal(::robotgpt_interfaces::msg::StateReward & msg)
  : msg_(msg)
  {}
  ::robotgpt_interfaces::msg::StateReward terminal(::robotgpt_interfaces::msg::StateReward::_terminal_type arg)
  {
    msg_.terminal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotgpt_interfaces::msg::StateReward msg_;
};

class Init_StateReward_reward
{
public:
  explicit Init_StateReward_reward(::robotgpt_interfaces::msg::StateReward & msg)
  : msg_(msg)
  {}
  Init_StateReward_terminal reward(::robotgpt_interfaces::msg::StateReward::_reward_type arg)
  {
    msg_.reward = std::move(arg);
    return Init_StateReward_terminal(msg_);
  }

private:
  ::robotgpt_interfaces::msg::StateReward msg_;
};

class Init_StateReward_info
{
public:
  explicit Init_StateReward_info(::robotgpt_interfaces::msg::StateReward & msg)
  : msg_(msg)
  {}
  Init_StateReward_reward info(::robotgpt_interfaces::msg::StateReward::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_StateReward_reward(msg_);
  }

private:
  ::robotgpt_interfaces::msg::StateReward msg_;
};

class Init_StateReward_info_keys
{
public:
  explicit Init_StateReward_info_keys(::robotgpt_interfaces::msg::StateReward & msg)
  : msg_(msg)
  {}
  Init_StateReward_info info_keys(::robotgpt_interfaces::msg::StateReward::_info_keys_type arg)
  {
    msg_.info_keys = std::move(arg);
    return Init_StateReward_info(msg_);
  }

private:
  ::robotgpt_interfaces::msg::StateReward msg_;
};

class Init_StateReward_state
{
public:
  Init_StateReward_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StateReward_info_keys state(::robotgpt_interfaces::msg::StateReward::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_StateReward_info_keys(msg_);
  }

private:
  ::robotgpt_interfaces::msg::StateReward msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotgpt_interfaces::msg::StateReward>()
{
  return robotgpt_interfaces::msg::builder::Init_StateReward_state();
}

}  // namespace robotgpt_interfaces

#endif  // ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__BUILDER_HPP_
